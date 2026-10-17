#include "route_follow.h"
#include "app_control.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

/* 默认路线：(0,0) -> (20,40) -> (50,80)，可在菜单 Route 中修改。
 * 坐标是 Navigation/Zero 坐标系下的 cm；先静止 Zero，再开 Run。 */
bool route_run_flag;
volatile float route_blend_cm = ROUTE_BLEND_DEFAULT_CM;
volatile float route_flow_cm = ROUTE_FLOW_DEFAULT_CM;
volatile int16_t route_node_count = ROUTE_DEFAULT_NODE_COUNT;
volatile int32_t route_state = ROUTE_IDLE;
volatile int32_t route_current_idx;
volatile int32_t route_flow_mode, route_stop_reason;
volatile float route_vref_cmps, route_error_cm;
route_node_t route_nodes[ROUTE_MAX_NODES] = ROUTE_DEFAULT_NODES;

/* 切入位置模式后等待的控制周期数：一拍被模式切换逻辑消耗，再留一拍稳定 */
#define ROUTE_ARM_SETTLE_TICKS 2U

static uint32 route_arm_ticks;

/* Fixed storage, no heap/new Keil units. Cubic through-node curves are
 * represented by short chords, with shared tangent handles at nodes. */
#define CURVE_STEPS 16U
#define PATH_CAPACITY (2U + ROUTE_MAX_NODES * (CURVE_STEPS + 1U))
typedef struct {
    float x, y, yaw, s, curvature, limit;
    uint8 node, curved;
} route_sample_t;
static route_sample_t path[PATH_CAPACITY];
static unsigned path_count, path_segment;
static uint8 path_stop_idx;
static float path_progress, path_blend, path_flow;
static bool path_ready, profile_ready, path_invalid, continuous, terminal;
static position_config_t path_config;

static float wrap(float a)
{
    a = fmodf(a, 360.0f);
    if (a > 180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}
static float yaw_lerp(float a, float b, float t) { return wrap(a + wrap(b-a)*t); }
static void path_reset(void)
{
    path_ready = profile_ready = path_invalid = continuous = terminal = false;
    path_count = path_segment = 0U;
    path_progress = 0.0f;
    route_flow_mode = ROUTE_MODE_WAIT;
    route_stop_reason = ROUTE_STOP_NONE;
    route_vref_cmps = route_error_cm = 0.0f;
}

void route_follow_init(void)
{
    route_run_flag = false;
    route_state = ROUTE_IDLE;
    route_current_idx = 0;
    route_arm_ticks = 0U;
    path_reset();
}

static uint8 route_count_clamped(void)
{
    int16_t count = route_node_count;

    if (count < 1) count = 1;
    if (count > ROUTE_MAX_NODES) count = ROUTE_MAX_NODES;
    return (uint8)count;
}

static void route_stop_internal(void)
{
    motor_run_enabled = false;
    route_run_flag = false;
    route_state = ROUTE_IDLE;
}

void route_follow_start(void)
{
    /* 运行中重复置 On 不重启 */
    if (route_state == ROUTE_ARMING || route_state == ROUTE_RUNNING ||
        route_state == ROUTE_ADVANCE)
    {
        return;
    }

    route_node_count = (int16_t)route_count_clamped();
    route_current_idx = 0;
    route_arm_ticks = 0U;
    path_reset();
    route_state = ROUTE_ARMING;
    route_run_flag = true;
}

void route_follow_stop(void)
{
    route_stop_internal();
}

void route_follow_config_edited(void)
{
    if (route_state == ROUTE_ARMING || route_state == ROUTE_RUNNING ||
        route_state == ROUTE_ADVANCE)
    {
        route_stop_internal();
    }
}

uint8 route_follow_is_node_field(const void *ptr)
{
    const uint8 *p = (const uint8 *)ptr;

    return (p >= (const uint8 *)route_nodes &&
            p < (const uint8 *)(route_nodes + ROUTE_MAX_NODES)) ? 1U : 0U;
}

float route_follow_clamp_node_field(const void *field, float value)
{
    size_t offset = (size_t)((const uint8 *)field - (const uint8 *)route_nodes);

    if (!route_follow_is_node_field(field))
    {
        return value;
    }
    if (offset % sizeof(route_node_t) == offsetof(route_node_t, yaw_deg))
    {
        if (value > 180.0f) value = 180.0f;
        if (value < -180.0f) value = -180.0f;
    }
    else
    {
        if (value > 10000.0f) value = 10000.0f;
        if (value < -10000.0f) value = -10000.0f;
    }
    return value;
}

static void route_set_goal(uint8 idx)
{
    motor_position_goal.x_cm = route_nodes[idx].x_cm;
    motor_position_goal.y_cm = route_nodes[idx].y_cm;
    motor_position_goal.yaw_deg = route_nodes[idx].yaw_deg;
}

/* Cubic curvature in inverse cm; a vanishing tangent is a cusp. */
static float cubic_curvature(float t, float ax, float ay, float bx, float by,
                             float cx, float cy, float dx, float dy)
{
    float u=1.0f-t;
    float vx=3.0f*(u*u*(bx-ax)+2.0f*u*t*(cx-bx)+t*t*(dx-cx));
    float vy=3.0f*(u*u*(by-ay)+2.0f*u*t*(cy-by)+t*t*(dy-cy));
    float qx=6.0f*(u*(cx-2.0f*bx+ax)+t*(dx-2.0f*cx+bx));
    float qy=6.0f*(u*(cy-2.0f*by+ay)+t*(dy-2.0f*cy+by));
    float v=hypotf(vx,vy);
    return v > 0.00001f ? fabsf(vx*qy-vy*qx)/(v*v*v) : 1.0e6f;
}
/* A process node is interpolated exactly. Its tangent is the incoming leg:
 * retain that direction at ordinary corners, then turn on the next leg.
 * Obtuse direction changes share the turn across both sides of the node
 * to avoid a tight hook immediately after it. Never chase a passed node.
 * Yaw-only legs and exact reversals stop only their local block. */
static bool build_path(const navigation_snapshot_t *pose, const position_config_t *config)
{
    route_node_t points[ROUTE_MAX_NODES + 1U];
    uint8 indices[ROUTE_MAX_NODES + 1U];
    unsigned n = 1U, count = route_count_clamped();
    float allowance = route_blend_cm;
    path_count = 0U;
    path_stop_idx = (uint8)(count-1U);
    route_stop_reason = allowance == 0.0f ? ROUTE_STOP_DISABLED : ROUTE_STOP_NONE;
    if (!isfinite(allowance) || allowance <= 0.0f || allowance > 20.0f ||
        !isfinite(route_flow_cm) || route_flow_cm < 1.0f || route_flow_cm > 100.0f ||
        !pose->valid || !pose->bias_ready || pose->status != NAV_RUNNING) return false;
    points[0] = (route_node_t){pose->x_m*100.0f,pose->y_m*100.0f,pose->yaw_deg};
    indices[0] = 0U;
    for (unsigned i = (unsigned)route_current_idx; i < count; ++i) {
        route_node_t b = route_nodes[i];
        if (!isfinite(b.x_cm) || !isfinite(b.y_cm) || !isfinite(b.yaw_deg) ||
            fabsf(b.x_cm) > 10000.0f || fabsf(b.y_cm) > 10000.0f ||
            fabsf(b.yaw_deg) > 180.0f) { path_invalid = true; return false; }
        float distance = hypotf(b.x_cm-points[n-1U].x_cm,b.y_cm-points[n-1U].y_cm);
        float yaw_error = fabsf(wrap(b.yaw_deg-points[n-1U].yaw_deg));
        /* A stored start point already satisfied by the measured pose is
         * not a tiny new leg or a mandatory yaw-only stop. */
        if (n == 1U && i+1U < count && distance <= config->xy_tolerance_cm &&
            yaw_error <= config->yaw_tolerance_deg) {
            indices[0] = (uint8)i;
            continue;
        }
        if (n == 1U && distance <= config->xy_tolerance_cm &&
            yaw_error > config->yaw_tolerance_deg) {
            route_stop_reason = ROUTE_STOP_YAW;
            path_stop_idx = (uint8)i;
            break;
        }
        if (distance < 0.01f) {
            if (yaw_error > config->yaw_tolerance_deg) {
                route_stop_reason = ROUTE_STOP_YAW;
                path_stop_idx = n == 1U ? (uint8)i : indices[n-1U];
                break;
            }
            indices[n-1U] = (uint8)i;
            points[n-1U].yaw_deg = b.yaw_deg;
            continue;
        }
        points[n] = b; indices[n++] = (uint8)i;
    }
    if (n < 2U) {
        if (route_stop_reason == ROUTE_STOP_NONE) route_stop_reason = ROUTE_STOP_SINGLE;
        return false;
    }
    path[path_count++] = (route_sample_t){points[0].x_cm,points[0].y_cm,
        wrap(points[0].yaw_deg),0,0,0,indices[0],0};
    float tx[ROUTE_MAX_NODES+1U], ty[ROUTE_MAX_NODES+1U];
    float handles[ROUTE_MAX_NODES+1U];
    for (unsigned i=0U; i<n; ++i) {
        unsigned j=i ? i-1U : 0U, k=i ? i : 1U;
        float dx=points[k].x_cm-points[j].x_cm;
        float dy=points[k].y_cm-points[j].y_cm;
        float len=hypotf(dx,dy);
        tx[i]=dx/len; ty[i]=dy/len;
        float adjacent=len;
        if (i+1U<n) {
            dx=points[i+1U].x_cm-points[i].x_cm;
            dy=points[i+1U].y_cm-points[i].y_cm;
            float next=hypotf(dx,dy);
            if (i && (tx[i]*dx+ty[i]*dy)/next <= -0.9999f) {
                n = i+1U;
                path_stop_idx = indices[i];
                route_stop_reason = ROUTE_STOP_REVERSE;
                handles[i] = fminf(route_flow_cm,0.3f*len);
                break;
            }
            adjacent=fminf(adjacent,next);
        }
        handles[i]=fminf(route_flow_cm,0.3f*adjacent);
    }
    for (unsigned i=1U; i+1U<n; ++i) {
        float ox=points[i+1U].x_cm-points[i].x_cm;
        float oy=points[i+1U].y_cm-points[i].y_cm;
        float length=hypotf(ox,oy); ox/=length; oy/=length;
        float dot=tx[i]*ox+ty[i]*oy;
        float weight=0.5f*fmaxf(0.0f,-dot);
        float x=(1.0f-weight)*tx[i]+weight*ox;
        float y=(1.0f-weight)*ty[i]+weight*oy;
        length=hypotf(x,y); tx[i]=x/length; ty[i]=y/length;
    }
    for (unsigned i=1U; i<n; ++i) {
        route_node_t a=points[i-1U], d=points[i];
        float bx=a.x_cm+handles[i-1U]*tx[i-1U];
        float by=a.y_cm+handles[i-1U]*ty[i-1U];
        float cx=d.x_cm-handles[i]*tx[i], cy=d.y_cm-handles[i]*ty[i];
        for (unsigned j=1U; j<=CURVE_STEPS; ++j) {
            float t=(float)j/CURVE_STEPS, u=1.0f-t, curvature=0.0f;
            /* Sample within each chord, conservatively sharing its bound
             * between both endpoints for the braking pass. */
            for (unsigned k=0U; k<=4U; ++k) {
                float q=((float)(j-1U)+0.25f*k)/CURVE_STEPS;
                curvature=fmaxf(curvature,cubic_curvature(q,a.x_cm,a.y_cm,bx,by,cx,cy,d.x_cm,d.y_cm));
            }
            path[path_count-1U].curvature=fmaxf(path[path_count-1U].curvature,curvature);
            path[path_count++]=(route_sample_t){
                u*u*u*a.x_cm+3.0f*u*u*t*bx+3.0f*u*t*t*cx+t*t*t*d.x_cm,
                u*u*u*a.y_cm+3.0f*u*u*t*by+3.0f*u*t*t*cy+t*t*t*d.y_cm,
                yaw_lerp(a.yaw_deg,d.yaw_deg,t),0,curvature,0,indices[i],
                curvature>0.000001f ? 1U : 0U};
        }
    }
    for (unsigned i = 1U; i < path_count; ++i)
        path[i].s = path[i-1U].s+hypotf(path[i].x-path[i-1U].x,path[i].y-path[i-1U].y);
    return true;
}

static float route_diag(const position_config_t *c)
{
    return isfinite(c->diag_speed_k) ? fmaxf(0.2f,fminf(1.0f,c->diag_speed_k)) : 1.0f;
}
static float braking(const position_config_t *c)
{
    return fminf(c->brake_ceiling_cmps2,c->max_accel_cmps2*route_diag(c)*c->brake_limit);
}
/* Backward speed pass anticipates every curve and the final stop. Curves
 * use half the acceleration budget; the vector limiter enforces the total. */
static void speed_profile(const position_config_t *c)
{
    float brake = braking(c);
    float acc = 0.5f*fminf(c->max_accel_cmps2*route_diag(c),brake);
    for (unsigned i = 0U; i < path_count; ++i) {
        float limit = c->max_speed_cmps*route_diag(c);
        if (path[i].curvature > 0.000001f) limit = fminf(limit,sqrtf(acc/path[i].curvature));
        path[i].limit = limit;
    }
    for (unsigned i = 1U; i < path_count; ++i) {
        float ds = path[i].s-path[i-1U].s;
        float dyaw = fabsf(wrap(path[i].yaw-path[i-1U].yaw))*0.01745329252f;
        if (dyaw > 0.000001f) {
            float limit = 0.7f*c->max_omega_radps*ds/dyaw;
            path[i].limit = fminf(path[i].limit,limit);
            path[i-1U].limit = fminf(path[i-1U].limit,limit);
        }
    }
    path[path_count-1U].limit = 0.0f;
    for (unsigned i = path_count-1U; i > 0U; --i) {
        float ds = path[i].s-path[i-1U].s;
        path[i-1U].limit = fminf(path[i-1U].limit,sqrtf(path[i].limit*path[i].limit+2.0f*brake*ds));
    }
}

void route_follow_control_update(position_control_t *control,
                                 const position_config_t *config,
                                 const position_goal_t *goal,
                                 const navigation_snapshot_t *pose, float dt)
{
    route_vref_cmps = route_error_cm = 0.0f;
    if (route_state != ROUTE_RUNNING || !route_run_flag) {
        position_control_update(control,config,goal,pose,dt);
        return;
    }
    if (!isfinite(route_blend_cm) || route_blend_cm < 0.0f || route_blend_cm > 20.0f ||
        (route_blend_cm > 0.0f && route_blend_cm < ROUTE_BLEND_MIN_CM) ||
        !isfinite(route_flow_cm) || route_flow_cm < 1.0f || route_flow_cm > 100.0f) {
        position_control_reset(control);
        control->output.status = POSITION_BAD_CONFIG;
        return;
    }
    if (!path_ready) {
        continuous = build_path(pose,config);
        route_flow_mode = continuous ? ROUTE_MODE_CURVE :
            (route_blend_cm == 0.0f ? ROUTE_MODE_POINT : ROUTE_MODE_SETTLE);
        path_ready = true;
        path_config = *config;
        path_blend = route_blend_cm;
        path_flow = route_flow_cm;
    }
    if (path_invalid) {
        position_control_reset(control);
        control->output.status = POSITION_BAD_CONFIG;
        return;
    }
    if (!continuous || terminal) {
        position_goal_t target = *goal;
        if (!continuous && route_blend_cm > 0.0f) {
            route_current_idx = path_stop_idx;
            route_set_goal(path_stop_idx);
            target = motor_position_goal;
        }
        position_control_update(control,config,&target,pose,dt);
        route_vref_cmps = hypotf(control->output.vx_cmps,control->output.vy_cmps);
        return;
    }
    /* Validate with the shared controller before projection arithmetic. */
    position_control_t validated = *control;
    position_control_track(&validated,config,goal,pose,0.0f,0.0f,dt);
    if (validated.output.status < 0) { *control = validated; return; }
    if (memcmp(&path_config,config,sizeof(*config)) != 0 || route_blend_cm != path_blend || route_flow_cm != path_flow) {
        position_control_reset(control);
        control->output.status = POSITION_BAD_CONFIG;
        return;
    }
    if (!profile_ready) { speed_profile(config); profile_ready = true; }
    float px = pose->x_m*100.0f, py = pose->y_m*100.0f;
    float best = 1.0e30f, progress = path_progress;
    float window = fmaxf(2.0f,2.0f*config->max_speed_cmps*dt);
    unsigned seg = path_segment;
    /* Local monotone projection avoids jumping ahead at self intersections. */
    for (unsigned i = path_segment; i+1U < path_count; ++i) {
        route_sample_t a = path[i], b = path[i+1U];
        if (a.s > path_progress+window) break;
        float dx = b.x-a.x, dy = b.y-a.y, ds = b.s-a.s;
        if (ds < 0.00001f) continue;
        float t = ((px-a.x)*dx+(py-a.y)*dy)/(ds*ds);
        float low = fmaxf(0.0f,(path_progress-a.s)/ds);
        float high = fminf(1.0f,(path_progress+window-a.s)/ds);
        t = fmaxf(low,fminf(high,t));
        float ex = px-a.x-t*dx, ey = py-a.y-t*dy;
        float error = ex*ex+ey*ey;
        if (error < best) { best=error; progress=a.s+t*ds; seg=i; }
    }
    path_progress = progress; path_segment = seg;
    route_sample_t a = path[seg], b = path[seg+1U];
    float ds = b.s-a.s, t = fmaxf(0.0f,fminf(1.0f,(progress-a.s)/ds));
    float remaining = path[path_count-1U].s-progress;
    if (seg+2U == path_count && remaining <= fmaxf(3.0f,2.0f*config->xy_tolerance_cm)) {
        terminal = true;
        route_flow_mode = ROUTE_MODE_SETTLE;
        route_current_idx = path_stop_idx;
        route_set_goal((uint8)route_current_idx);
        position_goal_t final_goal = motor_position_goal;
        /* Handoff without resetting translation/yaw velocity states. */
        control->previous_goal = final_goal;
        control->initial_distance_cm = hypotf(final_goal.x_cm-px,final_goal.y_cm-py);
        control->have_initial = true;
        control->diag_mix = 0.0f;
        position_control_update(control,config,&final_goal,pose,dt);
        route_vref_cmps = hypotf(control->output.vx_cmps,control->output.vy_cmps);
        return;
    }
    route_current_idx = b.node;
    float speed = fminf(config->max_speed_cmps*route_diag(config),
        fminf(sqrtf(a.limit*a.limit+2.0f*config->max_accel_cmps2*route_diag(config)*(progress-a.s)),
              sqrtf(b.limit*b.limit+2.0f*braking(config)*(b.s-progress))));
    if (b.curved) speed = fminf(speed,sqrtf(0.5f*fminf(config->max_accel_cmps2*route_diag(config),braking(config))/b.curvature));
    float dyaw = fabsf(wrap(b.yaw-a.yaw))*0.01745329252f;
    if (dyaw > 0.000001f) speed = fminf(speed,0.7f*config->max_omega_radps*ds/dyaw);
    speed = fminf(speed,control->plan_v_cmps+config->max_accel_cmps2*route_diag(config)*dt);
    float ex = a.x+t*(b.x-a.x)-px, ey = a.y+t*(b.y-a.y)-py;
    /* Progress is monotone. After a disturbance its reference can remain
     * ahead of/behind the car; never let that tangential error command a
     * return to a passed reference. Correct only the cross-track error. */
    float tx = (b.x-a.x)/ds, ty = (b.y-a.y)/ds;
    float along_error = ex*tx+ey*ty;
    ex -= along_error*tx; ey -= along_error*ty;
    /* Correct laterally without a second error-dependent speed collapse.
     * Reserve at least 80% of the planned speed for forward progress.
     * A smaller tolerance increases correction gain within this budget. */
    float error = hypotf(ex,ey);
    float gain = 4.0f*fmaxf(0.25f,fminf(4.0f,ROUTE_BLEND_DEFAULT_CM/route_blend_cm));
    float correction = fminf(gain*error,0.6f*speed);
    float forward = sqrtf(fmaxf(0.0f,speed*speed-correction*correction));
    float factor = error > 0.00001f ? correction/error : 0.0f;
    float vx=forward*tx+factor*ex, vy=forward*ty+factor*ey;
    route_error_cm = error;
    route_vref_cmps = speed;
    position_goal_t ref = {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),yaw_lerp(a.yaw,b.yaw,t)};
    position_control_track(control,config,&ref,pose,vx,vy,dt);
}

void route_follow_tick_10ms(void)
{
    position_output_t snapshot;
    uint8 count;
    int32_t state = route_state;

    if (state == ROUTE_IDLE || state == ROUTE_DONE || state == ROUTE_FAULT)
    {
        return;
    }

    count = route_count_clamped();

    if (state == ROUTE_ARMING)
    {
        /* 强制位置模式：模式切换那一拍 app_control 会清零 Run，
         * 等切换完成后再写目标并开 Run。 */
        motor_pwm_test_enabled = false;
        motor_position_enabled = true;
        motor_run_enabled = false;
        route_arm_ticks++;
        if (route_arm_ticks >= ROUTE_ARM_SETTLE_TICKS)
        {
            route_set_goal(0U);
            motor_run_enabled = true;
            route_state = ROUTE_RUNNING;
        }
        return;
    }

    if (state == ROUTE_ADVANCE)
    {
        /* 上一拍位置环已到点并自动关 Run；这一拍换新目标并重新开 Run */
        path_reset(); /* Replan after a local stop, never switch the whole route to stop mode. */
        route_set_goal((uint8)route_current_idx);
        motor_run_enabled = true;
        route_state = ROUTE_RUNNING;
        return;
    }

    /* ROUTE_RUNNING */
    if (!motor_position_enabled || motor_pwm_test_enabled)
    {
        /* 运行中被手动切走模式：放弃路线 */
        route_run_flag = false;
        route_state = ROUTE_IDLE;
        return;
    }

    app_control_get_position_snapshot(&snapshot);
    if (snapshot.status == POSITION_REACHED)
    {
        if (route_current_idx + 1 >= count)
        {
            /* 终点到达，app_control 已关 Run 并清零输出 */
            route_run_flag = false;
            route_state = ROUTE_DONE;
        }
        else
        {
            route_current_idx++;
            route_state = ROUTE_ADVANCE;
        }
        return;
    }
    if (snapshot.status < 0)
    {
        /* 定位无效或参数非法：停车并报错，重新 Run 可再次启动 */
        motor_run_enabled = false;
        route_run_flag = false;
        route_state = ROUTE_FAULT;
        return;
    }
    if (!motor_run_enabled)
    {
        /* 非到点/非故障却被关了 Run（手动关闭等）：取消路线 */
        route_run_flag = false;
        route_state = ROUTE_IDLE;
    }
}
