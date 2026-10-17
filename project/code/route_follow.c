#include "route_follow.h"
#include "app_control.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

/* 默认路线：(0,0) -> (20,40) -> (50,80)，可在菜单 Route 中修改。
 * 坐标是 Navigation/Zero 坐标系下的 cm；先静止 Zero，再开 Run。 */
bool route_run_flag;
volatile float route_blend_cm = ROUTE_BLEND_DEFAULT_CM;
volatile int16_t route_node_count = ROUTE_DEFAULT_NODE_COUNT;
volatile int32_t route_state = ROUTE_IDLE;
volatile int32_t route_current_idx;
route_node_t route_nodes[ROUTE_MAX_NODES] = ROUTE_DEFAULT_NODES;

/* 切入位置模式后等待的控制周期数：一拍被模式切换逻辑消耗，再留一拍稳定 */
#define ROUTE_ARM_SETTLE_TICKS 2U

static uint32 route_arm_ticks;

/* Fixed storage, no heap/new Keil units. Tangent-continuous quadratic
 * Bezier corners are represented by 16 short chords. */
#define CURVE_STEPS 16U
#define PATH_CAPACITY (2U + ROUTE_MAX_NODES * (CURVE_STEPS + 1U))
typedef struct {
    float x, y, yaw, s, curvature, limit;
    uint8 node, curved;
} route_sample_t;
static route_sample_t path[PATH_CAPACITY];
static unsigned path_count, path_segment;
static float path_progress, path_blend;
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

/* Zero-length yaw-only legs and turns >150 degrees retain stop-at-node
 * behavior for the whole route; do not round a cusp or skip a rotation. */
static bool build_path(const navigation_snapshot_t *pose)
{
    route_node_t points[ROUTE_MAX_NODES + 1U];
    uint8 indices[ROUTE_MAX_NODES + 1U];
    unsigned n = 1U, count = route_count_clamped();
    float allowance = route_blend_cm;
    path_count = 0U;
    if (!isfinite(allowance) || allowance <= 0.0f || allowance > 20.0f ||
        !pose->valid || !pose->bias_ready || pose->status != NAV_RUNNING) return false;
    points[0] = (route_node_t){pose->x_m*100.0f,pose->y_m*100.0f,pose->yaw_deg};
    indices[0] = 0U;
    for (unsigned i = 0; i < count; ++i) {
        route_node_t b = route_nodes[i];
        if (!isfinite(b.x_cm) || !isfinite(b.y_cm) || !isfinite(b.yaw_deg) ||
            fabsf(b.x_cm) > 10000.0f || fabsf(b.y_cm) > 10000.0f ||
            fabsf(b.yaw_deg) > 180.0f) { path_invalid = true; return false; }
        if (hypotf(b.x_cm-points[n-1U].x_cm,b.y_cm-points[n-1U].y_cm) < 0.01f) {
            if (fabsf(wrap(b.yaw_deg-points[n-1U].yaw_deg)) > 0.01f) return false;
            indices[n-1U] = (uint8)i;
            continue;
        }
        points[n] = b; indices[n++] = (uint8)i;
    }
    if (n < 2U) return false;
    path[path_count++] = (route_sample_t){points[0].x_cm,points[0].y_cm,
        wrap(points[0].yaw_deg),0,0,0,indices[0],0};
    for (unsigned i = 1U; i < n; ++i) {
        route_node_t a = points[i-1U], b = points[i];
        float ix = b.x_cm-a.x_cm, iy = b.y_cm-a.y_cm;
        float in_len = hypotf(ix,iy), trim = 0.0f, curvature = 0.0f;
        float ox = 0.0f, oy = 0.0f, out_len = 1.0f;
        ix /= in_len; iy /= in_len;
        if (i+1U < n) {
            ox = points[i+1U].x_cm-b.x_cm; oy = points[i+1U].y_cm-b.y_cm;
            out_len = hypotf(ox,oy); ox /= out_len; oy /= out_len;
            float dot = fmaxf(-1.0f,fminf(1.0f,ix*ox+iy*oy));
            if (dot < -0.8660254f) return false;
            if (dot < 0.99999f) {
                float sin_half = sqrtf(0.5f*(1.0f-dot));
                /* Midpoint distance to node = trim*sin(theta/2)/2.
                 * Use half the allowance as geometry, leaving tracking margin.
                 * Adjacent trims consume <=80% of a shared leg. */
                trim = fminf(allowance/sin_half,0.4f*fminf(in_len,out_len));
                curvature = sin_half/(trim*0.5f*(1.0f+dot));
            }
        }
        float entry_x = b.x_cm-trim*ix, entry_y = b.y_cm-trim*iy;
        float entry_yaw = yaw_lerp(a.yaw_deg,b.yaw_deg,1.0f-trim/in_len);
        path[path_count++] = (route_sample_t){entry_x,entry_y,entry_yaw,0,curvature,0,indices[i],0};
        if (trim > 0.0f) {
            float exit_x = b.x_cm+trim*ox, exit_y = b.y_cm+trim*oy;
            float exit_yaw = yaw_lerp(b.yaw_deg,points[i+1U].yaw_deg,trim/out_len);
            for (unsigned j = 1U; j <= CURVE_STEPS; ++j) {
                float t = (float)j/CURVE_STEPS, u = 1.0f-t;
                path[path_count++] = (route_sample_t){
                    u*u*entry_x+2.0f*u*t*b.x_cm+t*t*exit_x,
                    u*u*entry_y+2.0f*u*t*b.y_cm+t*t*exit_y,
                    yaw_lerp(yaw_lerp(entry_yaw,b.yaw_deg,t),yaw_lerp(b.yaw_deg,exit_yaw,t),t),
                    0,curvature,0,indices[i],1};
            }
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
    float acc = 0.5f*c->max_accel_cmps2*route_diag(c), brake = braking(c);
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
    if (route_state != ROUTE_RUNNING || !route_run_flag) {
        position_control_update(control,config,goal,pose,dt);
        return;
    }
    if (!isfinite(route_blend_cm) || route_blend_cm < 0.0f || route_blend_cm > 20.0f ||
        (route_blend_cm > 0.0f && route_blend_cm < ROUTE_BLEND_MIN_CM)) {
        position_control_reset(control);
        control->output.status = POSITION_BAD_CONFIG;
        return;
    }
    if (!path_ready) {
        continuous = build_path(pose);
        path_ready = true;
        path_config = *config;
        path_blend = route_blend_cm;
    }
    if (path_invalid) {
        position_control_reset(control);
        control->output.status = POSITION_BAD_CONFIG;
        return;
    }
    if (!continuous || terminal) {
        position_control_update(control,config,goal,pose,dt);
        return;
    }
    /* Validate with the shared controller before projection arithmetic. */
    position_control_t validated = *control;
    position_control_track(&validated,config,goal,pose,0.0f,0.0f,dt);
    if (validated.output.status < 0) { *control = validated; return; }
    if (memcmp(&path_config,config,sizeof(*config)) != 0 || route_blend_cm != path_blend) {
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
        route_current_idx = (int32_t)route_count_clamped()-1;
        route_set_goal((uint8)route_current_idx);
        position_goal_t final_goal = motor_position_goal;
        /* Handoff without resetting translation/yaw velocity states. */
        control->previous_goal = final_goal;
        control->initial_distance_cm = hypotf(final_goal.x_cm-px,final_goal.y_cm-py);
        control->have_initial = true;
        control->diag_mix = 0.0f;
        position_control_update(control,config,&final_goal,pose,dt);
        return;
    }
    route_current_idx = b.node;
    float speed = fminf(config->max_speed_cmps*route_diag(config),
        fminf(sqrtf(a.limit*a.limit+2.0f*config->max_accel_cmps2*route_diag(config)*(progress-a.s)),
              sqrtf(b.limit*b.limit+2.0f*braking(config)*(b.s-progress))));
    if (b.curved) speed = fminf(speed,sqrtf(0.5f*config->max_accel_cmps2*route_diag(config)/b.curvature));
    float dyaw = fabsf(wrap(b.yaw-a.yaw))*0.01745329252f;
    if (dyaw > 0.000001f) speed = fminf(speed,0.7f*config->max_omega_radps*ds/dyaw);
    speed = fminf(speed,control->plan_v_cmps+config->max_accel_cmps2*route_diag(config)*dt);
    float ex = a.x+t*(b.x-a.x)-px, ey = a.y+t*(b.y-a.y)-py;
    float vx = speed*(b.x-a.x)/ds+4.0f*ex, vy = speed*(b.y-a.y)/ds+4.0f*ey;
    float norm = hypotf(vx,vy);
    if (norm > speed && norm > 0.00001f) { vx*=speed/norm; vy*=speed/norm; }
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
