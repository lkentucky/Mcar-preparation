/** ivision 风格定点控制（替代版实现）
 *
 * 设计来源：ivision project/Algorithm/MotionControl.cpp
 *   - Trajectory::velocity_planning_1d（平移一维速度规划）
 *   - YawProfiled::calculate（航向角速度规划）
 *   - PathLineFollower::filter_direction（方向一阶滤波）
 *
 * 控制链与原位置 PD 版本的根本区别：整条链路不存在 XY 位置反馈。
 * 慢环按"剩余距离→速度"做开环规划（sqrt 制动曲线 + 加减速斜坡 +
 * 接近区双段刹车 + 容差圈内弹簧静止死区），输出世界系期望速度，
 * 经方向滤波旋到车体系后交给 Drive/四轮速度内环跟踪；到位与否由
 * 容差带 + 静止检测 + 沉降时间判定（与原状态机一致，REACHED 后
 * app_control 照常停车）。航向用远端 sqrt 时间最优曲线、近端有界
 * 线性带与陀螺阻尼，替代原 yaw_kp 比例环。
 *
 * 不引入的东西（保持与原模块同样的边界）：无前馈外推、无距离补偿表、
 * 不按时间判到达、不做路径线前瞻——这是"定点"模块，只负责逐点行驶。 */

#include "position_control.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define POSITION_SETTLE_TIME_S 0.20f
#define POSITION_REST_SPEED_CMPS 3.0f
#define POSITION_REST_OMEGA_RADPS 0.10f
/* Heading hold is tighter than arrival tolerance, including while translating. */
#define POSITION_HEADING_HOLD_MAX_DEG 0.5f
#define POSITION_YAW_RATE_TAU_S 0.03f

/* ---------------- 规划参数默认值（ivision tuning 默认，适配 Mcar） ----------------
 * 以下宏只是兜底默认：实际取值来自 position_config_t 对应字段，可在车上
 * Position 菜单实时修改、随 SaveCfg 掉电保存。字段越界/NaN 时回退到宏值。 */
/* 刹车能力与加速解耦：min(max_accel×BRAKE_LIMIT, 附着上限)。调高 max_accel
 * 不会让规划误以为刹车也变强（ivision brake_limit / brake_acc_ceiling）。 */
#define BRAKE_LIMIT 0.90f
#define BRAKE_ACC_CEILING_CMPS2 450.0f
/* 短段起步加速提升：锁定段长 <= SHORT_SEGMENT_CM 时，只抬高加速斜坡。 */
#define SHORT_SEGMENT_CM 40.0f
#define SHORT_BOOST_GAIN 1.40f
/* 末端接近区双段 sqrt：进入 zone 换更缓曲线提前减速，外段按正常刹车强度
 * 衔接（C0 连续）。zone 同时受锁定段长×APPROACH_RATIO 封顶。 */
#define APPROACH_ZONE_CM 10.0f
#define APPROACH_RATIO 0.25f
#define APPROACH_ACC_K 0.45f
/* ---------------- 航向规划参数（ivision yaw 默认值） ---------------- */
#define YAW_LIN_BAND_RAD 0.22f  /* 线性带半宽，保证近端等效增益有界 */
#define YAW_KD 0.22f            /* 静止转向陀螺阻尼 */
#define YAW_KD_TRANSLATE 0.36f  /* 平移时陀螺阻尼 */
#define YAW_TRANSLATE_GAIN 1.0f
#define YAW_STICTION_RADPS 0.15f /* 静摩擦角速度地板默认值（菜单 YawStic 可改，0=关闭） */
/* 容差死区内保留的陀螺阻尼双地板：实测角速度低于 MIN_RATE 视为静止不产生
 * 阻尼；产生的阻尼低于 MIN_RADPS 时过不了轮死区门槛（±2 counts）执行不出来，
 * 反而可能引起 PWM 抖动——两者都归零，保证静止时 cmd_zero 严格成立，
 * 沉降到达判定不受影响。 */
#define YAW_DEADBAND_DAMP_MIN_RATE_RADPS 0.02f
#define YAW_DEADBAND_DAMP_MIN_RADPS 0.05f
/* ---------------- 末端蠕行（替代"进容差圈即砍零"） ----------------
 * 进容差圈后保持 CRAWL 速度爬向目标，到 CUTOFF 半径才切断命令。
 * 切断瞬间速度从 ~10cm/s 降到 3cm/s，主动急刹造成的轮子打滑/编码器
 * 少计随之大幅缩小；若地面摩擦使车在圈内爬不动，静止沉降照常判到达。 */
#define POS_CRAWL_SPEED_CMPS 3.0f
#define POS_CRAWL_STOP_CM 0.8f
/* ---------------- 方向滤波（ivision PATH_DIRECTION_FILTER_ALPHA） ---------------- */
#define DIR_FILTER_ALPHA 0.25f

static float wrap_degrees(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static bool in_range(float value, float low, float high)
{
    return isfinite(value) && value >= low && value <= high;
}

/* ---------------- 参数兜底：越界/NaN 回退到 ivision 默认值 ---------------- */
static float cfg_or(float value, float low, float high, float fallback)
{
    return in_range(value, low, high) ? value : fallback;
}

static bool config_valid(const position_config_t *c, const position_goal_t *g)
{
    return in_range(c->xy_kp, 0.01f, 20.0f) &&
           in_range(c->xy_kd, 0.0f, 5.0f) &&
           in_range(c->yaw_kp, 0.01f, 20.0f) &&
           in_range(c->max_speed_cmps, 1.0f, 500.0f) &&
           in_range(c->max_omega_radps, 0.05f, 3.0f) &&
           in_range(c->max_accel_cmps2, 1.0f, 300.0f) &&
           in_range(c->max_alpha_radps2, 0.05f, 10.0f) &&
           in_range(c->xy_tolerance_cm, 0.5f, 20.0f) &&
           in_range(c->yaw_tolerance_deg, 0.5f, 20.0f) &&
           in_range(c->brake_limit, 0.05f, 2.0f) &&
           in_range(c->brake_ceiling_cmps2, 50.0f, 5000.0f) &&
           in_range(c->short_segment_cm, 1.0f, 200.0f) &&
           in_range(c->short_boost_gain, 1.0f, 3.0f) &&
           in_range(c->approach_zone_cm, 0.0f, 50.0f) &&
           in_range(c->approach_ratio, 0.0f, 1.0f) &&
           in_range(c->approach_acc_k, 0.05f, 0.95f) &&
           in_range(c->yaw_lin_band_rad, 0.02f, 1.0f) &&
           in_range(c->yaw_kd, 0.0f, 2.0f) &&
           in_range(c->yaw_kd_translate, 0.0f, 2.0f) &&
           in_range(g->x_cm, -10000.0f, 10000.0f) &&
           in_range(g->y_cm, -10000.0f, 10000.0f) &&
           in_range(g->yaw_deg, -180.0f, 180.0f);
}

void position_control_reset(position_control_t *control)
{
    memset(control, 0, sizeof(*control));
}

static void reject(position_control_t *control, int32_t status)
{
    position_control_reset(control);
    control->output.status = status;
}

/* 平移一维速度规划（ivision velocity_planning_1d 的定点简化版）。
 * 输入剩余距离，输出本拍标量速度参考；状态存放在 control->plan_v_cmps。 */
static float plan_translation(position_control_t *control,
                              const position_config_t *config,
                              float distance_cm, float dt)
{
    /* 斜向降速（菜单 DiagK，1=关闭）：滚子滑差随速度上升，按段启动时锁定
     * 的混合度成比例同时缩放巡航速度与加速度；刹车能力 = max_acc×BrakeLim
     * 随之同比例变小，sqrt 制动规划与实际减速能力保持一致。 */
    float diag_k = cfg_or(config->diag_speed_k, 0.2f, 1.0f, 1.0f);
    float max_speed, max_acc;
    float brake_limit = cfg_or(config->brake_limit, 0.05f, 2.0f, BRAKE_LIMIT);
    float brake_ceiling = cfg_or(config->brake_ceiling_cmps2, 50.0f, 5000.0f,
                                 BRAKE_ACC_CEILING_CMPS2);
    float short_seg = cfg_or(config->short_segment_cm, 1.0f, 200.0f, SHORT_SEGMENT_CM);
    float short_gain = cfg_or(config->short_boost_gain, 1.0f, 3.0f, SHORT_BOOST_GAIN);
    float zone_cfg = cfg_or(config->approach_zone_cm, 0.0f, 50.0f, APPROACH_ZONE_CM);
    float zone_ratio = cfg_or(config->approach_ratio, 0.0f, 1.0f, APPROACH_RATIO);
    float approach_k = cfg_or(config->approach_acc_k, 0.05f, 0.95f, APPROACH_ACC_K);
    float brake_acc, accel_ramp;
    float zone, approach_acc, target_v;

    diag_k = 1.0f - (1.0f - diag_k) * control->diag_mix;
    max_speed = config->max_speed_cmps * diag_k;
    max_acc = config->max_accel_cmps2 * diag_k;
    brake_acc = max_acc * brake_limit;
    accel_ramp = max_acc;

    if (brake_ceiling > 1.0f && brake_ceiling < brake_acc)
    {
        brake_acc = brake_ceiling;
    }
    if (control->have_initial &&
        control->initial_distance_cm >= 1.0f &&
        control->initial_distance_cm <= short_seg)
    {
        accel_ramp = max_acc * short_gain; /* 只提加速斜坡，不动刹车 */
    }

    /* 蠕行切断半径：不超过容差的一半，小容差时自动缩小 */
    float stop_radius = fminf(POS_CRAWL_STOP_CM, 0.5f * config->xy_tolerance_cm);

    /* 到切断半径：命令零速，剩余距离靠极低速的惯性/摩擦收尾 */
    if (distance_cm <= stop_radius)
    {
        control->plan_v_cmps = 0.0f;
        return 0.0f;
    }

    /* 接近区双段 sqrt：近端缓曲线、外段正常刹车、边界 C0 连续。 */
    zone = zone_cfg;
    if (control->have_initial)
    {
        float zone_cap = control->initial_distance_cm * zone_ratio;
        if (zone_cap < zone) zone = zone_cap;
    }
    approach_acc = brake_acc * approach_k;
    if (!(approach_acc >= 1.0f && approach_acc < brake_acc))
    {
        zone = 0.0f; /* 参数无效退回单段 sqrt */
    }

    if (zone >= 0.5f && distance_cm <= zone)
    {
        target_v = sqrtf(2.0f * approach_acc * distance_cm);
    }
    else if (zone >= 0.5f)
    {
        target_v = sqrtf(2.0f * approach_acc * zone +
                         2.0f * brake_acc * (distance_cm - zone));
    }
    else
    {
        target_v = sqrtf(2.0f * brake_acc * distance_cm);
    }
    if (target_v > max_speed) target_v = max_speed;

    /* 容差圈内限速蠕行（上限钳制）：一进容差圈就把速度压到 CRAWL 慢慢
     * 爬向切断半径。目的：切断瞬间动量最小化——即使随后的刹车让轮子
     * 打滑，编码器少计也被限制在约 1cm 以内，而不是 3~4cm。
     * 注意这是"上限"而非下限：曲线本身更低（更接近目标）时照用曲线。 */
    if (distance_cm <= config->xy_tolerance_cm && target_v > POS_CRAWL_SPEED_CMPS)
    {
        target_v = POS_CRAWL_SPEED_CMPS;
    }

    /* 每拍斜坡：加速用 accel_ramp，减速一律用物理刹车能力。 */
    if (target_v > control->plan_v_cmps)
    {
        control->plan_v_cmps += accel_ramp * dt;
        if (control->plan_v_cmps > target_v) control->plan_v_cmps = target_v;
    }
    else
    {
        control->plan_v_cmps -= brake_acc * dt;
        if (control->plan_v_cmps < target_v) control->plan_v_cmps = target_v;
    }
    if (control->plan_v_cmps < 0.0f) control->plan_v_cmps = 0.0f;
    return control->plan_v_cmps;
}

/* 航向角速度规划（ivision YawProfiled::calculate 全移植）。
 * 实测角速度由位姿航向差分得到（导航快照不含独立陀螺输出）。 */
static float heading_hold_tolerance(const position_config_t *config)
{
    return fminf(POSITION_HEADING_HOLD_MAX_DEG, 0.5f * config->yaw_tolerance_deg);
}

static float heading_resume_tolerance(const position_config_t *config)
{
    return fminf(config->yaw_tolerance_deg, 2.0f * heading_hold_tolerance(config));
}

static float plan_yaw(position_control_t *control,
                      const position_config_t *config,
                      float yaw_error_deg, float yaw_rate_radps,
                      bool translating, float dt)
{
    float err_rad = yaw_error_deg * DEG_TO_RAD;
    float abs_err = fabsf(err_rad);
    float max_ang_vel, max_ang_acc, lin_band, v_edge, target_abs_vw;
    float stiction_vw, stiction_brake_err, target_vw, kd, max_dv;

    /* 容差死区：不再追角度（防持续微调抖动），但保留陀螺阻尼抵抗真实
     * 转动——横向起步死区冲量、斜向滑差拧转等扰动落在容差内时以前是
     * 零抵抗，现在会被顶回去。双地板量化保证静止时输出严格为零，
     * 沉降到达判定与 YAW_STALLED 故障检测都不受影响。 */
    if (abs_err <= heading_hold_tolerance(config) * DEG_TO_RAD)
        control->yaw_correcting = false;
    else if (abs_err >= heading_resume_tolerance(config) * DEG_TO_RAD)
        control->yaw_correcting = true;
    if (!control->yaw_correcting)
    {
        float damp = 0.0f, max_dv;
        if (fabsf(yaw_rate_radps) >= YAW_DEADBAND_DAMP_MIN_RATE_RADPS)
        {
            float kd = translating ? cfg_or(config->yaw_kd_translate, 0.0f, 2.0f, YAW_KD_TRANSLATE)
                                   : cfg_or(config->yaw_kd, 0.0f, 2.0f, YAW_KD);
            damp = -kd * yaw_rate_radps;
            if (fabsf(damp) < YAW_DEADBAND_DAMP_MIN_RADPS) damp = 0.0f;
        }
        max_dv = fmaxf(config->max_alpha_radps2, 0.001f) * dt;
        if (damp > control->plan_vw_radps + max_dv) control->plan_vw_radps += max_dv;
        else if (damp < control->plan_vw_radps - max_dv) control->plan_vw_radps -= max_dv;
        else control->plan_vw_radps = damp;
        return control->plan_vw_radps;
    }

    max_ang_vel = fmaxf(config->max_omega_radps, 0.0f);
    max_ang_acc = fmaxf(config->max_alpha_radps2, 0.001f);
    lin_band = fmaxf(cfg_or(config->yaw_lin_band_rad, 0.02f, 1.0f, YAW_LIN_BAND_RAD),
                     0.005f);

    /* 远端时间最优 sqrt 减速曲线；近端与带边相切的线性律（等效增益有界）。 */
    v_edge = sqrtf(2.0f * max_ang_acc * lin_band);
    if (abs_err >= lin_band)
    {
        target_abs_vw = sqrtf(2.0f * max_ang_acc * abs_err);
    }
    else
    {
        target_abs_vw = (v_edge / lin_band) * abs_err;
    }
    if (translating)
    {
        target_abs_vw *= YAW_TRANSLATE_GAIN;
    }
    if (target_abs_vw > max_ang_vel) target_abs_vw = max_ang_vel;

    /* 静摩擦补偿地板（菜单 YawStic，默认 0.15）：误差仍大于"地板速度制动距离"
     * 时，命令角速度不低于地板值，保证传动系真的在转，避免小误差下靠
     * 蠕动式走走停停长时间收敛。 */
    stiction_vw = fminf(cfg_or(config->yaw_stiction_radps, 0.0f, 1.0f, YAW_STICTION_RADPS),
                        max_ang_vel);
    /* Fade friction floor near alignment instead of a 0 -> 0.15 rad/s kick. */
    {
        float inner = heading_hold_tolerance(config) * DEG_TO_RAD;
        float fade = fminf(1.0f, fmaxf(0.0f, (abs_err-inner)/(3.0f*DEG_TO_RAD)));
        stiction_vw *= fade;
    }
    if (stiction_vw > 0.0f)
    {
        if (translating) stiction_vw *= 0.4f;
        stiction_brake_err = (stiction_vw * stiction_vw) / (2.0f * max_ang_acc);
        if (target_abs_vw < stiction_vw && abs_err > stiction_brake_err)
        {
            target_abs_vw = stiction_vw;
        }
    }

    target_vw = copysignf(target_abs_vw, err_rad);

    /* 陀螺阻尼：直接对角速度做微分反馈，抑制冲过/回摆。 */
    kd = translating ? cfg_or(config->yaw_kd_translate, 0.0f, 2.0f, YAW_KD_TRANSLATE)
                     : cfg_or(config->yaw_kd, 0.0f, 2.0f, YAW_KD);
    target_vw -= kd * yaw_rate_radps;

    /* 角加速度物理限幅。 */
    max_dv = max_ang_acc * dt;
    if (target_vw > control->plan_vw_radps + max_dv)
    {
        control->plan_vw_radps += max_dv;
    }
    else if (target_vw < control->plan_vw_radps - max_dv)
    {
        control->plan_vw_radps -= max_dv;
    }
    else
    {
        control->plan_vw_radps = target_vw;
    }
    return control->plan_vw_radps;
}

void position_control_update(position_control_t *control,
                             const position_config_t *config,
                             const position_goal_t *goal,
                             const navigation_snapshot_t *pose, float dt)
{
    float ex, ey, yaw_error, yaw_rate = 0.0f;
    float theta, cs, sn, distance, v_ref, omega;
    float want_wx = 0.0f, want_wy = 0.0f, filtered_speed;
    bool xy_inside, yaw_inside, heading_held, resting, goal_changed, translating;
    if (!config_valid(config, goal) || !in_range(dt, 0.001f, 0.05f)) {
        reject(control, POSITION_BAD_CONFIG);
        return;
    }
    if (!pose->valid || !pose->bias_ready || pose->status != NAV_RUNNING ||
        !in_range(pose->x_m, -1000.0f, 1000.0f) ||
        !in_range(pose->y_m, -1000.0f, 1000.0f) || !isfinite(pose->yaw_deg) ||
        !in_range(pose->vx_mps, -12.0f, 12.0f) ||
        !in_range(pose->vy_mps, -12.0f, 12.0f)) {
        reject(control, POSITION_NO_POSE);
        return;
    }

    ex = goal->x_cm - pose->x_m * 100.0f;
    ey = goal->y_cm - pose->y_m * 100.0f;
    /* Wrap before subtraction to avoid overflowing very large finite yaw. */
    theta = wrap_degrees(pose->yaw_deg);
    yaw_error = wrap_degrees(goal->yaw_deg - theta);
    distance = hypotf(ex, ey);
    goal_changed = !control->have_previous ||
                   goal->x_cm != control->previous_goal.x_cm ||
                   goal->y_cm != control->previous_goal.y_cm ||
                   goal->yaw_deg != control->previous_goal.yaw_deg;
    if (control->have_previous)
        yaw_rate = wrap_degrees(theta - control->previous_yaw_deg) * DEG_TO_RAD / dt;
    if (control->output.status == POSITION_YAW_STALLED && !goal_changed) return;
    if (goal_changed) {
        /* 换目标即重新规划（ivision：target 变化 reset 规划器）。 */
        control->settled_seconds = 0.0f;
        control->stall_seconds = 0.0f;
        control->yaw_stall_seconds = 0.0f;
        control->plan_v_cmps = 0.0f;
        control->plan_vw_radps = 0.0f;
        control->yaw_rate_filtered = 0.0f;
        control->yaw_correcting = true;
        control->world_vx_cmps = 0.0f;
        control->world_vy_cmps = 0.0f;
        control->initial_distance_cm = distance;
        control->have_initial = true;
        /* 斜向混合度同样在换目标瞬间锁定：min/max 分量比，0=纯轴，1=45°。
         * 行驶中不更新——接近目标时残差方向抖动会误触发降速。 */
        {
            float adx = fabsf(ex), ady = fabsf(ey);
            control->diag_mix = (distance >= 2.0f && fmaxf(adx, ady) > 1e-3f)
                                    ? fminf(adx, ady) / fmaxf(adx, ady)
                                    : 0.0f;
        }
    }
    if (!goal_changed)
        control->yaw_rate_filtered += dt/(POSITION_YAW_RATE_TAU_S+dt) *
                                      (yaw_rate-control->yaw_rate_filtered);
    yaw_rate = control->yaw_rate_filtered;
    control->previous_goal = *goal;
    control->previous_yaw_deg = theta;
    control->have_previous = true;

    control->output.distance_cm = distance;
    control->output.yaw_error_deg = yaw_error;
    xy_inside = distance <= config->xy_tolerance_cm;
    yaw_inside = fabsf(yaw_error) <= config->yaw_tolerance_deg;
    heading_held = false; /* updated after the hysteretic planner below */
    resting = hypotf(pose->vx_mps, pose->vy_mps) * 100.0f <= POSITION_REST_SPEED_CMPS &&
              fabsf(yaw_rate) <= POSITION_REST_OMEGA_RADPS;

    /* 平移：剩余距离 → 标量速度参考 → 世界系期望速度（朝目标单位向量）。 */
    v_ref = plan_translation(control, config, distance, dt);
    if (distance > 1e-3f && v_ref > 0.0f) {
        want_wx = v_ref * ex / distance;
        want_wy = v_ref * ey / distance;
    }

    /* 方向一阶滤波 + 速度钳制（ivision filter_direction）。 */
    control->world_vx_cmps += (want_wx - control->world_vx_cmps) * DIR_FILTER_ALPHA;
    control->world_vy_cmps += (want_wy - control->world_vy_cmps) * DIR_FILTER_ALPHA;
    filtered_speed = hypotf(control->world_vx_cmps, control->world_vy_cmps);
    if (filtered_speed > v_ref && filtered_speed > 0.001f) {
        float scale = v_ref / filtered_speed;
        control->world_vx_cmps *= scale;
        control->world_vy_cmps *= scale;
    }

    /* 航向：明显平移时按平移工况规划（阻尼/增益切换）。 */
    translating = v_ref > 2.0f;
    omega = plan_yaw(control, config, yaw_error, yaw_rate, translating, dt);
    heading_held = !control->yaw_correcting &&
                   fabsf(yaw_error) <= heading_resume_tolerance(config);

    /* 世界系 → 车体系输出，交给 Drive/轮速内环跟踪。 */
    theta *= DEG_TO_RAD;
    cs = cosf(theta); sn = sinf(theta);
    control->output.vx_cmps = cs * control->world_vx_cmps + sn * control->world_vy_cmps;
    control->output.vy_cmps = -sn * control->world_vx_cmps + cs * control->world_vy_cmps;
    control->output.omega_radps = omega;

    control->output.status = xy_inside && yaw_inside && heading_held ? POSITION_SETTLING : POSITION_MOVING;
    /* Arrival requires heading hold as well as XY/yaw arrival tolerances.
     * Even translational crawl stall must not bypass final heading alignment.
     * Translational crawl may settle inside both tolerances. Failure to turn
     * is a fault after 2s, never a successful waypoint/route advance. */
    {
        float measured_cmps = hypotf(pose->vx_mps, pose->vy_mps) * 100.0f;
        bool cmd_zero = control->plan_v_cmps < 0.001f &&
                        fabsf(control->plan_vw_radps) < 0.001f;
        bool hard_stopped = measured_cmps < 1.0f &&
                            fabsf(yaw_rate) <= POSITION_REST_OMEGA_RADPS;
        bool xy_stalled = hard_stopped &&
                          control->plan_v_cmps >= 0.5f * POS_CRAWL_SPEED_CMPS;
        /* 航向"冻结"判据比平移更严：实测角速度 <0.02rad/s（≈1°/s）才算
         * 执行不出来；还在缓慢收敛（>1°/s）时不触发，让它继续转。 */
        bool yaw_stalled = measured_cmps < 1.0f &&
                           fabsf(yaw_rate) < 0.02f &&
                           fabsf(control->plan_vw_radps) >= 0.05f;
        bool step = false;
        if (xy_inside && yaw_inside && heading_held) {
            if (resting && cmd_zero) {
                control->stall_seconds = 0.0f;
                control->settled_seconds += dt;
                step = true;
            } else if (xy_stalled) {
                control->settled_seconds = 0.0f;
                control->stall_seconds += dt;
                step = true;
            }
        }
        if (!step) {
            control->settled_seconds = 0.0f;
            control->stall_seconds = 0.0f;
        }
        if (xy_inside && !heading_held && yaw_stalled) control->yaw_stall_seconds += dt;
        else control->yaw_stall_seconds = 0.0f;
        if (control->yaw_stall_seconds >= 2.0f) {
            control->output.status = POSITION_YAW_STALLED;
            control->output.vx_cmps = control->output.vy_cmps = control->output.omega_radps = 0.0f;
            control->plan_v_cmps = control->plan_vw_radps = 0.0f;
            control->world_vx_cmps = control->world_vy_cmps = 0.0f;
            return;
        }
        if (xy_inside && yaw_inside && heading_held && (control->settled_seconds >= POSITION_SETTLE_TIME_S - 0.00001f ||
            control->stall_seconds >= 0.30f)) {
            control->output.status = POSITION_REACHED;
            control->output.vx_cmps = control->output.vy_cmps = control->output.omega_radps = 0.0f;
        }
    }
}
