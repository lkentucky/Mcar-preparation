#ifndef MCAR_NAVIGATION_CONFIG_H
#define MCAR_NAVIGATION_CONFIG_H

/* 借鉴 HDU 的预测/修正结构，参数需在本车标定，不能沿用参考车常数。 */
#define NAV_ACCEL_BIAS_SAMPLES        100u  /* 200Hz，约 0.5s 静止采样 */
#define NAV_STATIC_WHEEL_MPS          0.003f
#define NAV_STATIC_GYRO_DPS           1.5f
#define NAV_STATIC_ACCEL_MPS2         0.25f
#define NAV_STATIC_CONFIRM_TICKS      20u  /* 100Hz，连续静止 0.2s */
#define NAV_STATIC_BIAS_ALPHA         0.005f
#define NAV_ENCODER_WEIGHT            1.0f  /* 正常时按真实编码器位移，避免惯性漂移 */
#define NAV_SLIP_ENCODER_WEIGHT       0.15f
#define NAV_SLIP_VELOCITY_MPS         0.50f
#define NAV_SLIP_ACCEL_MPS2           3.0f
#define NAV_SLIP_HOLD_TICKS           8u
#define NAV_MAX_WHEEL_MPS             12.0f /* 异常脉冲门限，不是速度指令限幅 */
#define NAV_IMU_STALE_S               0.050f
#define NAV_IMU_MAX_DT_S              0.020f
#define NAV_MAX_YAW_RADPS             40.0f

/* Body-axis odometry calibration: new = old * measured_distance / displayed_distance.
 * Leave at 1 until measured; Y multiplies the legacy Motor.h lateral factor.
 * These correct localization only, not encoder counts or wheel-speed PID units. */
#define NAV_FORWARD_SCALE_DEFAULT      0.97f
#define NAV_LEFT_SCALE_DEFAULT         1.04f

/* 斜向滚子滑差二次修正（菜单 DiagCmp）：麦轮斜向运动时滚子轴向滑动
 * 编码器不可见，实测横移增量被系统性多计。斜向时在 lateral_scale 之上
 * 再对 dy 乘本系数（<1 = 缩小横移计数），1 = 关闭。混合度按车体系运动
 * 方向 min(|dx|,|dy|)/max(|dx|,|dy|) 计（0=纯轴，1=45°），补偿随混合度
 * 线性混入，纯轴运动完全不受影响。只改定位，不改编码器计数与轮速 PID。 */
#define NAV_DIAG_LATERAL_SCALE_DEFAULT 1.0f
#define NAV_DIAG_MIX_ALPHA             0.1f /* 混合度一阶滤波 @100Hz，时间常数约 0.1s */

#endif
