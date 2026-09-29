/*********************************************************************************************************************
* 文件名称          mahony6
* 功能说明          六轴 Mahony 姿态融合（流水线 STEP 6 的一种实现）。由陀螺积分提供姿态变化，用加速度估计的
*                   重力方向构造叉积误差，经 PI 反馈修正角速度，兼顾低频倾角精度与高频动态响应。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          无磁力计，无法观测绕重力方向的绝对航向（yaw），长期仅靠陀螺积分。
*                   输入单位：gyro rad/s、accel g、dt s；姿态用四元数 q=(w,x,y,z) 表示。
*                   是否选用本算法由 config.h 的 AHRS_METHOD 在编译期决定。
********************************************************************************************************************/

#include "mahony6.h"
#include "config.h"
#include <math.h>
#include "imu_numeric.h"
#include "attitude_math.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     限幅
// 参数说明     x               输入值
// 参数说明     lo              下限
// 参数说明     hi              上限
// 返回参数     float           钳位到 [lo, hi] 后的值
// 使用示例     clamp(0.3f, -0.1f, 0.1f);                                   // 结果为 0.1f
// 备注信息     内部调用。用于限制积分项的幅值，防止长时间偏差累积成大幅零偏。
//-------------------------------------------------------------------------------------------------------------------
static float clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mahony 姿态初始化（STEP 5）
// 参数说明     s               输出姿态结构体
// 参数说明     a               静止重力向量，单位 g
// 返回参数     int             1-成功 0-输入的加速度模长非法
// 使用示例     mahony6_init(&attitude, calibration.initial_accel);
// 备注信息     由重力向量解出初始 roll/pitch，初始 yaw 约定为 0；积分项清零。
//             初始化失败时不改动原姿态，主循环以 CH3=-4 上报。
//-------------------------------------------------------------------------------------------------------------------
int mahony6_init(Mahony6 *s, const float a[3])
{
    Mahony6 next = {0};
    if (!attitude_init(next.q, &next.accel_norm, a)) return 0;
    *s = next;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mahony 姿态融合单步更新（STEP 6）
// 参数说明     s               姿态结构体（输入旧姿态，成功时更新为新姿态）
// 参数说明     g               已去零偏的角速度，单位 rad/s
// 参数说明     a               加速度，单位 g
// 参数说明     dt              距上一帧的时间间隔，单位秒
// 返回参数     int             1-成功 0-参数或四元数非法（原姿态保持不变）
// 使用示例     mahony6_update(&attitude, gyro, sample.accel_g, dt);
// 备注信息     单帧流程：校验 → 求加速度模长与门限 → 叉积误差 → PI 反馈修正角速度 →
//             四元数精确轴角更新 → 归一化。加速度未通过门限时仅做陀螺积分，并冻结积分项。
//             姿态更新用常值角速度下微分方程的解析解，而非一阶欧拉，高速旋转时无角度损失。
//-------------------------------------------------------------------------------------------------------------------
int mahony6_update(Mahony6 *s, const float g[3], const float a[3], float dt)
{
    Mahony6 next = *s;                                   /* 先在副本上运算，全部成功后才写回，避免半更新 */
    float w=s->q[0], x=s->q[1], y=s->q[2], z=s->q[3];
    float omega[3], error[3]={0,0,0}, n;
    unsigned i;
    /* 输入校验：dt 必须落在 (0, MAX_SAMPLE_DT]，否则本帧不可信，直接放弃。 */
    if (!imu_finite(dt) || dt <= 0.0f || dt > MAX_SAMPLE_DT) return 0;
    for (i=0; i<3; ++i) if (!imu_finite(g[i]) || !imu_finite(a[i])) return 0;
    /* 求加速度模长并做门限判定：只有接近 1g 时才认为它可作为重力参考。 */
    n = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!imu_finite(n)) return 0;
    next.accel_norm = n;
    next.accel_used = (n >= ACC_NORM_MIN && n <= ACC_NORM_MAX);
    if (next.accel_used) {
        float ax=a[0]/n, ay=a[1]/n, az=a[2]/n;           /* 归一化为单位方向，只看方向不看大小 */
        /* 当前四元数预测的“静止加速度方向”，用传感器坐标表示。
         * 加速度计测的是比力，静止时与物理重力反向；这里遵循 +Z=+1g 的约定。 */
        float vx=2.0f*(x*z-w*y);
        float vy=2.0f*(w*x+y*z);
        float vz=w*w-x*x-y*y+z*z;
        /* 核心反馈误差：实测单位方向 a_hat × 预测单位方向 v_hat。
         * 它的方向表示应修正的旋转轴，模长为 sin(夹角)，不是欧拉角之差。
         * 仅有重力参考，不能观测绕重力方向的绝对航向。 */
        error[0]=ay*vz-az*vy;
        error[1]=az*vx-ax*vz;
        error[2]=ax*vy-ay*vx;
    }
    /* PI 反馈：把误差同时用作比例项（Kp，直接修正当前角速度）和积分项（Ki，补偿持续偏差）。 */
    for (i=0; i<3; ++i) {
        if (AHRS_KI > 0.0f) {
            /* 加速度被门限拒绝时冻结积分，避免将明显的线加速度累积为零偏。 */
            if (next.accel_used)
                next.integral[i]=clamp(next.integral[i]+AHRS_KI*error[i]*dt,
                                     -AHRS_INTEGRAL_LIMIT, AHRS_INTEGRAL_LIMIT);
        } else next.integral[i]=0.0f;
        /* 修正后的角速度 = 陀螺实测 + 比例修正 + 积分修正 */
        omega[i]=g[i]+AHRS_KP*error[i]+next.integral[i];
    }
    /* 四元数精确轴角更新：q_new = q ⊗ dq，dq 是绕 omega 方向转过 |omega|*dt 的旋转。
     * 这是微分方程 q_dot = 0.5*q ⊗ [0,omega] 在常值角速度下的解析解，单步无积分误差；
     * 一阶欧拉每步会少转 (|omega|*dt)^3/12（弧度），高速回转时表现为航向角损失
     * （1000deg/s、200Hz 下每秒约 0.63deg，整圈累计 0.23deg）。
     * 展开 dq = [cos(theta), omega/mag*sin(theta)]，theta 为半步转角 |omega|*dt/2。 */
    {
        float mag=sqrtf(omega[0]*omega[0]+omega[1]*omega[1]+omega[2]*omega[2]);
        float theta=0.5f*mag*dt;
        float dq0, s, dq1, dq2, dq3;
        if (!imu_finite(mag) || !imu_finite(theta)) return 0;
        dq0=cosf(theta);
        /* mag→0 时 dq 退化为单位四元数，直接取极限值 0.5*dt，避免 0/0。 */
        s=(mag > 1e-9f) ? sinf(theta)/mag : 0.5f*dt;
        dq1=omega[0]*s; dq2=omega[1]*s; dq3=omega[2]*s;
        next.q[0]=w*dq0-x*dq1-y*dq2-z*dq3;
        next.q[1]=w*dq1+x*dq0+y*dq3-z*dq2;
        next.q[2]=w*dq2-x*dq3+y*dq0+z*dq1;
        next.q[3]=w*dq3+x*dq2-y*dq1+z*dq0;
    }
    /* 归一化：抑制数值积分带来的模长漂移，保证四元数始终代表纯旋转。 */
    n=sqrtf(next.q[0]*next.q[0]+next.q[1]*next.q[1]+
            next.q[2]*next.q[2]+next.q[3]*next.q[3]);
    if (!imu_finite(n) || n < 1e-6f) return 0;
    for (i=0; i<4; ++i) next.q[i]/=n;
    *s=next;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取 ZYX 欧拉角（STEP 7）
// 参数说明     s               姿态结构体
// 参数说明     e               输出 roll/pitch/yaw，单位度
// 返回参数     void
// 使用示例     mahony6_euler(&attitude, channels);                        // 结果写入 CH0~CH2
// 备注信息     四元数 → 欧拉角，约定 R=Rz(yaw)*Ry(pitch)*Rx(roll)，实现见 attitude_euler()。
//-------------------------------------------------------------------------------------------------------------------
void mahony6_euler(const Mahony6 *s, float e[3]) { attitude_euler(s->q,e); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取 Qt3D Cube 专用欧拉角（STEP 7）
// 参数说明     s               姿态结构体
// 参数说明     e               输出 X/Y/Z，单位度
// 返回参数     void
// 使用示例     mahony6_cube_euler(&attitude, &channels[7]);               // 结果写入 CH7~CH9
// 备注信息     仅供 VOFA+ 的 Qt3D Cube 显示使用，顺序 R=Ry(Y)*Rx(X)*Rz(Z)，
//             多轴旋转时这三个角不等于常规 ZYX roll/pitch/yaw。
//-------------------------------------------------------------------------------------------------------------------
void mahony6_cube_euler(const Mahony6 *s, float e[3]) { attitude_cube_euler(s->q,e); }
