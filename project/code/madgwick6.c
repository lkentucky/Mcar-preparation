/*********************************************************************************************************************
* 文件名称          madgwick6
* 功能说明          六轴 Madgwick 姿态融合（流水线 STEP 6 的一种实现，默认选用）。按 Sebastian O.H. Madgwick 原始
*                   报告中的 IMU 公式实现，以陀螺积分为主体，用加速度构造梯度下降项进行归一化修正。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          目标函数 f = 预测重力方向 − 实测单位加速度，反馈方向为 J^T * f。
*                   无磁力计、无固定 512Hz 假设、无快速倒平方根的指针类型转换。
*                   输入单位：gyro rad/s、accel g、dt s；姿态用四元数 q=(w,x,y,z) 表示。
*                   报告出处：https://x-io.co.uk/downloads/madgwick_internal_report.pdf
*                   应用实现遵循本项目 GPL-3.0-or-later 许可证。
********************************************************************************************************************/

#include "madgwick6.h"
#include "imu_attitude.h"
#include "config.h"
#include "imu_numeric.h"
#include <math.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Madgwick 姿态初始化（STEP 5）
// 参数说明     s               输出姿态结构体
// 参数说明     a               静止重力向量，单位 g
// 返回参数     int             1-成功 0-输入的加速度模长非法
// 使用示例     madgwick6_init(&attitude, calibration.initial_accel);
// 备注信息     由重力向量解出初始 roll/pitch，初始 yaw 约定为 0。与 Mahony 共用初始化逻辑。
//-------------------------------------------------------------------------------------------------------------------
int madgwick6_init(Madgwick6 *s, const float a[3])
{
    Madgwick6 next = {0};
    if (!attitude_init(next.q, &next.accel_norm, a)) return 0;
    *s = next;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Madgwick 姿态融合单步更新（STEP 6）
// 参数说明     s               姿态结构体（输入旧姿态，成功时更新为新姿态）
// 参数说明     g               已去零偏的角速度，单位 rad/s
// 参数说明     a               加速度，单位 g
// 参数说明     dt              距上一帧的时间间隔，单位秒
// 返回参数     int             1-成功 0-参数或四元数非法（原姿态保持不变）
// 使用示例     madgwick6_update(&attitude, gyro, sample.accel_g, dt);
// 备注信息     单帧流程：校验 → 陀螺项 qdot → 加速度门限判定 → 梯度下降修正 qdot →
//             按实测 dt 积分 → 归一化。加速度未通过门限时只保留陀螺积分项。
//-------------------------------------------------------------------------------------------------------------------
int madgwick6_update(Madgwick6 *s, const float g[3], const float a[3], float dt)
{
    Madgwick6 next = *s;                                 /* 先在副本上运算，全部成功后才写回 */
    float w=s->q[0], x=s->q[1], y=s->q[2], z=s->q[3];
    float qdot[4], n;
    unsigned i;
    /* 输入校验：dt 与 BETA 必须合法且四元数非退化。 */
    if (!imu_finite(dt) || dt<=0.0f || dt>MAX_SAMPLE_DT ||
        !imu_finite(MADGWICK_BETA) || MADGWICK_BETA<0.0f) return 0;
    for (i=0; i<3; ++i)
        if (!imu_finite(g[i]) || !imu_finite(a[i])) return 0;
    for (i=0; i<4; ++i) if (!imu_finite(s->q[i])) return 0;
    n=w*w+x*x+y*y+z*z;
    if (!imu_finite(n) || n<1e-12f) return 0;

    /* 1. 陀螺仪给出四元数导数：qdot=0.5*q⊗[0,g]。 */
    qdot[0]=0.5f*(-x*g[0]-y*g[1]-z*g[2]);
    qdot[1]=0.5f*( w*g[0]+y*g[2]-z*g[1]);
    qdot[2]=0.5f*( w*g[1]-x*g[2]+z*g[0]);
    qdot[3]=0.5f*( w*g[2]+x*g[1]-y*g[0]);

    /* 2. 与Mahony相同的模长门限。加减速或自由落体时仅做陀螺仪积分。
     * 门限只拒绝明显异常，不能完全分离重力与线加速度。 */
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!imu_finite(n)) return 0;
    next.accel_norm=n;
    next.accel_used=(n>=ACC_NORM_MIN && n<=ACC_NORM_MAX);
    if (next.accel_used && MADGWICK_BETA>0.0f) {
        float ax=a[0]/n, ay=a[1]/n, az=a[2]/n;
        /* 3. 单位四元数预测的重力参考方向，减去实测方向得到三维残差。
         * 这里的 f 不是欧拉角误差，也不是 Mahony 使用的叉积误差。 */
        float f0=2.0f*(x*z-w*y)-ax;
        float f1=2.0f*(w*x+y*z)-ay;
        float f2=1.0f-2.0f*(x*x+y*y)-az;
        /* J=df/dq。梯度step=J^T*f指向残差平方和增大的方向，
         * 因此从qdot中减去beta*归一化step，让估计向实测重力靠近。 */
        float step[4]={
            -2.0f*y*f0+2.0f*x*f1,
             2.0f*z*f0+2.0f*w*f1-4.0f*x*f2,
            -2.0f*w*f0+2.0f*z*f1-4.0f*y*f2,
             2.0f*x*f0+2.0f*y*f1
        };
        float step_norm=sqrtf(step[0]*step[0]+step[1]*step[1]+
                              step[2]*step[2]+step[3]*step[3]);
        if (!imu_finite(step_norm)) return 0;
        /* 水平静止或重力方向完全吻合时梯度可以为0，不能做0/0。
         * 这个数值保护只用于反馈梯度，不会抹掉小角速度或伪造yaw稳定。 */
        if (step_norm>1e-6f)
            for (i=0; i<4; ++i) qdot[i]-=MADGWICK_BETA*step[i]/step_norm;
    }

    /* 4. 用同一帧的旧姿态计算全部导数，再按实测dt积分并归一化。 */
    for (i=0; i<4; ++i) next.q[i]=s->q[i]+qdot[i]*dt;
    n=sqrtf(next.q[0]*next.q[0]+next.q[1]*next.q[1]+
            next.q[2]*next.q[2]+next.q[3]*next.q[3]);
    if (!imu_finite(n) || n<1e-6f) return 0;
    for (i=0; i<4; ++i) next.q[i]/=n;
    *s=next;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取 ZYX 欧拉角（STEP 7）
// 参数说明     s               姿态结构体
// 参数说明     e               输出 roll/pitch/yaw，单位度
// 返回参数     void
// 使用示例     madgwick6_euler(&attitude, channels);                      // 结果写入 CH0~CH2
// 备注信息     四元数 → 欧拉角，约定 R=Rz(yaw)*Ry(pitch)*Rx(roll)，实现见 attitude_euler()。
//-------------------------------------------------------------------------------------------------------------------
void madgwick6_euler(const Madgwick6 *s, float e[3]) { attitude_euler(s->q,e); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取 Qt3D Cube 专用欧拉角（STEP 7）
// 参数说明     s               姿态结构体
// 参数说明     e               输出 X/Y/Z，单位度
// 返回参数     void
// 使用示例     madgwick6_cube_euler(&attitude, &channels[7]);             // 结果写入 CH7~CH9
// 备注信息     仅供 VOFA+ 的 Qt3D Cube 显示使用，顺序 R=Ry(Y)*Rx(X)*Rz(Z)，
//             多轴旋转时这三个角不等于常规 ZYX roll/pitch/yaw。
//-------------------------------------------------------------------------------------------------------------------
void madgwick6_cube_euler(const Madgwick6 *s, float e[3]) { attitude_cube_euler(s->q,e); }
