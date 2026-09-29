/*********************************************************************************************************************
* 文件名称          mahony6
* 功能说明          六轴 Mahony 姿态融合（流水线 STEP 6 的一种实现）接口声明，实现见 mahony6.c。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          输入单位：gyro rad/s、accel g、dt s。本算法与 Madgwick 二选一，由 AHRS_METHOD 在编译期决定。
********************************************************************************************************************/

#ifndef MAHONY6_H
#define MAHONY6_H

typedef struct {
    float q[4];           /* w,x,y,z；将传感器坐标旋转到参考坐标 */
    float integral[3];    /* 叉积误差的积分反馈，单位 rad/s */
    float accel_norm;     /* 加速度模长，单位 g，便于观察运动干扰 */
    int accel_used;       /* 当前加速度是否参与反馈 */
} Mahony6;

/* 用静止时的加速度初始化倾角，初始航向任意规定为 0。
 * 参考姿态：ax=ay=0, az=+1g 时 q=(1,0,0,0)。 */
int mahony6_init(Mahony6 *s, const float accel_g[3]);
/* gyro_rad_s 是去除上电零偏后的角速度，dt 必须用秒。
 * 返回 1=成功，0=参数/四元数无效；无效输入不会污染原姿态。 */
int mahony6_update(Mahony6 *s, const float gyro_rad_s[3],
                   const float accel_g[3], float dt);
/* ZYX 欧拉角：R=Rz(yaw)*Ry(pitch)*Rx(roll)，输出单位度。 */
void mahony6_euler(const Mahony6 *s, float rpy_deg[3]);
/* VOFA+ Qt3D Cube 专用：R=Ry(Y)*Rx(X)*Rz(Z)，单位度。
 * 多轴旋转时，这三个角不等于常规 ZYX roll/pitch/yaw。 */
void mahony6_cube_euler(const Mahony6 *s, float xyz_deg[3]);

#endif
