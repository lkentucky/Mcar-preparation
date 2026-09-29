/*********************************************************************************************************************
* 文件名称          madgwick6
* 功能说明          六轴 Madgwick 姿态融合（流水线 STEP 6 的默认实现）接口声明，实现见 madgwick6.c。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          输入单位：gyro rad/s、accel g、dt s。本算法与 Mahony 二选一，由 AHRS_METHOD 在编译期决定。
********************************************************************************************************************/

#ifndef MADGWICK6_H
#define MADGWICK6_H

typedef struct {
    float q[4];       /* w,x,y,z；传感器坐标旋转到参考坐标 */
    float accel_norm; /* 加速度模长，g；与Mahony共用I5通道 */
    int accel_used;   /* 加速度是否通过门限，可作为重力参考 */
} Madgwick6;

int madgwick6_init(Madgwick6 *s, const float accel_g[3]);           /* STEP 5：由静止重力向量确定初始姿态 */
/* 输入已经扣除静止零偏的角速度rad/s、加速度g、实测时间间隔秒。
 * 成功返回1；非法输入返回0，原状态保持不变。 */
int madgwick6_update(Madgwick6 *s, const float gyro_rad_s[3],
                     const float accel_g[3], float dt);             /* STEP 6：单步融合 */
void madgwick6_euler(const Madgwick6 *s, float rpy_deg[3]);         /* STEP 7：ZYX 欧拉角，单位度 */
void madgwick6_cube_euler(const Madgwick6 *s, float xyz_deg[3]);    /* STEP 7：Qt3D Cube 专用角，单位度 */
#endif
