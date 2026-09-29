/*********************************************************************************************************************
* 文件名称          attitude_math
* 功能说明          两种融合方法共用的姿态几何工具（STEP 5 初始化 / STEP 7 欧拉角）接口声明，实现见 attitude_math.c。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          加速度单位 g，欧拉角输出单位度。
********************************************************************************************************************/

#ifndef ATTITUDE_MATH_H
#define ATTITUDE_MATH_H

/* STEP 5：由静止重力向量求初始姿态。w,x,y,z，传感器坐标到参考坐标；静止+Z加速度为+1g，初始yaw=0。
 * 返回 1=成功，0=输入非法。 */
int attitude_init(float q[4], float *accel_norm, const float accel_g[3]);

/* STEP 7：ZYX roll/pitch/yaw，单位度。 */
void attitude_euler(const float q[4], float rpy_deg[3]);

/* STEP 7：Qt Cube 专用，R=Ry(Y)*Rx(X)*Rz(Z)，单位度。 */
void attitude_cube_euler(const float q[4], float xyz_deg[3]);
#endif
