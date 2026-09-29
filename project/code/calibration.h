/*********************************************************************************************************************
* 文件名称          calibration
* 功能说明          上电静止标定（流水线 STEP 3）接口声明，实现见 calibration.c。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          输入单位：gyro deg/s、accel g。标定结果供 STEP 4 零漂消除与 STEP 5 姿态初始化使用。
********************************************************************************************************************/

#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

typedef struct {
    unsigned count;                                                   // 已累计的静止样本数
    float sum_g[3], sum_a[3], min_g[3], max_g[3], min_a[3], max_a[3]; // 累加和与各轴极值（峰峰值判据用）
    float bias_dps[3], initial_accel[3];                              // 标定结果：陀螺零偏(deg/s)、初始重力(g)
} ImuCalibration;

void calibration_reset(ImuCalibration *c);                            // 清空累加器，准备重新标定

/* STEP 3：送入一帧样本。输入均为新样本，gyro 单位 deg/s，accel 单位 g。
 * 运动时重新计数；返回 1 表示收集完成。它是静止检查，无法识别所有匀速运动。 */
int calibration_push(ImuCalibration *c, const float gyro[3], const float accel[3]);
#endif
