/*********************************************************************************************************************
* 文件名称          calibration
* 功能说明          上电静止标定（流水线 STEP 3）。持续采样静止状态下的陀螺与加速度，样本均值即为陀螺零偏
*                   与初始重力向量：零偏供 STEP 4 零漂消除使用，重力向量供 STEP 5 姿态初始化使用。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          输入单位：gyro deg/s、accel g。积分窗口为 CALIBRATION_SAMPLES 个样本（200Hz 下约 2 秒）。
********************************************************************************************************************/

#include "calibration.h"
#include "config.h"
#include <math.h>
#include "imu_numeric.h"
#include <string.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空标定累加器
// 参数说明     c               标定结构体
// 返回参数     void
// 使用示例     calibration_reset(&calibration);                           // 上电或按 R 重新标定前调用
// 备注信息     清零 count 与各累加/极值，使下一次 calibration_push() 重新从第 0 个样本开始。
//-------------------------------------------------------------------------------------------------------------------
void calibration_reset(ImuCalibration *c) { memset(c, 0, sizeof(*c)); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     静止标定送样（STEP 3）
// 参数说明     c               标定结构体
// 参数说明     g               本帧陀螺，单位 deg/s
// 参数说明     a               本帧加速度，单位 g
// 返回参数     int             1-样本已收满，零偏与初始重力有效 0-仍在采集中或被判定为运动
// 使用示例     if (calibration_push(&calibration, sample.gyro_dps, sample.accel_g)) { /* 标定完成 */ }
// 备注信息     每帧先做静止判定，任一条件不满足即整体重新计数（丢弃已累积样本）：
//             ① 加速度模长须在 0.9~1.1g；② 各轴陀螺须有限且不超过 CAL_GYRO_LIMIT_DPS；
//             ③ 各轴峰峰值不得超过 CAL_GYRO_SPAN_DPS / CAL_ACCEL_SPAN_G。
//             收满 CALIBRATION_SAMPLES 个样本后取均值，得到 bias_dps（零偏）与 initial_accel（初始重力）。
//             本质是静止检查，无法识别所有匀速运动。
//-------------------------------------------------------------------------------------------------------------------
int calibration_push(ImuCalibration *c, const float g[3], const float a[3])
{
    unsigned i;
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    /* 静止判定①：加速度模长偏离 1g 说明在加/减速或自由落体，本次标定作废。 */
    if (!imu_finite(n) || n < 0.9f || n > 1.1f) {
        calibration_reset(c); return 0;
    }
    for (i=0; i<3; ++i) {
        /* 静止判定②：陀螺数值非法或超出静止阈值，同样作废重来。 */
        if (!imu_finite(g[i]) || fabsf(g[i]) > CAL_GYRO_LIMIT_DPS) {
            calibration_reset(c); return 0;
        }
        /* 记录各轴极值，用于下面的峰峰值判据。 */
        if (!c->count) {
            c->min_g[i]=c->max_g[i]=g[i]; c->min_a[i]=c->max_a[i]=a[i];
        }
        if (g[i]<c->min_g[i]) c->min_g[i]=g[i];
        if (g[i]>c->max_g[i]) c->max_g[i]=g[i];
        if (a[i]<c->min_a[i]) c->min_a[i]=a[i];
        if (a[i]>c->max_a[i]) c->max_a[i]=a[i];
        /* 静止判定③：峰峰值超限说明窗口内出现过运动，整个窗口重新计数。 */
        if (c->max_g[i]-c->min_g[i]>CAL_GYRO_SPAN_DPS ||
            c->max_a[i]-c->min_a[i]>CAL_ACCEL_SPAN_G) {
            calibration_reset(c); return 0;
        }
    }
    for (i=0; i<3; ++i) { c->sum_g[i]+=g[i]; c->sum_a[i]+=a[i]; }
    if (++c->count < CALIBRATION_SAMPLES) return 0;
    /* 样本收满：取均值。零偏用于 STEP 4 扣零漂，initial_accel 用于 STEP 5 求初始姿态。 */
    for (i=0; i<3; ++i) {
        c->bias_dps[i]=c->sum_g[i]/c->count;
        c->initial_accel[i]=c->sum_a[i]/c->count;
    }
    return 1;
}
