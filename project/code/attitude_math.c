/*********************************************************************************************************************
* 文件名称          attitude_math
* 功能说明          两种融合方法（Mahony / Madgwick）共用的姿态几何工具：只负责初始姿态求解（STEP 5）与
*                   四元数到欧拉角的转换（STEP 7），不含任何滤波逻辑。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          坐标系约定：传感器坐标 → 参考坐标；静止时 +Z 加速度为 +1g，初始 yaw 任意规定为 0。
*                   角速度单位 rad/s，加速度单位 g，欧拉角输出单位度。
********************************************************************************************************************/

#include "attitude_math.h"
#include "config.h"
#include "imu_numeric.h"
#include <math.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     限幅
// 参数说明     x               输入值
// 参数说明     lo              下限
// 参数说明     hi              上限
// 返回参数     float           钳位到 [lo, hi] 后的值
// 使用示例     clamp(1.2f, -1.0f, 1.0f);                                   // 结果为 1.0f
// 备注信息     内部调用。用于 asinf 之前把参数夹到定义域，避免浮点误差导致 NaN。
//-------------------------------------------------------------------------------------------------------------------
static float clamp(float x,float lo,float hi) { return x<lo ? lo : (x>hi ? hi : x); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     由静止重力向量求初始姿态（STEP 5）
// 参数说明     q               输出四元数 w,x,y,z
// 参数说明     accel_norm      输出加速度模长，单位 g
// 参数说明     a               输入静止重力向量，单位 g
// 返回参数     int             1-成功 0-加速度非法或模长超出门限
// 使用示例     attitude_init(next.q, &next.accel_norm, a);
// 备注信息     由重力方向解出 roll/pitch 并转成四元数，yaw 固定为 0（六轴无磁力计，航向不可观测）。
//             参考姿态：ax=ay=0, az=+1g 时 q=(1,0,0,0)。被 mahony6_init / madgwick6_init 调用。
//-------------------------------------------------------------------------------------------------------------------
int attitude_init(float q[4], float *accel_norm, const float a[3])
{
    float norm, roll, pitch, cr, sr, cp, sp;
    /* 输入校验：分量必须有限，且模长落在 ACC_NORM_MIN~MAX 内才认为静止可靠。 */
    if (!imu_finite(a[0]) || !imu_finite(a[1]) || !imu_finite(a[2])) return 0;
    norm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (!imu_finite(norm) || norm < ACC_NORM_MIN || norm > ACC_NORM_MAX) return 0;
    /* 由重力分量直接求倾角：roll 绕 X，pitch 绕 Y。 */
    roll = atan2f(a[1], a[2]);
    pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
    /* 半角形式组装四元数，等价于先 roll 后 pitch 的两轴旋转。 */
    cr = cosf(roll*0.5f); sr = sinf(roll*0.5f);
    cp = cosf(pitch*0.5f); sp = sinf(pitch*0.5f);
    q[0] = cr*cp; q[1] = sr*cp;
    q[2] = cr*sp; q[3] = -sr*sp;
    *accel_norm = norm;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转 ZYX 欧拉角（STEP 7）
// 参数说明     q               输入四元数 w,x,y,z
// 参数说明     e               输出 roll/pitch/yaw，单位度
// 返回参数     void
// 使用示例     attitude_euler(attitude.q, channels);
// 备注信息     约定 R=Rz(yaw)*Ry(pitch)*Rx(roll)。pitch 由 asinf 求得，须先限幅再换算；
//             在 pitch=±90° 的万向锁处 roll/yaw 不唯一，但姿态表示本身仍然有效。
//-------------------------------------------------------------------------------------------------------------------
void attitude_euler(const float q[4], float e[3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    e[0]=atan2f(2.0f*(w*x+y*z), 1.0f-2.0f*(x*x+y*y))*RAD_TO_DEG;
    e[1]=asinf(clamp(2.0f*(w*y-z*x), -1.0f, 1.0f))*RAD_TO_DEG;
    e[2]=atan2f(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z))*RAD_TO_DEG;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转 Qt3D Cube 专用欧拉角（STEP 7）
// 参数说明     q               输入四元数 w,x,y,z
// 参数说明     e               输出 X/Y/Z，单位度
// 返回参数     void
// 使用示例     attitude_cube_euler(attitude.q, &channels[7]);
// 备注信息     先由四元数展开旋转矩阵，再按 Cube 需要的顺序分解，避免复合转动时 3D 模型显示错误。
//             分解顺序 R=Ry(Y)*Rx(X)*Rz(Z)，与 attitude_euler() 的 ZYX 不同，不可混用。
//-------------------------------------------------------------------------------------------------------------------
void attitude_cube_euler(const float q[4], float e[3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    float r00=1.0f-2.0f*(y*y+z*z), r02=2.0f*(x*z+w*y);
    float r10=2.0f*(x*y+w*z), r11=1.0f-2.0f*(x*x+z*z);
    float r12=2.0f*(y*z-w*x), r20=2.0f*(x*z-w*y);
    float r22=1.0f-2.0f*(x*x+y*y);
    float cx=sqrtf(r10*r10+r11*r11);
    /* Cube源码将数据赋给 Qt3D Transform.rotationX/Y/Z，底层对应
     * QQuaternion::fromEulerAngles(X,Y,Z)，即先绕Z，再X，再Y。
     * 从同一个姿态矩阵分解为这个顺序，避免复合转动时显示错误。 */
    e[0]=atan2f(-r12,cx)*RAD_TO_DEG;
    if (cx>1e-5f) {
        e[1]=atan2f(r02,r22)*RAD_TO_DEG;
        e[2]=atan2f(r10,r11)*RAD_TO_DEG;
    } else {
        /* X接近±90度时Y/Z不唯一，取Z=0，保留等效姿态。 */
        e[1]=atan2f(-r20,r00)*RAD_TO_DEG;
        e[2]=0;
    }
}
