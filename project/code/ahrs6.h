/*********************************************************************************************************************
* 文件名称          ahrs6
* 功能说明          融合算法编译期分发层。按 config.h 的 AHRS_METHOD 把统一的 ahrs6_* 接口映射到 Mahony 或
*                   Madgwick 的具体实现，并定义统一的姿态类型 Ahrs6。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          采样、标定与 USB 输出只依赖本文件暴露的 ahrs6_init/update/euler/cube_euler，
*                   不需要知道当前用的是哪种融合算法。运行时不可切换。
********************************************************************************************************************/

#ifndef AHRS6_H
#define AHRS6_H
#include "config.h"

/* 编译时开关：采样、校准和USB输出不需要知道具体融合算法。 */
#if AHRS_METHOD == AHRS_METHOD_MAHONY
#include "mahony6.h"
typedef Mahony6 Ahrs6;
#define ahrs6_init       mahony6_init
#define ahrs6_update     mahony6_update
#define ahrs6_euler      mahony6_euler
#define ahrs6_cube_euler mahony6_cube_euler
#elif AHRS_METHOD == AHRS_METHOD_MADGWICK
#include "madgwick6.h"
typedef Madgwick6 Ahrs6;
#define ahrs6_init       madgwick6_init
#define ahrs6_update     madgwick6_update
#define ahrs6_euler      madgwick6_euler
#define ahrs6_cube_euler madgwick6_cube_euler
#endif
#endif
