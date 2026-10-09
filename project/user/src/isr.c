/*********************************************************************************************************************
* RT1064DVL6A Opensource Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 RT1064DVL6A 开源库的一部分
*
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅 https://www.gnu.org/licenses/
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称 isr
* 公司名称 成都逐飞科技有限公司
* 版本信息 查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境 IAR 8.32.4 or MDK 5.33
* 适用平台 RT1064DVL6A
* 店铺链接 https://seekfree.taobao.com/
*
* 修改记录
* 日期 作者 备注
* 2022-09-21 SeekFree first version
* 2026-10-07 IMU 合并：头文件改为 imu.h；CH0 调用 imu_update_5ms。
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "app_control.h"
#include "Mymenu.h"
#include "imu.h"
#include "app_navigation.h"

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();
}
void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH0)) {
        pit_flag_clear(PIT_CH0);
        imu_update_5ms();
        app_navigation_imu_tick_5ms();
    }
    if(pit_flag_get(PIT_CH1)) {
        pit_flag_clear(PIT_CH1);
        app_control_motor_tick_10ms();
    }
    if(pit_flag_get(PIT_CH2)) {
        pit_flag_clear(PIT_CH2);
        key_scanner();
        Menu_Tick_20ms();
    }
    if(pit_flag_get(PIT_CH3)) pit_flag_clear(PIT_CH3);
    __DSB();
}
void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1)) {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}
void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2)) {
        /* 原预留接收中断。 */
    }
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);
}
void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3)) {
        /* 原预留接收中断。 */
    }
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);
}
void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4)) {
        flexio_camera_uart_handler();
        gnss_uart_callback();
    }
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);
}
void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5)) camera_uart_handler();
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);
}
void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6)) {
        /* 原预留接收中断。 */
    }
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);
}
void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8)) wireless_module_uart_handler();
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);
}
void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0)) exti_flag_clear(B0);
}
void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16)) exti_flag_clear(B16);
}
void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    if(exti_flag_get(C0)) exti_flag_clear(C0);
}
void GPIO2_Combined_16_31_IRQHandler(void)
{
    tof_module_exti_handler();
    if(exti_flag_get(C16)) exti_flag_clear(C16);
}
void GPIO3_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(IMU660RC_INT2_PIN)) {
        imu660rc_callback();
        exti_flag_clear(IMU660RC_INT2_PIN);
    }
    if(exti_flag_get(D4)) exti_flag_clear(D4);
}
