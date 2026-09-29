#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"

#include "app_control.h"
#include "imu_attitude.h"

void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0))
    {
        pit_flag_clear(PIT_CH0);
        imu_attitude_update_5ms();
    }

    if (pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
        app_control_motor_tick_10ms();
    }

    if (pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
        key_scanner();
    }

    if (pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }
    __DSB();
}

/* debug_init() enables UART1 RX interrupts in the stock SeekFree library. */
void LPUART1_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}
