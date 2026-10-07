#include "zf_common_headfile.h"
#include "Mymenu.h"
#include "app_control.h"
#include "wifispi.h"
#include "imu.h"

#define PIT_SHARED_IRQ_PRIORITY 2u

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);

    app_control_init();
    imu_init();
    Menu_Init();
    wifispi_telemetry_init();

    /* CH0: 200 Hz IMU；CH1: 100 Hz 电机控制；CH2: 50 Hz 按键扫描。 */
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 10);
    pit_ms_init(PIT_CH2, 20);
    interrupt_set_priority(PIT_IRQn, PIT_SHARED_IRQ_PRIORITY);
    interrupt_global_enable(0);

    while (1)
    {
        /* 阻塞重初始化和 UDP 同步发送只在主循环执行。 */
        imu_service();
        wifispi_telemetry_service();
        Menu_Switch();
        Menu_Show();
    }
}
