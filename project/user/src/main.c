#include "zf_common_headfile.h"

#include "Mymenu.h"
#include "app_control.h"
#include "app_wifi_telemetry.h"
#include "imu_attitude.h"

#define PIT_SHARED_IRQ_PRIORITY 2u

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);

    app_control_init();
    imu_attitude_init();
    Menu_Init();
    app_wifi_telemetry_init();

    /* CH0: 200 Hz IMU; CH1: 100 Hz wheel loop; CH2: 50 Hz key scan. */
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 10);
    pit_ms_init(PIT_CH2, 20);
    interrupt_set_priority(PIT_IRQn, PIT_SHARED_IRQ_PRIORITY);
    interrupt_global_enable(0);

    while (1)
    {
        /* Blocking reinitialization and UDP transmission stay outside IRQs. */
        imu_attitude_service();
        app_wifi_telemetry_service();
        Menu_Switch();
        Menu_Show();
    }
}



