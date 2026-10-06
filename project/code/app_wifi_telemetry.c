#include "app_wifi_telemetry.h"

#include "config.h"
#include "imu_attitude.h"
#include "imu_wifi_spi.h"
#include "vofa_packet.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "fsl_common.h"

#if IMU_WIFI_ENABLED
static uint32 g_last_wifi_tx_ticks;
#endif

void app_wifi_telemetry_init(void)
{
    (void)imu_wifi_init();
#if IMU_WIFI_ENABLED
    g_last_wifi_tx_ticks = DWT->CYCCNT;
#endif
}

void app_wifi_telemetry_service(void)
{
#if IMU_WIFI_ENABLED
    uint32 now = DWT->CYCCNT;
    uint32 period_ticks = (system_clock / 1000u) * IMU_WIFI_PERIOD_MS;
    uint32 primask;

    if ((uint32)(now - g_last_wifi_tx_ticks) < period_ticks)
    {
        return;
    }
    g_last_wifi_tx_ticks = now;

    /* 修改 WiFi 传输内容只需增删下面的变量，数组顺序对应 CH0、CH1……。
     * 在短暂关中断期间取得一致快照，实际打包和发送在恢复中断后执行。 */
    primask = interrupt_global_disable();
    const float channels[] = {
        imu_roll_deg,    /* CH0: roll，度 */
        imu_pitch_deg,   /* CH1: pitch，度 */
        imu_yaw_deg      /* CH2: yaw，度 */
    };
    typedef char channels_fit_frame[
        (sizeof(channels) / sizeof(channels[0]) <= VOFA_MAX_CHANNELS) ? 1 : -1];
    (void)sizeof(channels_fit_frame);
    interrupt_global_enable(primask);
    (void)imu_wifi_send_floats(channels, sizeof(channels) / sizeof(channels[0]));
#endif
}
