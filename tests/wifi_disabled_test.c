#include "config.h"
#include "wifispi.h"
#include <assert.h>
#include <stdio.h>

/* Link without WiFi/SPI/GPIO/delay or packet implementations: disabled paths
 * must not reference hardware even when init/send are called explicitly. */
int main(void)
{
    const float channels[] = {1, 2, 3};
    assert(IMU_WIFI_ENABLED == 0);
    assert(wifispi_init() == 0);
    assert(imu_wifi_status == -5);
    assert(imu_wifi_init_attempts == 0 && imu_wifi_tx_packets == 0);
    assert(wifispi_send_floats(channels, 3) == 0);
    assert(wifispi_send_floats(NULL, 0) == 0);
    assert(imu_wifi_status == -5 && imu_wifi_tx_packets == 0);
    puts("WiFi disabled tests passed: no init/send hardware dependencies");
    return 0;
}
