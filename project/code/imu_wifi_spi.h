#ifndef IMU_WIFI_SPI_H
#define IMU_WIFI_SPI_H

#include <stdint.h>

/* Keil Watch 状态：0=尚未开始，1=UDP 已就绪；2/3/4=正在读取模块/连接 WiFi/建立 UDP；
 * -1=模块版本读取失败，-2=WiFi 连接失败，-3=UDP socket 建立失败，-4=发送失败。 */
extern volatile int imu_wifi_status;
extern volatile uint32_t imu_wifi_tx_packets;
/* 本次上电已尝试次数；最近一次错误即使后续重试成功也会保留，0 表示未发生错误。 */
extern volatile uint32_t imu_wifi_init_attempts;
extern volatile int imu_wifi_last_error;

int imu_wifi_init(void);
int imu_wifi_send_angles(const float angles[3]);

#endif
