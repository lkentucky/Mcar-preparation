#ifndef IMU_WIFI_SPI_H
#define IMU_WIFI_SPI_H

#include <stddef.h>
#include <stdint.h>

/* Keil Watch 状态：0=尚未开始，1=UDP 已就绪；2/3/4=正在读取模块/连接 WiFi/建立 UDP；
 * -1=模块版本读取失败，-2=WiFi 连接失败，-3=UDP socket 建立失败，-4=发送失败，-5=功能禁用。 */
extern volatile int imu_wifi_status;
extern volatile uint32_t imu_wifi_tx_packets;
/* 本次上电已尝试次数；最近一次错误即使后续重试成功也会保留，0 表示未发生错误。 */
extern volatile uint32_t imu_wifi_init_attempts;
extern volatile int imu_wifi_last_error;

int imu_wifi_init(void);
/* 通过 UDP 发送 count 个 JustFloat 通道，顺序与 channels 一致。
 * 返回 1=发送成功，0=未就绪、参数非法或发送失败；在主循环调用。 */
int imu_wifi_send_floats(const float *channels, size_t count);

/* 自动计算通道数；整数自动转为 float，每个表达式只求值一次。
 * 示例：wifi_justfloat(imu_roll_deg, imu_pitch_deg, imu_yaw_deg, up_L_all);
 * 至少传入一个数值；发送仍是同步的，不应在中断或关中断区内调用。 */
#define wifi_justfloat(...) \
    imu_wifi_send_floats((const float[]){__VA_ARGS__}, \
        sizeof((const float[]){__VA_ARGS__}) / sizeof(float))

#endif
