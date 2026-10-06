/*********************************************************************************************************************
* 文件名称          imu_wifi_spi
* 功能说明          逐飞 WiFi-SPI2.0 UDP 传输适配。发送可变数量的小端 float32 通道及 JustFloat 帧尾。
* 备注信息          SPI1 接 WiFi 模块，IMU 使用 SPI4，互不占用同一 SPI 控制器。
********************************************************************************************************************/

#include "imu_wifi_spi.h"
#include "config.h"
#include "vofa_packet.h"
#include "zf_device_wifi_spi.h"
#include "zf_driver_delay.h"
#include <string.h>

volatile int imu_wifi_status;
volatile uint32_t imu_wifi_tx_packets;
volatile uint32_t imu_wifi_init_attempts;
volatile int imu_wifi_last_error;
#if IMU_WIFI_ENABLED
#include "zf_driver_spi.h"
/* SPI1 can only use D12/D14/D15 in this library; those are motor pins. */
typedef char wifi_spi1_conflicts_with_motor_D12_D15[(WIFI_SPI_INDEX == SPI_1) ? -1 : 1];
static int wifi_ready;

/* 分开执行逐飞驱动的三个阶段，避免一个 -1 混淆模块、WiFi 和 UDP 故障。
 * 驱动初始化传入空 SSID，只完成 SPI/模块握手；随后显式连接热点。 */
static int imu_wifi_init_once(void)
{
    /* 清除上一次尝试留下的字符串，Watch 不会把旧 IP 当作本次成功的证据。 */
    memset(wifi_spi_version, 0, sizeof(wifi_spi_version));
    memset(wifi_spi_ip_addr_port, 0, sizeof(wifi_spi_ip_addr_port));
    imu_wifi_status = 2;
    /* 驱动遇到非预期版本回复时也可能返回 0，再核对 V2 前缀。 */
    if (wifi_spi_init(0, 0) != 0 || strncmp(wifi_spi_version, "V2", 2) != 0) return -1;

    imu_wifi_status = 3;
    if (wifi_spi_wifi_connect(IMU_WIFI_SSID, IMU_WIFI_PASSWORD) != 0) return -2;

    imu_wifi_status = 4;
    if (wifi_spi_socket_connect("UDP", IMU_WIFI_TARGET_IP,
                                IMU_WIFI_TARGET_PORT, IMU_WIFI_LOCAL_PORT) != 0) return -3;
    return 1;
}
#endif

int imu_wifi_init(void)
{
#if IMU_WIFI_ENABLED
    uint32_t attempt;
    int result;
    wifi_ready = 0;
    imu_wifi_status = 0;
    imu_wifi_init_attempts = 0;
    imu_wifi_last_error = 0;
    /* 逐飞连接函数会等待模块响应，只在 board_init() 中调用，不能放进采样热路径。 */
    system_delay_ms(IMU_WIFI_STARTUP_DELAY_MS);
    for (attempt = 0; attempt < IMU_WIFI_INIT_ATTEMPTS; ++attempt) {
        if (attempt != 0) system_delay_ms(IMU_WIFI_RETRY_DELAY_MS);
        ++imu_wifi_init_attempts;
        result = imu_wifi_init_once();
        if (result == 1) {
            wifi_ready = 1;
            imu_wifi_status = 1;
            return 1;
        }
        imu_wifi_last_error = result;
        imu_wifi_status = result;
    }
    return 0;
#else
    /* Do not initialize SPI or GPIO: keep motor pin muxes intact. */
    imu_wifi_status = -5;
    imu_wifi_init_attempts = 0;
    imu_wifi_last_error = 0;
    imu_wifi_tx_packets = 0;
    return 0;
#endif
}

int imu_wifi_send_floats(const float *channels, size_t count)
{
#if IMU_WIFI_ENABLED
    uint8_t packet[VOFA_MAX_FRAME_BYTES];
    size_t packet_bytes;

    if (!wifi_ready) return 0;
    packet_bytes = vofa_pack(packet, sizeof(packet), channels, count);
    if (packet_bytes == 0u) return 0;
    /* send_buffer 返回“未发送的字节数”；send_now 强制结束这一个 UDP 数据报。 */
    if (wifi_spi_send_buffer(packet, (uint32_t)packet_bytes) != 0 ||
        wifi_spi_udp_send_now() != 0) {
        imu_wifi_status = -4;
        return 0;
    }
    imu_wifi_status = 1;
    ++imu_wifi_tx_packets;
    return 1;
#else
    (void)channels;
    (void)count;
    return 0;
#endif
}
