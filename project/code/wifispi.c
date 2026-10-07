/** 网络驱动、JustFloat 打包及周期遥测合并。
 * 保留原 WiFi 禁用行为和 SPI1 引脚冲突编译检查。
 * 发送为同步操作，只在主循环且开中断时调用。
 */
#include "wifispi.h"
#include "imu.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_device_wifi_spi.h"
#include "zf_driver_delay.h"
#include "fsl_common.h"
#include <string.h>

volatile int imu_wifi_status;
volatile uint32_t imu_wifi_tx_packets, imu_wifi_init_attempts;
volatile int imu_wifi_last_error;

/* 编译期确认 float32；协议不依赖主机字节序。 */
typedef char float_must_be_32_bits[(sizeof(float)==4)?1:-1];
static void pack_float32_le(uint8_t *out,float value)
{
    uint32_t bits;
    unsigned j;
    memcpy(&bits,&value,4);
    for (j=0;j<4;++j) out[j]=(uint8_t)(bits>>(8*j));
}
size_t vofa_pack(uint8_t *out,size_t out_capacity,const float *channels,size_t count)
{
    size_t i,frame_bytes;
    if (out==NULL || channels==NULL || count==0u || count>VOFA_MAX_CHANNELS) return 0u;
    frame_bytes=VOFA_FRAME_BYTES(count);
    if (out_capacity<frame_bytes) return 0u;
    for (i=0u;i<count;++i) pack_float32_le(&out[4u*i],channels[i]);
    out[4u*count]=0x00u;
    out[4u*count+1u]=0x00u;
    out[4u*count+2u]=0x80u;
    out[4u*count+3u]=0x7fu;
    return frame_bytes;
}
#if IMU_WIFI_ENABLED
#include "zf_driver_spi.h"
/* 原 SPI1 固定引脚 D12/D14/D15 与前轮冲突；必须先改非冲突接线和驱动。 */
typedef char wifi_spi1_conflicts_with_motor_D12_D15[(WIFI_SPI_INDEX==SPI_1)?-1:1];
static int wifi_ready;
static uint32 g_last_wifi_tx_ticks;
static int imu_wifi_init_once(void)
{
    memset(wifi_spi_version,0,sizeof(wifi_spi_version));
    memset(wifi_spi_ip_addr_port,0,sizeof(wifi_spi_ip_addr_port));
    imu_wifi_status=2;
    if (wifi_spi_init(0,0)!=0 || strncmp(wifi_spi_version,"V2",2)!=0) return -1;
    imu_wifi_status=3;
    if (wifi_spi_wifi_connect(IMU_WIFI_SSID,IMU_WIFI_PASSWORD)!=0) return -2;
    imu_wifi_status=4;
    if (wifi_spi_socket_connect("UDP",IMU_WIFI_TARGET_IP,
                               IMU_WIFI_TARGET_PORT,IMU_WIFI_LOCAL_PORT)!=0) return -3;
    return 1;
}
#endif
int wifispi_init(void)
{
#if IMU_WIFI_ENABLED
    uint32_t attempt;
    int result;
    wifi_ready=0;
    imu_wifi_status=0; imu_wifi_init_attempts=0; imu_wifi_last_error=0;
    /* 保留原初始化统计语义：启用模式重新初始化不清发送累计数。 */
    system_delay_ms(IMU_WIFI_STARTUP_DELAY_MS);
    for (attempt=0;attempt<IMU_WIFI_INIT_ATTEMPTS;++attempt) {
        if (attempt!=0) system_delay_ms(IMU_WIFI_RETRY_DELAY_MS);
        ++imu_wifi_init_attempts;
        result=imu_wifi_init_once();
        if (result==1) { wifi_ready=1; imu_wifi_status=1; return 1; }
        imu_wifi_last_error=result; imu_wifi_status=result;
    }
    return 0;
#else
    /* 禁用时绝不初始化 SPI/GPIO，防止覆盖电机引脚复用。 */
    imu_wifi_status=-5; imu_wifi_init_attempts=0;
    imu_wifi_last_error=0; imu_wifi_tx_packets=0;
    return 0;
#endif
}
int wifispi_send_floats(const float *channels,size_t count)
{
#if IMU_WIFI_ENABLED
    uint8_t packet[VOFA_MAX_FRAME_BYTES];
    size_t packet_bytes;
    if (!wifi_ready) return 0;
    packet_bytes=vofa_pack(packet,sizeof(packet),channels,count);
    if (packet_bytes==0u) return 0;
    /* send_buffer 返回未发送字节数；send_now 结束本 UDP 数据报。 */
    if (wifi_spi_send_buffer(packet,(uint32_t)packet_bytes)!=0 ||
        wifi_spi_udp_send_now()!=0) { imu_wifi_status=-4; return 0; }
    imu_wifi_status=1; ++imu_wifi_tx_packets;
    return 1;
#else
    (void)channels; (void)count;
    return 0;
#endif
}
int wifispi_send_imu(const float angles_deg[3])
{
    return wifispi_send_floats(angles_deg,3u);
}
void wifispi_telemetry_init(void)
{
    (void)wifispi_init();
#if IMU_WIFI_ENABLED
    g_last_wifi_tx_ticks=DWT->CYCCNT;
#endif
}
void wifispi_telemetry_service(void)
{
#if IMU_WIFI_ENABLED
    uint32 now=DWT->CYCCNT;
    uint32 period_ticks=(system_clock/1000u)*IMU_WIFI_PERIOD_MS;
    uint32 primask;
    if ((uint32)(now-g_last_wifi_tx_ticks)<period_ticks) return;
    g_last_wifi_tx_ticks=now;
    /* 仅在短临界区复制变量，恢复中断之后才打包、发送。
     * 以后增加电机等数据，只需在这里包含相应头文件并扩展列表。 */
    primask=interrupt_global_disable();
    const float channels[]={
        imu_roll_deg,  /* CH0，度 */
        imu_pitch_deg, /* CH1，度 */
        imu_yaw_deg    /* CH2，度 */
    };
    typedef char channels_fit_frame[(sizeof(channels)/sizeof(channels[0])<=VOFA_MAX_CHANNELS)?1:-1];
    (void)sizeof(channels_fit_frame);
    interrupt_global_enable(primask);
    (void)wifispi_send_floats(channels,sizeof(channels)/sizeof(channels[0]));
#endif
}
