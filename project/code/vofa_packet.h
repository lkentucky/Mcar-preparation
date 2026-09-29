/*********************************************************************************************************************
* 文件名称          vofa_packet
* 功能说明          VOFA+ JustFloat 协议打包接口与通道定义，实现见 vofa_packet.c。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          上位机按 JustFloat 解析，10 通道依次对应下方 CH0~CH9。
********************************************************************************************************************/

#ifndef VOFA_PACKET_H
#define VOFA_PACKET_H
#include <stdint.h>

#define VOFA_CHANNELS 10u                              // 上传通道数
#define VOFA_FRAME_BYTES (4u*(VOFA_CHANNELS+1u))       // 帧长：10 通道 × 4 字节 + 4 字节帧尾 = 44
#define IMU_UDP_ANGLES_BYTES 12u                        // UDP 帧：roll/pitch/yaw 各 4 字节，无额外帧头帧尾

/* CH0..2=roll/pitch/yaw 度，CH3=状态，CH4=校准进度%，CH5=|a|g，CH6=dt毫秒 */
/* CH7..9=Cube专用欧拉角X/Y/Z，绑定这三个通道到Cube。 */
void vofa_pack(uint8_t out[VOFA_FRAME_BYTES], const float channels[VOFA_CHANNELS]);   // STEP 8：打包一帧
void imu_udp_pack_angles(uint8_t out[IMU_UDP_ANGLES_BYTES], const float angles[3]);  // WiFi UDP 三轴欧拉角
#endif
