/*********************************************************************************************************************
* 文件名称          vofa_packet
* 功能说明          VOFA+ JustFloat 协议打包（流水线 STEP 8 的一部分）。把 10 个 float 通道按小端序拼成定长字节帧，
*                   供 imu_usb_send_frame() 经 USB CDC 发出。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          帧格式：10 通道 × 4 字节 + 4 字节帧尾，共 44 字节。帧尾固定 0x7f800000，
*                   是 JustFloat 的帧结束标志（float 的 +Inf），上位机据此切分数据。
********************************************************************************************************************/

#include "vofa_packet.h"
#include <string.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将通道数据打包为 JustFloat 帧
// 参数说明     out             输出缓冲区，长度须不小于 VOFA_FRAME_BYTES
// 参数说明     channels        输入通道数据，含义见 vofa_packet.h
// 返回参数     void
// 使用示例     vofa_pack(s_txFrame, channels);
// 备注信息     按位拷贝后显式写小端，避免结构体填充和指针别名问题。
//             末尾写入固定帧尾 0x7f800000，上位机以此识别一帧的结束。
//-------------------------------------------------------------------------------------------------------------------
typedef char float_must_be_32_bits[(sizeof(float)==4) ? 1 : -1]; /* 编译期校验 float 为 32 位 */
static void pack_float32_le(uint8_t *out, float value)
{
    uint32_t bits;
    unsigned j;
    memcpy(&bits, &value, 4);
    for (j=0; j<4; ++j) out[j]=(uint8_t)(bits>>(8*j));
}

void vofa_pack(uint8_t out[VOFA_FRAME_BYTES], const float channels[VOFA_CHANNELS])
{
    unsigned i;
    for (i=0; i<VOFA_CHANNELS; ++i) pack_float32_le(&out[4*i], channels[i]);
    out[4*VOFA_CHANNELS]=0; out[4*VOFA_CHANNELS+1]=0;
    out[4*VOFA_CHANNELS+2]=0x80; out[4*VOFA_CHANNELS+3]=0x7f;    /* JustFloat 帧尾 0x7f800000 */
}

/* UDP 自带报文边界，因此不附加 JustFloat 帧尾；上位机偏移 0/4/8 直接读取。 */
void imu_udp_pack_angles(uint8_t out[IMU_UDP_ANGLES_BYTES], const float angles[3])
{
    unsigned i;
    for (i=0; i<3; ++i) pack_float32_le(&out[4*i], angles[i]);
}
