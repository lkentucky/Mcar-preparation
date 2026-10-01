/*********************************************************************************************************************
* 文件名称          vofa_packet
* 功能说明          VOFA+ JustFloat 协议打包，把调用处指定的 float 通道按小端序拼成字节帧。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          帧格式：N 通道 × 4 字节 + 4 字节帧尾。帧尾固定 0x7f800000，
*                   是 JustFloat 的帧结束标志（float 的 +Inf），上位机据此切分数据。
********************************************************************************************************************/

#include "vofa_packet.h"
#include <string.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将通道数据打包为 JustFloat 帧
// 参数说明     out             输出缓冲区
// 参数说明     out_capacity    输出缓冲区长度，单位字节
// 参数说明     channels        输入通道数据，顺序即上位机 CH0、CH1……
// 参数说明     count           输入通道数，范围 1~VOFA_MAX_CHANNELS
// 返回参数     size_t          实际帧长；参数非法或空间不足返回 0
// 使用示例     vofa_pack(frame, sizeof(frame), channels, sizeof(channels) / sizeof(channels[0]));
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

size_t vofa_pack(uint8_t *out, size_t out_capacity,
                 const float *channels, size_t count)
{
    size_t i;
    size_t frame_bytes;

    if (out == NULL || channels == NULL || count == 0u || count > VOFA_MAX_CHANNELS)
    {
        return 0u;
    }
    frame_bytes = VOFA_FRAME_BYTES(count);
    if (out_capacity < frame_bytes)
    {
        return 0u;
    }

    for (i = 0u; i < count; ++i)
    {
        pack_float32_le(&out[4u * i], channels[i]);
    }
    out[4u * count] = 0x00u;
    out[4u * count + 1u] = 0x00u;
    out[4u * count + 2u] = 0x80u;
    out[4u * count + 3u] = 0x7fu;
    return frame_bytes;
}
