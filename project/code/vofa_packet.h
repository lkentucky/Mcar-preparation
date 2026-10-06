/*********************************************************************************************************************
* 文件名称          vofa_packet
* 功能说明          通用 VOFA+ JustFloat 打包接口，支持可变数量的 float32 通道。
* 开发环境          MDK / armclang
* 适用平台          RT1064 Lite 核心板
* 备注信息          通道顺序由调用处决定，不自动添加时间戳或变量名称。
********************************************************************************************************************/

#ifndef VOFA_PACKET_H
#define VOFA_PACKET_H
#include <stddef.h>
#include <stdint.h>

#define VOFA_MAX_CHANNELS       40u                    // 最多上传 40 个用户通道
#define VOFA_TAIL_BYTES         4u                     // 帧尾 00 00 80 7F
#define VOFA_FRAME_BYTES(count) (4u * (count) + VOFA_TAIL_BYTES) // N 通道的总帧长
#define VOFA_MAX_FRAME_BYTES    VOFA_FRAME_BYTES(VOFA_MAX_CHANNELS) // 最大缓冲区长度

/* 按小端序打包 count 个 float32 通道并追加 JustFloat 帧尾。
 * out_capacity 为输出缓冲区字节数；返回实际帧长，参数非法或空间不足返回 0，且不写入 out。 */
size_t vofa_pack(uint8_t *out, size_t out_capacity,
                 const float *channels, size_t count);
#endif
