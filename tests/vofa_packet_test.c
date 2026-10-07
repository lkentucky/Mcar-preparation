/* 主机端协议测试：实际 WiFi 驱动由 Keil 构建验证，此处仅替代传输层。 */
#include "wifispi.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t sent_frame[VOFA_MAX_FRAME_BYTES];
static size_t sent_count;
static size_t sent_bytes;

/* GNU host linker wraps only the transport. Packing remains the real code. */
int __wrap_wifispi_send_floats(const float *channels, size_t count)
{
    sent_count = count;
    sent_bytes = vofa_pack(sent_frame, sizeof(sent_frame), channels, count);
    return sent_bytes != 0u;
}

static void check_tail(const uint8_t *frame, size_t count)
{
    static const uint8_t tail[] = {0x00u, 0x00u, 0x80u, 0x7fu};
    assert(memcmp(frame + 4u * count, tail, sizeof(tail)) == 0);
}

int main(void)
{
    const float angles[] = {1.0f, -2.5f, 0.0f};
    const uint8_t expected[] = {
        0x00u, 0x00u, 0x80u, 0x3fu,
        0x00u, 0x00u, 0x20u, 0xc0u,
        0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x80u, 0x7fu
    };
    uint8_t frame[VOFA_MAX_FRAME_BYTES + 1u];
    uint8_t unchanged[sizeof(frame)];
    float maximum[VOFA_MAX_CHANNELS];
    int once = 3;
    size_t i;

    memset(frame, 0xa5, sizeof(frame));
    assert(vofa_pack(frame, sizeof(frame), angles, 3u) == sizeof(expected));
    assert(memcmp(frame, expected, sizeof(expected)) == 0);
    assert(frame[sizeof(expected)] == 0xa5u);

    /* 非法参数和不足的容量必须返回 0，且不能改写输出。 */
    memset(frame, 0xa5, sizeof(frame));
    memcpy(unchanged, frame, sizeof(frame));
    assert(vofa_pack(NULL, sizeof(frame), angles, 3u) == 0u);
    assert(vofa_pack(frame, sizeof(frame), NULL, 3u) == 0u);
    assert(vofa_pack(frame, sizeof(frame), angles, 0u) == 0u);
    assert(vofa_pack(frame, sizeof(frame), angles, VOFA_MAX_CHANNELS + 1u) == 0u);
    assert(vofa_pack(frame, sizeof(frame), angles, SIZE_MAX) == 0u);
    assert(vofa_pack(frame, sizeof(expected) - 1u, angles, 3u) == 0u);
    assert(memcmp(frame, unchanged, sizeof(frame)) == 0);

    for (i = 0u; i < VOFA_MAX_CHANNELS; ++i)
    {
        maximum[i] = (float)i;
    }
    assert(vofa_pack(frame, VOFA_MAX_FRAME_BYTES, maximum, VOFA_MAX_CHANNELS)
           == VOFA_MAX_FRAME_BYTES);
    check_tail(frame, VOFA_MAX_CHANNELS);
    assert(frame[VOFA_MAX_FRAME_BYTES] == 0xa5u);

    /* 自动计数，混合整数/浮点参数，每个表达式只能执行一次。 */
    assert(wifi_justfloat(1.0f, -2.5f, 0));
    assert(sent_count == 3u && sent_bytes == sizeof(expected));
    assert(memcmp(sent_frame, expected, sizeof(expected)) == 0);
    assert(wifi_justfloat(once++));
    assert(once == 4 && sent_count == 1u && sent_bytes == 8u);
    check_tail(sent_frame, sent_count);

    assert(wifi_justfloat(1, 2, 3, 4, 5, 6, 7));
    assert(sent_count == 7u && sent_bytes == 32u);
    check_tail(sent_frame, sent_count);

    puts("PASS: little-endian bytes, frame sizes/tails, 40-channel limit, invalid inputs, macro count/evaluation");
    return 0;
}
