/* UART7 Plotter 帧格式，纯字节编码。 */
#include "telemetry_encoding.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

enum {
    TELEMETRY_HEADER_SIZE = 4,
    TELEMETRY_FOOTER_SIZE = 4,
};

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "Float encoding requires IEEE 754 binary32");
_Static_assert(IMU_TELEMETRY_FRAME_SIZE == TELEMETRY_HEADER_SIZE +
                                               sizeof(float) * IMU_TELEMETRY_CHANNEL_COUNT +
                                               TELEMETRY_FOOTER_SIZE,
               "IMU telemetry frame size");
_Static_assert(MOTOR_TELEMETRY_FRAME_SIZE == TELEMETRY_HEADER_SIZE +
                                                 sizeof(float) * MOTOR_TELEMETRY_CHANNEL_COUNT +
                                                 TELEMETRY_FOOTER_SIZE,
               "motor telemetry frame size");

/** 作用按固定帧头、float32 小端通道和帧尾完成纯字节编码。
 * 输入非空 frame（至少 8+4×count 字节）、4 字节 header、channels 和 count，输出 frame 字节，无返回值。 */
static void encode_frame(uint8_t *frame, const uint8_t header[TELEMETRY_HEADER_SIZE],
                         const float *channels, size_t count)
{
    const uint8_t footer[TELEMETRY_FOOTER_SIZE] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(frame, header, TELEMETRY_HEADER_SIZE);
    uint8_t *payload = frame + TELEMETRY_HEADER_SIZE;
    for (size_t i = 0U; i < count; ++i) {
        uint32_t bits;
        memcpy(&bits, &channels[i], sizeof(bits));
        for (size_t j = 0U; j < sizeof(bits); ++j) {
            payload[i * sizeof(bits) + j] = (uint8_t)(bits >> (8U * j));
        }
    }
    memcpy(payload + count * sizeof(float), footer, sizeof(footer));
}

/** 作用将按 IM 顺序排列的 21 路通道编码成 IMU 遥测帧。
 * 输入非空 92 字节 frame 和 fields，输出包含帧头、通道和帧尾的 frame，无返回值。 */
void telemetry_encode_imu(uint8_t frame[IMU_TELEMETRY_FRAME_SIZE],
                          const imu_telemetry_fields_t *fields)
{
    const uint8_t header[TELEMETRY_HEADER_SIZE] = {0xA5, 0x5A, 0x49, 0x4D};
    encode_frame(frame, header, fields->channels, IMU_TELEMETRY_CHANNEL_COUNT);
}

/** 作用将按 DM 顺序排列的 16 路通道编码成电机遥测帧。
 * 输入非空 72 字节 frame 和 fields，输出包含帧头、通道和帧尾的 frame，无返回值。 */
void telemetry_encode_motor(uint8_t frame[MOTOR_TELEMETRY_FRAME_SIZE],
                            const motor_telemetry_fields_t *fields)
{
    const uint8_t header[TELEMETRY_HEADER_SIZE] = {0xA5, 0x5A, 0x44, 0x4D};
    encode_frame(frame, header, fields->channels, MOTOR_TELEMETRY_CHANNEL_COUNT);
}
