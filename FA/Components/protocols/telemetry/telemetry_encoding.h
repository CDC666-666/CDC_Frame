#ifndef FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_TELEMETRY_ENCODING_H
#define FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_TELEMETRY_ENCODING_H

#include <float.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/** @brief 向至少 4 字节的非空缓冲区写入 IEEE 754 binary32 小端位模式。 */
static inline void telemetry_encoding_write_float32_le(uint8_t *out, float value)
{
    _Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
                   "Float encoding requires IEEE 754 binary32");
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (size_t i = 0U; i < sizeof(bits); ++i) {
        out[i] = (uint8_t)(bits >> (8U * i));
    }
}

#endif /* FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_TELEMETRY_ENCODING_H */
