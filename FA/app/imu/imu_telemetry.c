/* 首条验证链的 UART7 二进制协议与单帧发送缓冲。 */
#include "imu_telemetry.h"

#include <float.h>
#include <string.h>

#include "mc02_board.h"

enum {
    IMU_TELEMETRY_CHANNEL_COUNT = 21,
    IMU_TELEMETRY_HEADER_SIZE = 4,
    IMU_TELEMETRY_FOOTER_SIZE = 4,
};

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "Serial Plotter requires IEEE 754 binary32");
_Static_assert(IMU_TELEMETRY_FRAME_SIZE == IMU_TELEMETRY_HEADER_SIZE +
                                               4 * IMU_TELEMETRY_CHANNEL_COUNT +
                                               IMU_TELEMETRY_FOOTER_SIZE,
               "Serial Plotter frame size mismatch");

void imu_telemetry_init(imu_telemetry_t *telemetry)
{
    telemetry->frame_offset = sizeof(telemetry->frame);
    telemetry->dropped_frames = 0U;
}

static void encode_float32_le(uint8_t *out, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (size_t i = 0; i < sizeof(bits); ++i) {
        out[i] = (uint8_t)(bits >> (8U * i));
    }
}

void imu_telemetry_submit(imu_telemetry_t *telemetry, const imu_telemetry_snapshot_t *snapshot,
                          uint32_t skipped_slots)
{
    telemetry->dropped_frames += skipped_slots;
    if (telemetry->frame_offset != sizeof(telemetry->frame)) {
        ++telemetry->dropped_frames;
        return;
    }

    const double *q = snapshot->attitude.q_odom_from_body;
    const double *a = snapshot->body_observation.accel_body_m_s2;
    const double *g = snapshot->body_observation.gyro_body_rad_s;
    const double channels[IMU_TELEMETRY_CHANNEL_COUNT] = {
        q[0],
        q[1],
        q[2],
        q[3],
        a[0],
        a[1],
        a[2],
        g[0],
        g[1],
        g[2],
        snapshot->temperature_deg_c,
        snapshot->sample_dt_s,
        snapshot->body_observation.is_valid ? 1.0 : 0.0,
        snapshot->attitude.status,
        snapshot->sample_status,
        snapshot->sample_sequence,
        (double)snapshot->sample_time_us * 1e-6,
        snapshot->missed_samples,
        telemetry->dropped_frames,
        snapshot->accel_init_status,
        snapshot->gyro_init_status,
    };
    const uint8_t header[IMU_TELEMETRY_HEADER_SIZE] = {0xA5, 0x5A, 0x49, 0x4D};
    const uint8_t footer[IMU_TELEMETRY_FOOTER_SIZE] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(telemetry->frame, header, sizeof(header));
    for (size_t i = 0; i < IMU_TELEMETRY_CHANNEL_COUNT; ++i) {
        encode_float32_le(&telemetry->frame[sizeof(header) + 4U * i], (float)channels[i]);
    }
    memcpy(&telemetry->frame[sizeof(telemetry->frame) - sizeof(footer)], footer, sizeof(footer));
    telemetry->frame_offset = 0U;
}

void imu_telemetry_poll(imu_telemetry_t *telemetry)
{
    const size_t remaining = sizeof(telemetry->frame) - telemetry->frame_offset;
    telemetry->frame_offset +=
        mc02_board_uart_write(telemetry->frame + telemetry->frame_offset, remaining);
}
