#ifndef FRAME_CDC_APP_TELEMETRY_IMU_TELEMETRY_H
#define FRAME_CDC_APP_TELEMETRY_IMU_TELEMETRY_H

#include <stddef.h>

#include "imu_observer.h"
#include "ins.h"
#include "telemetry_encoding.h"

typedef struct {
    uint8_t frame[IMU_TELEMETRY_FRAME_SIZE];
    size_t frame_offset;
    uint64_t next_frame_us;
    uint32_t dropped_frames;
} imu_telemetry_t;

/** @brief 初始化实例。first_frame_us 与采样时钟同源，单位为微秒。 */
void imu_telemetry_init(imu_telemetry_t *telemetry, uint64_t first_frame_us);

/** @brief 按 20 ms 时槽选取快照，非阻塞推进 UART7。仅主循环调用，不留存输入指针。 */
void imu_telemetry_update(imu_telemetry_t *telemetry, const imu_observer_output_t *imu,
                          const attitude_sample_t *pose, uint64_t now_us);

#endif /* FRAME_CDC_APP_TELEMETRY_IMU_TELEMETRY_H */
