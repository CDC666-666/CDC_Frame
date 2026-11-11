#ifndef FRAME_CDC_COMPONENTS_TELEMETRY_IMU_TELEMETRY_H
#define FRAME_CDC_COMPONENTS_TELEMETRY_IMU_TELEMETRY_H

#include <stddef.h>

#include "imu_observer.h"
#include "ins.h"
#include "telemetry_encoding.h"

typedef struct {
    uint8_t frame[IMU_TELEMETRY_FRAME_SIZE];
    size_t frame_offset;
    uint64_t next_frame_us;
    uint64_t period_us;
    size_t (*write_bytes)(const uint8_t *data, size_t size);
    uint32_t dropped_frames;
} imu_telemetry_t;

/** 作用初始化 IMU 遥测帧缓冲、周期调度和非阻塞写入接口。
 * 输入非空 telemetry、同源 first_frame_us、正 period_us 和 write_bytes，输出就绪的 telemetry，无返回值。 */
void imu_telemetry_init(imu_telemetry_t *telemetry, uint64_t first_frame_us, uint64_t period_us,
                        size_t (*write_bytes)(const uint8_t *, size_t));

/** 作用按时槽选取 IMU/INS 快照并非阻塞推进输出，输入只在主循环本次调用使用。
 * 输入非空 telemetry、imu、pose 和 now_us，输出帧缓冲、发送偏移及丢帧统计，无返回值。 */
void imu_telemetry_update(imu_telemetry_t *telemetry, const imu_observer_output_t *imu,
                          const attitude_sample_t *pose, uint64_t now_us);

#endif /* FRAME_CDC_COMPONENTS_TELEMETRY_IMU_TELEMETRY_H */
