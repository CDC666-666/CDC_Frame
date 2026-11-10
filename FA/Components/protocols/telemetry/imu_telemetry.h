#ifndef FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_IMU_TELEMETRY_H
#define FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_IMU_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#include "attitude.h"
#include "imu_device.h"

enum {
    IMU_TELEMETRY_FRAME_SIZE = 92,
};

typedef struct {
    uint8_t frame[IMU_TELEMETRY_FRAME_SIZE];
    size_t frame_offset;
    uint32_t dropped_frames;
} imu_telemetry_t;

/** @brief 初始化非空实例的缓冲区状态及发送侧计数。仅主循环上下文调用。 */
void imu_telemetry_init(imu_telemetry_t *telemetry);

/**
 * @brief 按当前快照编码一帧。实例与快照均须非空。
 * @param skipped_slots 本次提交前已错过的遥测时槽数。
 * @details 仅主循环调用，不等待串口，也不保留快照指针。
 *          若仍有待发字节，保留原帧并累计丢弃本次提交。
 */
void imu_telemetry_submit(imu_telemetry_t *telemetry, const imu_device_output_t *imu,
                          const attitude_t *attitude, uint32_t skipped_slots);

/** @brief 仅从主循环推进非空实例的 UART7 发送。FIFO 满时返回，实例保留剩余字节。 */
void imu_telemetry_poll(imu_telemetry_t *telemetry);

#endif /* FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_IMU_TELEMETRY_H */
