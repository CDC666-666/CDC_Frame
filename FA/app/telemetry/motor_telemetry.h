#ifndef FRAME_CDC_APP_TELEMETRY_MOTOR_TELEMETRY_H
#define FRAME_CDC_APP_TELEMETRY_MOTOR_TELEMETRY_H

#include <stddef.h>

#include "imu_observer.h"
#include "single_motor_control.h"
#include "telemetry_encoding.h"

typedef struct {
    uint8_t frame[MOTOR_TELEMETRY_FRAME_SIZE];
    size_t offset;
    uint64_t next_frame_us;
} motor_telemetry_t;

/** @brief 初始化实例。first_frame_us 与板级时钟同源，单位为微秒。 */
void motor_telemetry_init(motor_telemetry_t *telemetry, uint64_t first_frame_us);

/** @brief 按 20 ms 周期选取快照，非阻塞推进 UART7。仅主循环调用，不留存输入指针。 */
void motor_telemetry_update(motor_telemetry_t *telemetry,
                            const single_motor_control_output_t *control,
                            const imu_observer_output_t *imu, uint64_t now_us);

#endif /* FRAME_CDC_APP_TELEMETRY_MOTOR_TELEMETRY_H */
