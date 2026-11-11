#ifndef FRAME_CDC_COMPONENTS_TELEMETRY_MOTOR_TELEMETRY_H
#define FRAME_CDC_COMPONENTS_TELEMETRY_MOTOR_TELEMETRY_H

#include <stddef.h>

#include "damiao_motor.h"
#include "imu_observer.h"
#include "speed_controller.h"
#include "telemetry_encoding.h"

typedef struct {
    uint8_t frame[MOTOR_TELEMETRY_FRAME_SIZE];
    size_t offset;
    uint64_t next_frame_us;
    uint64_t period_us;
    size_t (*write_bytes)(const uint8_t *data, size_t size);
} motor_telemetry_t;

/** 作用初始化电机遥测帧缓冲、周期调度和非阻塞写入接口。
 * 输入非空 telemetry、同源 first_frame_us、正 period_us 和 write_bytes，输出就绪的 telemetry，无返回值。 */
void motor_telemetry_init(motor_telemetry_t *telemetry, uint64_t first_frame_us, uint64_t period_us,
                          size_t (*write_bytes)(const uint8_t *, size_t));

/** 作用按周期选取设备、目标、IMU 和 INS 快照并非阻塞推进输出，输入本次同步消费。
 * 输入非空 telemetry、motor、control、imu、pose 和 now_us，输出快照帧、发送偏移和调度状态，无返回值。 */
void motor_telemetry_update(motor_telemetry_t *telemetry, const damiao_motor_output_t *motor,
                            const speed_controller_output_t *control,
                            const imu_observer_output_t *imu, const attitude_sample_t *pose,
                            uint64_t now_us);

#endif /* FRAME_CDC_COMPONENTS_TELEMETRY_MOTOR_TELEMETRY_H */
