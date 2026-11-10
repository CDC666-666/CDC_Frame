#ifndef FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_MOTOR_TELEMETRY_H
#define FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_MOTOR_TELEMETRY_H

#include "damiao_motor.h"

#define MOTOR_TELEMETRY_FRAME_SIZE 72U

typedef struct {
    uint8_t frame[MOTOR_TELEMETRY_FRAME_SIZE];
    size_t offset;
} motor_telemetry_t;

typedef struct {
    damiao_motor_output_t motor;
    double pitch_rad;
    double target_speed_rad_s;
    double imu_sample_dt_s;
    uint32_t imu_missed_samples;
    uint8_t bench_state;
    bool is_armed;
    bool is_pitch_valid;
} motor_telemetry_snapshot_t;

/** @brief 初始化发送缓冲。由主循环独占。 */
void motor_telemetry_init(motor_telemetry_t *telemetry);

/** @brief 空闲时打包一帧。待发帧尚未发完则保留原帧。 */
void motor_telemetry_submit(motor_telemetry_t *telemetry,
                            const motor_telemetry_snapshot_t *snapshot);

/** @brief 非阻塞地推进 UART7 发送。 */
void motor_telemetry_poll(motor_telemetry_t *telemetry);

#endif /* FRAME_CDC_COMPONENTS_PROTOCOLS_TELEMETRY_MOTOR_TELEMETRY_H */
