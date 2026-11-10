#ifndef FRAME_CDC_APP_IMU_IMU_OBSERVER_H
#define FRAME_CDC_APP_IMU_IMU_OBSERVER_H

#include "bmi088.h"
#include "imu_observation.h"

typedef struct {
    imu_observation_t observation;
    double temperature_deg_c;
    double sample_dt_s; /* 相邻成功采样的间隔，当前失败或首次成功时为 0。 */
    uint32_t missed_samples;
    bmi088_status_t sample_status;
    bmi088_status_t accel_init_status;
    bmi088_status_t gyro_init_status;
} imu_observer_output_t;

typedef struct {
    bmi088_t *sensor;
    imu_observer_output_t output;
    uint64_t next_sample_us;
    uint32_t slot_sequence;
    bool has_previous_sample;
} imu_observer_t;

/** @brief 绑定已初始化的 BMI088，first_sample_us 为板级微秒。实例和设备均由主循环持有。 */
void imu_observer_init(imu_observer_t *observer, bmi088_t *sensor, uint64_t first_sample_us);

/** @brief 按 5 ms 时槽更新观测。返回 true 表示本轮尝试过采样，结果见 output。 */
bool imu_observer_update(imu_observer_t *observer, uint64_t now_us);

#endif /* FRAME_CDC_APP_IMU_IMU_OBSERVER_H */
