#ifndef FRAME_CDC_COMPONENTS_DEVICES_IMU_IMU_DEVICE_H
#define FRAME_CDC_COMPONENTS_DEVICES_IMU_IMU_DEVICE_H

#include "bmi088.h"

typedef struct {
    double accel_body_m_s2[3];
    double gyro_body_rad_s[3];
    double temperature_deg_c;
    double sample_dt_s;
    double continuous_dt_s;
    uint64_t sample_time_us;
    uint32_t sample_sequence;
    uint32_t missed_samples;
    bmi088_status_t sample_status;
    bmi088_status_t accel_init_status;
    bmi088_status_t gyro_init_status;
    bool is_valid;
} imu_device_output_t;

typedef struct {
    bmi088_t sensor;
    imu_device_output_t output;
    uint64_t next_sample_us;
    uint64_t previous_sample_us;
    uint32_t slot_sequence;
    bool has_previous_sample;
} imu_device_t;

/** @brief 初始化 BMI088，等待传感器上电后启动 200 Hz 采样调度。 */
void imu_device_init(imu_device_t *imu);

/** @brief 按期更新 body 观测、时间戳和有效性，返回本轮是否已尝试采样。 */
bool imu_device_poll(imu_device_t *imu, uint64_t now_us);

#endif /* FRAME_CDC_COMPONENTS_DEVICES_IMU_IMU_DEVICE_H */
