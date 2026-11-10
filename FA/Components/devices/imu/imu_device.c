/* 夹具的 BMI088 采样、body 观测适配及采样时序。 */
#include "imu_device.h"

#include <string.h>

#include "mc02_board.h"

#define IMU_DEVICE_SAMPLE_PERIOD_US 5000U

void imu_device_init(imu_device_t *imu)
{
    *imu = (imu_device_t){0};
    bmi088_init(&imu->sensor);
    imu->output.accel_init_status = imu->sensor.accel_init_status;
    imu->output.gyro_init_status = imu->sensor.gyro_init_status;
    imu->next_sample_us = mc02_board_time_us();
}

bool imu_device_poll(imu_device_t *imu, uint64_t now_us)
{
    if (now_us < imu->next_sample_us) {
        return false;
    }
    const uint64_t slots = (now_us - imu->next_sample_us) / IMU_DEVICE_SAMPLE_PERIOD_US + 1U;
    imu->next_sample_us += slots * IMU_DEVICE_SAMPLE_PERIOD_US;
    imu->slot_sequence += (uint32_t)slots;
    imu_device_output_t *output = &imu->output;
    output->missed_samples += (uint32_t)(slots - 1U);
    const bool was_valid = output->is_valid;
    bmi088_sample(&imu->sensor, now_us, imu->slot_sequence);
    const bmi088_sample_t *sample = &imu->sensor.sample;
    output->is_valid = sample->is_valid;
    output->sample_time_us = sample->time_us;
    output->sample_sequence = sample->sequence;
    output->sample_status = sample->status;
    output->temperature_deg_c = sample->temperature_deg_c;
    output->sample_dt_s = 0.0;
    output->continuous_dt_s = 0.0;
    if (sample->is_valid) {
        /* 验证夹具的 body 轴与 BMI088 数据轴一致。 */
        memcpy(output->accel_body_m_s2, sample->accel_sensor_m_s2, sizeof(output->accel_body_m_s2));
        memcpy(output->gyro_body_rad_s, sample->gyro_sensor_rad_s, sizeof(output->gyro_body_rad_s));
        if (imu->has_previous_sample) {
            output->sample_dt_s = (double)(sample->time_us - imu->previous_sample_us) * 1e-6;
        }
        if (slots == 1U && was_valid) {
            output->continuous_dt_s = output->sample_dt_s;
        }
        imu->previous_sample_us = sample->time_us;
        imu->has_previous_sample = true;
    } else {
        ++output->missed_samples;
    }
    return true;
}
