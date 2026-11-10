/* 采样调度和验证夹具的 body 轴适配。 */
#include "imu_observer.h"

#include <string.h>

#define IMU_OBSERVER_SAMPLE_PERIOD_US 5000U

void imu_observer_init(imu_observer_t *observer, bmi088_t *sensor, uint64_t first_sample_us)
{
    *observer = (imu_observer_t){
        .sensor = sensor,
        .next_sample_us = first_sample_us,
        .output =
            {
                .accel_init_status = sensor->accel_init_status,
                .gyro_init_status = sensor->gyro_init_status,
            },
    };
}

bool imu_observer_update(imu_observer_t *observer, uint64_t now_us)
{
    if (now_us < observer->next_sample_us) {
        return false;
    }
    const uint64_t slots = (now_us - observer->next_sample_us) / IMU_OBSERVER_SAMPLE_PERIOD_US + 1U;
    observer->next_sample_us += slots * IMU_OBSERVER_SAMPLE_PERIOD_US;
    observer->slot_sequence += (uint32_t)slots;
    imu_observer_output_t *output = &observer->output;
    output->missed_samples += (uint32_t)(slots - 1U);
    bmi088_sample(observer->sensor, now_us, observer->slot_sequence);

    const bmi088_sample_t *sample = &observer->sensor->sample;
    imu_observation_t *observation = &output->observation;
    output->sample_dt_s = 0.0;
    output->sample_status = sample->status;
    observation->body.is_valid = sample->is_valid;
    if (sample->is_valid) {
        /* 验证夹具的 body 轴与 BMI088 数据轴一致。 */
        memcpy(observation->body.accel_body_m_s2, sample->accel_sensor_m_s2,
               sizeof(observation->body.accel_body_m_s2));
        memcpy(observation->body.gyro_body_rad_s, sample->gyro_sensor_rad_s,
               sizeof(observation->body.gyro_body_rad_s));
        if (observer->has_previous_sample) {
            output->sample_dt_s = (double)(sample->time_us - observation->sample_time_us) * 1e-6;
        }
        observer->has_previous_sample = true;
        observation->sample_time_us = sample->time_us;
        observation->sample_sequence = sample->sequence;
        output->temperature_deg_c = sample->temperature_deg_c;
    } else {
        ++output->missed_samples;
    }
    return true;
}
