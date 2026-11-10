/* 静态装配首条验证链，在主循环调度采样、估计和遥测。 */
#include "imu_app.h"

#include <stddef.h>
#include <stdint.h>

#include "attitude.h"
#include "bmi088.h"
#include "imu_telemetry.h"
#include "mc02_board.h"

#define IMU_APP_SAMPLE_PERIOD_US 5000U
#define IMU_APP_TELEMETRY_PERIOD_US 20000U

static bool s_board_is_ready;
static bmi088_t s_imu;
static attitude_t s_attitude;
static attitude_observation_t s_body_observation;
static imu_telemetry_t s_telemetry;
static uint64_t s_next_sample_us;
static uint64_t s_next_telemetry_us;
static uint64_t s_previous_sample_us;
static bool s_has_previous_sample;
static bool s_previous_sample_is_valid;
static uint32_t s_slot_sequence;
static uint32_t s_missed_samples;
static double s_dt_s;

static void sensor_to_body(const double sensor[3], double body[3])
{
    /* 验证夹具的 body 沿 BMI088 数据轴定义。整车安装旋转在这里替换。 */
    for (size_t i = 0; i < 3; ++i) {
        body[i] = sensor[i];
    }
}

static void sample_and_estimate(uint64_t now_us, uint64_t elapsed_slots)
{
    s_slot_sequence += (uint32_t)elapsed_slots;
    s_missed_samples += (uint32_t)(elapsed_slots - 1U);
    bmi088_sample(&s_imu, now_us, s_slot_sequence);
    const bmi088_sample_t *sample = &s_imu.sample;
    s_body_observation.is_valid = sample->is_valid;
    s_dt_s = 0.0;
    double estimator_dt_s = 0.0;
    if (sample->is_valid) {
        sensor_to_body(sample->accel_sensor_m_s2, s_body_observation.accel_body_m_s2);
        sensor_to_body(sample->gyro_sensor_rad_s, s_body_observation.gyro_body_rad_s);
        if (s_has_previous_sample) {
            s_dt_s = (double)(sample->time_us - s_previous_sample_us) * 1e-6;
        }
        if (elapsed_slots == 1U && s_previous_sample_is_valid) {
            estimator_dt_s = s_dt_s;
        }
        s_previous_sample_us = sample->time_us;
        s_has_previous_sample = true;
    } else {
        ++s_missed_samples;
    }

    attitude_update(&s_attitude, &s_body_observation, estimator_dt_s);
    s_previous_sample_is_valid =
        sample->is_valid && s_attitude.status != ATTITUDE_STATUS_INVALID_SAMPLE;
}

void imu_app_init(void)
{
    s_board_is_ready = mc02_board_init();
    if (!s_board_is_ready) {
        return;
    }
    bmi088_init(&s_imu);
    attitude_init(&s_attitude);
    imu_telemetry_init(&s_telemetry);
    s_next_sample_us = mc02_board_time_us();
    s_next_telemetry_us = s_next_sample_us;
}

void imu_app_poll(void)
{
    if (!s_board_is_ready) {
        return;
    }
    const uint64_t now_us = mc02_board_time_us();
    if (now_us >= s_next_sample_us) {
        const uint64_t elapsed_slots = (now_us - s_next_sample_us) / IMU_APP_SAMPLE_PERIOD_US + 1U;
        s_next_sample_us += elapsed_slots * IMU_APP_SAMPLE_PERIOD_US;
        sample_and_estimate(now_us, elapsed_slots);
    }
    if (now_us >= s_next_telemetry_us) {
        /* 延误时只发最新状态，保持原有相位。 */
        const uint64_t slots = (now_us - s_next_telemetry_us) / IMU_APP_TELEMETRY_PERIOD_US + 1U;
        s_next_telemetry_us += slots * IMU_APP_TELEMETRY_PERIOD_US;
        const bmi088_sample_t *sample = &s_imu.sample;
        const imu_telemetry_snapshot_t snapshot = {
            .body_observation = s_body_observation,
            .attitude = s_attitude,
            .temperature_deg_c = sample->temperature_deg_c,
            .sample_dt_s = s_dt_s,
            .sample_time_us = sample->time_us,
            .sample_sequence = sample->sequence,
            .missed_samples = s_missed_samples,
            .sample_status = sample->status,
            .accel_init_status = s_imu.accel_init_status,
            .gyro_init_status = s_imu.gyro_init_status,
        };
        imu_telemetry_submit(&s_telemetry, &snapshot, (uint32_t)(slots - 1U));
    }
    imu_telemetry_poll(&s_telemetry);
}
