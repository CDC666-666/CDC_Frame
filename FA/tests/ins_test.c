/* 直接向生产 INS 注入合成 body 观测，无板级或设备替身。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ins.h"

int main(void)
{
    ins_t level;
    ins_t tilted;
    ins_init(&level);
    ins_init(&tilted);
    assert(level.output.attitude.status == ATTITUDE_STATUS_WAITING);
    assert(!level.output.is_sample_valid);

    imu_observation_t observation = {
        .body = {.accel_body_m_s2 = {0.0, 0.0, 9.80665}, .is_valid = true},
        .sample_time_us = 1000U,
        .sample_sequence = UINT32_MAX,
    };
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_OK);
    assert(level.output.is_sample_valid && level.output.sample_time_us == 1000U);

    const double expected_pitch_rad = 0.3;
    imu_observation_t inclined = observation;
    inclined.body.accel_body_m_s2[0] = -9.80665 * sin(expected_pitch_rad);
    inclined.body.accel_body_m_s2[2] = 9.80665 * cos(expected_pitch_rad);
    ins_update(&tilted, &inclined);
    double pitch_rad = 0.0;
    assert(attitude_get_pitch_rad(&tilted.output.attitude, &pitch_rad));
    assert(fabs(pitch_rad - expected_pitch_rad) < 1e-12);

    observation.body.gyro_body_rad_s[2] = 1.0;
    observation.sample_time_us += 5000U;
    ++observation.sample_sequence; /* 时槽从 UINT32_MAX 回绕到 0。 */
    ins_update(&level, &observation);
    const double *q = level.output.attitude.q_odom_from_body;
    assert(level.output.attitude.status == ATTITUDE_STATUS_OK);
    assert(fabs(q[3] / q[0] - 0.0025) < 1e-12);
    assert(fabs(q[0] * q[0] + q[3] * q[3] - 1.0) < 1e-12);
    double previous_q[4];
    memcpy(previous_q, q, sizeof(previous_q));

    observation.body.is_valid = false; /* 无效观测保留最近成功的时间戳和时槽。 */
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_INVALID_SAMPLE);
    assert(!level.output.is_sample_valid && level.output.sample_time_us == 6000U);

    observation.body.is_valid = true;
    observation.sample_time_us += 10000U;
    observation.sample_sequence += 2U;
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_GAP);

    observation.sample_time_us += 10000U;
    observation.sample_sequence += 2U;
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_GAP);

    observation.sample_time_us += 30000U; /* 连号但超过算法允许的积分间隔。 */
    ++observation.sample_sequence;
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_GAP);

    --observation.sample_time_us; /* 连号但时间倒退。 */
    ++observation.sample_sequence;
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_GAP);
    assert(memcmp(previous_q, q, sizeof(previous_q)) == 0);

    observation.sample_time_us += 5000U;
    ++observation.sample_sequence;
    ins_update(&level, &observation);
    assert(level.output.attitude.status == ATTITUDE_STATUS_OK);
    assert(q[3] > previous_q[3]);
    assert(attitude_get_pitch_rad(&tilted.output.attitude, &pitch_rad));
    assert(fabs(pitch_rad - expected_pitch_rad) < 1e-12);

    puts("INS synthetic input, instance independence and continuity checks passed");
    return 0;
}
