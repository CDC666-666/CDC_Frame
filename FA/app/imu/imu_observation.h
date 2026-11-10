#ifndef FRAME_CDC_APP_IMU_IMU_OBSERVATION_H
#define FRAME_CDC_APP_IMU_IMU_OBSERVATION_H

#include <stdint.h>

#include "attitude.h"

typedef struct {
    attitude_observation_t body; /* 夹具 body 系，比力 m/s²、角速度 rad/s。 */
    uint64_t sample_time_us;     /* 同一来源的单调微秒时钟。 */
    uint32_t sample_sequence;    /* 采样槽序号，按 32 位回绕，跳槽时跳号。 */
} imu_observation_t;

#endif /* FRAME_CDC_APP_IMU_IMU_OBSERVATION_H */
