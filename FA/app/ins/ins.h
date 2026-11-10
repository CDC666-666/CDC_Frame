#ifndef FRAME_CDC_APP_INS_INS_H
#define FRAME_CDC_APP_INS_INS_H

#include "imu_observation.h"

typedef struct {
    attitude_t attitude;     /* 状态 OK 或 GYRO_ONLY 表示估计有效。 */
    uint64_t sample_time_us; /* 最近一次有效设备采样的微秒时间。 */
    bool is_sample_valid;    /* 本轮观测源报告的有效位。 */
} attitude_sample_t;

typedef struct {
    attitude_sample_t output;
    uint32_t previous_sequence;
} ins_t;

/** @brief 初始化非空实例。状态由调用方持有，每个实例独立。 */
void ins_init(ins_t *ins);

/** @brief 每次采样尝试调用一次，含无效样本。同步消费非空观测，无指针留存。 */
void ins_update(ins_t *ins, const imu_observation_t *observation);

#endif /* FRAME_CDC_APP_INS_INS_H */
