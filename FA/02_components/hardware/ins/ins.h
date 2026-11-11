#ifndef FRAME_CDC_COMPONENTS_HARDWARE_INS_INS_H
#define FRAME_CDC_COMPONENTS_HARDWARE_INS_INS_H

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

/** 作用将独立 INS 实例初始化为单位四元数和等待观测状态。
 * 输入非空 ins，输出清零的连续性状态和初始化姿态，无返回值。 */
void ins_init(ins_t *ins);

/** 作用按观测有效性、时槽及时间连续性调用姿态估计，每次采样尝试都应调用。
 * 输入非空 ins 和 observation，同步消费后不留存输入指针，输出 ins.output 与前次槽号，无返回值。 */
void ins_update(ins_t *ins, const imu_observation_t *observation);

#endif /* FRAME_CDC_COMPONENTS_HARDWARE_INS_INS_H */
