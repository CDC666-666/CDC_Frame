/* 按 body 观测的时间与时槽推进姿态估计。 */
#include "ins.h"

/** 作用将独立 INS 实例初始化为单位四元数和等待观测状态。
 * 输入非空 ins，输出清零的连续性状态和初始化姿态，无返回值。 */
void ins_init(ins_t *ins)
{
    *ins = (ins_t){0};
    attitude_init(&ins->output.attitude);
}

/** 作用按观测有效性、时槽及时间连续性调用姿态估计，每次采样尝试都应调用。
 * 输入非空 ins 和 observation，同步消费后不留存输入指针，输出 ins.output 与前次槽号，无返回值。 */
void ins_update(ins_t *ins, const imu_observation_t *observation)
{
    attitude_sample_t *output = &ins->output;
    double dt_s = 0.0;
    if (observation->body.is_valid && output->is_sample_valid &&
        output->attitude.status != ATTITUDE_STATUS_INVALID_SAMPLE &&
        observation->sample_sequence == (uint32_t)(ins->previous_sequence + 1U) &&
        observation->sample_time_us > output->sample_time_us) {
        dt_s = (double)(observation->sample_time_us - output->sample_time_us) * 1e-6;
    }

    attitude_update(&output->attitude, &observation->body, dt_s);
    output->sample_time_us = observation->sample_time_us;
    output->is_sample_valid = observation->body.is_valid;
    ins->previous_sequence = observation->sample_sequence;
}
