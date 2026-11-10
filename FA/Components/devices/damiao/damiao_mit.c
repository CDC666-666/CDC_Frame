/* 达妙 MIT 单电机的编解码及反馈生命期。 */
#include "damiao_mit.h"

#include <math.h>
#include <string.h>

#define DAMIAO_MIT_KP_MAX 500.0
#define DAMIAO_MIT_KD_MAX 5.0
#define DAMIAO_MIT_MASK_12 0xFFFU
#define DAMIAO_MIT_MASK_16 0xFFFFU

static bool is_range_valid(double maximum)
{
    return maximum > 0.0 && isfinite(2.0 * maximum);
}

bool damiao_mit_init(damiao_mit_t *motor, const damiao_mit_config_t *config)
{
    *motor = (damiao_mit_t){.config = *config};
    motor->has_valid_ranges = is_range_valid(config->position_max_rad) &&
                              is_range_valid(config->speed_max_rad_s) &&
                              is_range_valid(config->torque_max_n_m);
    return config->motor_id > 0U && config->motor_id < 0x7FFU && config->master_id < 0x7FFU &&
           config->master_id != config->motor_id && config->feedback_timeout_us > 0U;
}

static double decode_signed(uint16_t raw, double maximum, uint16_t mask)
{
    return (2.0 * raw / mask - 1.0) * maximum;
}

bool damiao_mit_accept_feedback(damiao_mit_t *motor, const can_frame_t *frame)
{
    const uint8_t *data = frame->data;
    const uint8_t state = data[0] >> 4;
    if (frame->id != motor->config.master_id || frame->is_extended || frame->is_remote ||
        frame->is_fd || frame->length != 8U ||
        (data[0] & 0x0FU) != (motor->config.motor_id & 0x0FU) ||
        (motor->feedback.has_feedback && frame->rx_time_us < motor->feedback.time_us)) {
        return false;
    }

    damiao_mit_feedback_t feedback = {
        .time_us = frame->rx_time_us,
        .mos_deg_c = data[6],
        .rotor_deg_c = data[7],
        .state_raw = state,
        .has_feedback = true,
    };
    if (motor->has_valid_ranges) {
        const uint16_t position = ((uint16_t)data[1] << 8) | data[2];
        const uint16_t speed = ((uint16_t)data[3] << 4) | (data[4] >> 4);
        const uint16_t torque = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
        feedback.position_rad =
            decode_signed(position, motor->config.position_max_rad, DAMIAO_MIT_MASK_16);
        feedback.speed_rad_s =
            decode_signed(speed, motor->config.speed_max_rad_s, DAMIAO_MIT_MASK_12);
        feedback.torque_n_m =
            decode_signed(torque, motor->config.torque_max_n_m, DAMIAO_MIT_MASK_12);
    }
    motor->feedback = feedback;
    return true;
}

damiao_mit_status_t damiao_mit_status(const damiao_mit_t *motor, uint64_t now_us)
{
    if (!motor->feedback.has_feedback) {
        return DAMIAO_MIT_FEEDBACK_UNSEEN;
    }
    if (now_us - motor->feedback.time_us >= motor->config.feedback_timeout_us) {
        return DAMIAO_MIT_FEEDBACK_STALE;
    }
    switch (motor->feedback.state_raw) {
        case 0:
            return DAMIAO_MIT_FEEDBACK_DISABLED;
        case 1:
            return DAMIAO_MIT_FEEDBACK_ENABLED;
        default:
            return DAMIAO_MIT_FEEDBACK_FAULT;
    }
}

static bool is_in_range(double value, double minimum, double maximum)
{
    return value >= minimum && value <= maximum;
}

static uint16_t quantize(double value, double minimum, double maximum, uint16_t mask)
{
    /* 与厂家示例一致，正数区间映射后截断。 */
    return (uint16_t)((value - minimum) / (maximum - minimum) * mask);
}

bool damiao_mit_encode_command(const damiao_mit_t *motor, const damiao_mit_command_t *command,
                               can_frame_t *frame)
{
    const damiao_mit_config_t *config = &motor->config;
    if (!motor->has_valid_ranges ||
        !is_in_range(command->position_rad, -config->position_max_rad, config->position_max_rad) ||
        !is_in_range(command->speed_rad_s, -config->speed_max_rad_s, config->speed_max_rad_s) ||
        !is_in_range(command->feedforward_n_m, -config->torque_max_n_m, config->torque_max_n_m) ||
        !is_in_range(command->kp_n_m_per_rad, 0.0, DAMIAO_MIT_KP_MAX) ||
        !is_in_range(command->kd_n_m_s_per_rad, 0.0, DAMIAO_MIT_KD_MAX)) {
        return false;
    }

    const uint16_t position = quantize(command->position_rad, -config->position_max_rad,
                                       config->position_max_rad, DAMIAO_MIT_MASK_16);
    const uint16_t speed = quantize(command->speed_rad_s, -config->speed_max_rad_s,
                                    config->speed_max_rad_s, DAMIAO_MIT_MASK_12);
    const uint16_t kp =
        quantize(command->kp_n_m_per_rad, 0.0, DAMIAO_MIT_KP_MAX, DAMIAO_MIT_MASK_12);
    const uint16_t kd =
        quantize(command->kd_n_m_s_per_rad, 0.0, DAMIAO_MIT_KD_MAX, DAMIAO_MIT_MASK_12);
    const uint16_t torque = quantize(command->feedforward_n_m, -config->torque_max_n_m,
                                     config->torque_max_n_m, DAMIAO_MIT_MASK_12);
    *frame = (can_frame_t){
        .id = config->motor_id,
        .length = 8U,
        .data = {(uint8_t)(position >> 8), (uint8_t)position, (uint8_t)(speed >> 4),
                 (uint8_t)((speed << 4) | (kp >> 8)), (uint8_t)kp, (uint8_t)(kd >> 4),
                 (uint8_t)((kd << 4) | (torque >> 8)), (uint8_t)torque},
    };

    /* 避免极限值量化后与厂家保留的 7×FF 特殊指令发生碰撞。 */
    static const uint8_t s_special_prefix[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    return memcmp(frame->data, s_special_prefix, sizeof(s_special_prefix)) != 0;
}

bool damiao_mit_encode_action(const damiao_mit_t *motor, damiao_mit_action_t action,
                              can_frame_t *frame)
{
    *frame = (can_frame_t){.id = motor->config.motor_id, .length = 8U};
    switch (action) {
        case DAMIAO_MIT_DISABLE:
        case DAMIAO_MIT_ENABLE:
            memset(frame->data, 0xFF, 7U);
            frame->data[7] = action == DAMIAO_MIT_ENABLE ? 0xFCU : 0xFDU;
            return true;
        case DAMIAO_MIT_READ_STATUS:
            frame->id = 0x7FFU;
            frame->data[0] = (uint8_t)motor->config.motor_id;
            frame->data[1] = (uint8_t)(motor->config.motor_id >> 8);
            frame->data[2] = 0xCCU;
            return true;
        default:
            return false;
    }
}
