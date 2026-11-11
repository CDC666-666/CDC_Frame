/* 达妙 MIT 的确定性协议编解码，不访问外设或设备历史。 */
#include "damiao_mit.h"

#include <math.h>
#include <string.h>

#define DAMIAO_MIT_KP_MAX 500.0
#define DAMIAO_MIT_MASK_16 0xFFFFU

/** 作用检查协议对称量程是否为正且双倍量程有限。
 * 输入量程上限 maximum，输出量程有效时为 true，否则为 false。 */
static bool is_range_valid(double maximum)
{
    return maximum > 0.0 && isfinite(2.0 * maximum);
}

/** 作用复制 MIT 配置并检查 ID、超时和量程，零量程表示仅解析原始反馈。
 * 输入非空 motor 和 config，输出配置后的 motor，ID 或超时无效返回 false，否则返回 true。 */
bool damiao_mit_init(damiao_mit_t *motor, const damiao_mit_config_t *config)
{
    *motor = (damiao_mit_t){.config = *config};
    motor->has_valid_ranges = is_range_valid(config->position_max_rad) &&
                              is_range_valid(config->speed_max_rad_s) &&
                              is_range_valid(config->torque_max_n_m);
    return config->motor_id > 0U && config->motor_id < 0x7FFU && config->master_id < 0x7FFU &&
           config->master_id != config->motor_id && config->feedback_timeout_us > 0U;
}

/** 作用将协议无符号整数还原为对称区间的物理量。
 * 输入原始值 raw、正量程 maximum 和量化掩码 mask，输出与 maximum 同单位的数值。 */
static double decode_signed(uint16_t raw, double maximum, uint16_t mask)
{
    return (2.0 * raw / mask - 1.0) * maximum;
}

/** 作用纯解码匹配的 Classic 标准帧，错误时保留原输出，历史时效由设备管理。
 * 输入非空 codec 配置、frame 和 output，输出解码反馈至 output，成功返回 true，格式或 ID 不符返回 false。 */
bool damiao_mit_decode_feedback(const damiao_mit_t *motor, const can_frame_t *frame,
                                damiao_mit_feedback_t *output)
{
    const uint8_t *data = frame->data;
    const uint8_t state = data[0] >> 4;
    if (frame->id != motor->config.master_id || frame->is_extended || frame->is_remote ||
        frame->is_fd || frame->length != 8U ||
        (data[0] & 0x0FU) != (motor->config.motor_id & 0x0FU)) {
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
    *output = feedback;
    return true;
}

/** 作用检查一个数值是否位于包含两端的指定区间。
 * 输入数值 value、有序下界 minimum 和上界 maximum，输出区间内为 true，否则为 false。 */
static bool is_in_range(double value, double minimum, double maximum)
{
    return value >= minimum && value <= maximum;
}

/** 作用将有效区间内的物理量映射并截断为协议整数。
 * 输入数值 value、有序界限 minimum/maximum 和掩码 mask，输出 uint16_t 量化值。 */
static uint16_t quantize(double value, double minimum, double maximum, uint16_t mask)
{
    /* 与厂家示例一致，正数区间映射后截断。 */
    return (uint16_t)((value - minimum) / (maximum - minimum) * mask);
}

/** 作用将输出轴 MIT 目标量化为帧，并拒绝越界、未知量程和特殊指令碰撞。
 * 输入非空 motor、command 和 frame，kp（N·m/rad）为 0..500、kd（N·m·s/rad）为 0..5，输出编码帧，成功返回 true。 */
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

/** 作用生成使能、失能或只读状态帧，只读请求需先核实电机固件支持。
 * 输入非空 motor、动作 action 和 frame，输出动作帧，支持的动作返回 true，其他动作返回 false。 */
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
