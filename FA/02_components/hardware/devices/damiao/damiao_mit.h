#ifndef FRAME_CDC_COMPONENTS_HARDWARE_DEVICES_DAMIAO_DAMIAO_MIT_H
#define FRAME_CDC_COMPONENTS_HARDWARE_DEVICES_DAMIAO_DAMIAO_MIT_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define DAMIAO_MIT_KD_MAX 5.0
#define DAMIAO_MIT_MASK_12 0xFFFU

typedef struct {
    uint16_t motor_id;
    uint16_t master_id;
    double position_max_rad;
    double speed_max_rad_s;
    double torque_max_n_m;
    uint64_t feedback_timeout_us;
} damiao_mit_config_t;

typedef struct {
    double position_rad;
    double speed_rad_s;
    double kp_n_m_per_rad;
    double kd_n_m_s_per_rad;
    double feedforward_n_m;
} damiao_mit_command_t;

typedef struct {
    uint64_t time_us;
    double position_rad;
    double speed_rad_s;
    double torque_n_m;
    uint8_t mos_deg_c;
    uint8_t rotor_deg_c;
    uint8_t state_raw;
    bool has_feedback;
} damiao_mit_feedback_t;

typedef struct {
    damiao_mit_config_t config;
    bool has_valid_ranges;
} damiao_mit_t;

typedef enum {
    DAMIAO_MIT_DISABLE = 0,
    DAMIAO_MIT_ENABLE,
    DAMIAO_MIT_READ_STATUS,
} damiao_mit_action_t;

/** 作用复制 MIT 配置并检查 ID、超时和量程，零量程表示仅解析原始反馈。
 * 输入非空 motor 和 config，输出配置后的 motor，ID 或超时无效返回 false，否则返回 true。 */
bool damiao_mit_init(damiao_mit_t *motor, const damiao_mit_config_t *config);

/** 作用纯解码匹配的 Classic 标准帧，错误时保留原输出，历史时效由设备管理。
 * 输入非空 codec 配置、frame 和 output，输出解码反馈至 output，成功返回 true，格式或 ID 不符返回 false。 */
bool damiao_mit_decode_feedback(const damiao_mit_t *codec, const can_frame_t *frame,
                                damiao_mit_feedback_t *output);

/** 作用将输出轴 MIT 目标量化为帧，并拒绝越界、未知量程和特殊指令碰撞。
 * 输入非空 motor、command 和 frame，kp（N·m/rad）为 0..500、kd（N·m·s/rad）为 0..5，输出编码帧，成功返回 true。 */
bool damiao_mit_encode_command(const damiao_mit_t *motor, const damiao_mit_command_t *command,
                               can_frame_t *frame);

/** 作用生成使能、失能或只读状态帧，只读请求需先核实电机固件支持。
 * 输入非空 motor、动作 action 和 frame，输出动作帧，支持的动作返回 true，其他动作返回 false。 */
bool damiao_mit_encode_action(const damiao_mit_t *motor, damiao_mit_action_t action,
                              can_frame_t *frame);

#endif /* FRAME_CDC_COMPONENTS_HARDWARE_DEVICES_DAMIAO_DAMIAO_MIT_H */
