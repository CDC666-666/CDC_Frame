#ifndef FRAME_CDC_DEVICES_DAMIAO_DAMIAO_MIT_H
#define FRAME_CDC_DEVICES_DAMIAO_DAMIAO_MIT_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

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

typedef enum {
    DAMIAO_MIT_FEEDBACK_DISABLED = 0,
    DAMIAO_MIT_FEEDBACK_ENABLED,
    DAMIAO_MIT_FEEDBACK_UNSEEN,
    DAMIAO_MIT_FEEDBACK_STALE,
    DAMIAO_MIT_FEEDBACK_FAULT,
} damiao_mit_status_t;

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
    damiao_mit_feedback_t feedback;
    bool has_valid_ranges;
} damiao_mit_t;

typedef enum {
    DAMIAO_MIT_DISABLE = 0,
    DAMIAO_MIT_ENABLE,
    DAMIAO_MIT_READ_STATUS,
} damiao_mit_action_t;

/** @brief 初始化实例并复制配置。所有指针非空，仅主循环使用。
 * @return ID 或超时配置无效时返回 false。未知量程可填零，只解析状态和温度。
 */
bool damiao_mit_init(damiao_mit_t *motor, const damiao_mit_config_t *config);

/** @brief 接收匹配的 Classic 标准数据帧。错误帧不更新反馈或时间。 */
bool damiao_mit_accept_feedback(damiao_mit_t *motor, const can_frame_t *frame);

/** @brief 查询当前反馈状态。now_us 与帧时间使用同一单调时钟。 */
damiao_mit_status_t damiao_mit_status(const damiao_mit_t *motor, uint64_t now_us);

/** @brief 将输出轴坐标下的 MIT 命令量化为 CAN 帧。越界或量程未知返回 false。
 * @details kp 范围 0..500 N·m/rad，kd 范围 0..5 N·m·s/rad。输出由调用方持有。
 */
bool damiao_mit_encode_command(const damiao_mit_t *motor, const damiao_mit_command_t *command,
                               can_frame_t *frame);

/** @brief 生成使能、失能或只读状态帧。状态请求须经电机固件版本确认。 */
bool damiao_mit_encode_action(const damiao_mit_t *motor, damiao_mit_action_t action,
                              can_frame_t *frame);

#endif /* FRAME_CDC_DEVICES_DAMIAO_DAMIAO_MIT_H */
