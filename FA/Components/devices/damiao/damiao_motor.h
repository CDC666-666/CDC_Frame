#ifndef FRAME_CDC_COMPONENTS_DEVICES_DAMIAO_DAMIAO_MOTOR_H
#define FRAME_CDC_COMPONENTS_DEVICES_DAMIAO_DAMIAO_MOTOR_H

#include "can_service.h"
#include "damiao_mit.h"

typedef struct {
    damiao_mit_t mit;
    can_service_endpoint_t *endpoint;
    uint32_t fault_count;
    uint32_t disabled_count;
} damiao_motor_t;

typedef struct {
    damiao_mit_feedback_t feedback;
    damiao_mit_status_t status;
    mc02_can_status_t bus_status;
    can_service_submission_t submission;
    uint32_t fault_count;
    uint32_t disabled_count;
    bool has_scaled_feedback;
} damiao_motor_output_t;

/** @brief 将零初始化的实例注册到指定总线。总线和实例均须活到主循环结束。 */
bool damiao_motor_register(damiao_motor_t *motor, can_service_t *bus,
                           const damiao_mit_config_t *config);

/** @brief 读取本实例最新反馈和提交事件。时间单位为板级微秒。 */
damiao_motor_output_t damiao_motor_output(const damiao_motor_t *motor, uint64_t now_us);

/** @brief 将合法命令排队，截止时刻不会超过当前使能反馈的有效期。 */
can_service_result_t damiao_motor_command(damiao_motor_t *motor,
                                          const damiao_mit_command_t *command, uint64_t now_us,
                                          uint64_t expires_us);

/** @brief 排队查询、使能或优先失能。失能清除本实例的旧运动目标。 */
can_service_result_t damiao_motor_action(damiao_motor_t *motor, damiao_mit_action_t action,
                                         uint64_t expires_us);

/** @brief 确认电机已失能后，移除尚未提交的失能重试帧。 */
void damiao_motor_confirm_disabled(damiao_motor_t *motor);

#endif /* FRAME_CDC_COMPONENTS_DEVICES_DAMIAO_DAMIAO_MOTOR_H */
