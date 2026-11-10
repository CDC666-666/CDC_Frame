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
    can_service_submission_t submission;     /* 硬件最近接受的请求。 */
    can_service_transmission_t transmission; /* 最近由硬件确认发出的请求。 */
    uint32_t fault_count;
    uint32_t disabled_count;
    bool has_scaled_feedback;
} damiao_motor_output_t;

/** @brief 启动总线前初始化零值实例、复制配置并注册反馈。无物理使能副作用。
 * @details 非空实例及总线在主循环期间保持地址不变。未知量程允许反馈模式。
 * @return 初始化和注册均成功才返回 true。
 */
bool damiao_motor_init(damiao_motor_t *motor, can_service_t *bus,
                       const damiao_mit_config_t *config);

/** @brief 读取已初始化实例的运动配置。量程未知或实例未初始化时返回 NULL。
 * @details 返回实例拥有的只读配置，生命周期与实例相同。
 */
const damiao_mit_config_t *damiao_motor_motion_config(const damiao_motor_t *motor);

/** @brief 读取本实例的反馈、提交及发送确认。now_us 为板级微秒。 */
damiao_motor_output_t damiao_motor_output(const damiao_motor_t *motor, uint64_t now_us);

/** @brief 将有效 MIT 目标复制入队，截止时刻限制在当前使能反馈的有效期内。
 * @details QUEUED 仅表示进入软件队列。统一轮询 CAN 服务，并通过 output 查询实际发送。
 */
can_service_result_t damiao_motor_command(damiao_motor_t *motor,
                                          const damiao_mit_command_t *command, uint64_t now_us,
                                          uint64_t expires_us);

/** @brief 显式排队查询、使能或优先失能。失能清除本实例的旧运动目标。
 * @details 应用决定何时允许使能，并通过后续反馈确认电机状态。
 */
can_service_result_t damiao_motor_action(damiao_motor_t *motor, damiao_mit_action_t action,
                                         uint64_t expires_us);

/** @brief 确认电机已失能后，移除尚未提交的失能重试帧。 */
void damiao_motor_confirm_disabled(damiao_motor_t *motor);

#endif /* FRAME_CDC_COMPONENTS_DEVICES_DAMIAO_DAMIAO_MOTOR_H */
