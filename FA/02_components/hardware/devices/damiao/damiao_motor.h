#ifndef FRAME_CDC_COMPONENTS_HARDWARE_DEVICES_DAMIAO_DAMIAO_MOTOR_H
#define FRAME_CDC_COMPONENTS_HARDWARE_DEVICES_DAMIAO_DAMIAO_MOTOR_H

#include "can_service.h"
#include "damiao_mit.h"

typedef enum {
    DAMIAO_MIT_FEEDBACK_DISABLED = 0,
    DAMIAO_MIT_FEEDBACK_ENABLED,
    DAMIAO_MIT_FEEDBACK_UNSEEN,
    DAMIAO_MIT_FEEDBACK_STALE,
    DAMIAO_MIT_FEEDBACK_FAULT,
} damiao_mit_status_t;

typedef enum {
    DAMIAO_MOTOR_DISARMED = 0,
    DAMIAO_MOTOR_STOPPING,
    DAMIAO_MOTOR_ARMING,
    DAMIAO_MOTOR_ARMED,
} damiao_motor_state_t;

typedef struct {
    uint64_t enable_timeout_us;
    uint64_t status_period_us;
    uint64_t disable_retry_us;
    double kd_n_m_s_per_rad;
    bool has_status_request;
} damiao_motor_execution_config_t;

typedef struct {
    double target_speed_rad_s;
    uint64_t target_expires_us; /* 板级单调微秒时钟的绝对截止时间。 */
    bool has_target;
    bool should_enable;
    bool should_stop;
} damiao_motor_input_t;

typedef struct {
    damiao_mit_feedback_t feedback;
    damiao_mit_status_t status;
    mc02_can_status_t bus_status;
    can_service_submission_t submission;     /* 硬件最近接受的请求。 */
    can_service_transmission_t transmission; /* 最近由硬件确认发出的请求。 */
    uint32_t fault_count;
    uint32_t disabled_count;
    damiao_motor_state_t state; /* 设备执行状态，依据新鲜反馈确认启停。 */
    bool has_scaled_feedback;
} damiao_motor_output_t;

typedef struct {
    damiao_mit_t mit;
    damiao_mit_feedback_t feedback;
    can_service_endpoint_t *endpoint;
    damiao_motor_output_t output;
    damiao_motor_execution_config_t execution;
    uint64_t next_action_us;
    uint64_t arm_status_delay_us;
    uint64_t request_us;
    uint64_t target_expires_us;
    uint64_t arm_deadline_us;
    uint64_t last_poll_us;
    uint32_t submission_sequence;
    uint32_t fault_count;
    uint32_t disabled_count;
    bool can_move;
    bool is_arm_safe;
    bool has_request_submission;
    bool should_probe_stop;
} damiao_motor_t;

/** 作用初始化软件设备并注册独立反馈端点，主循环持有的实例和总线须地址稳定。
 * 输入非空零初始化 motor、未启动 bus 和 config，输出初始化实例，成功返回 true，重复或注册失败返回 false。 */
bool damiao_motor_init(damiao_motor_t *motor, can_service_t *bus,
                       const damiao_mit_config_t *config);

/** 作用取得已注册且量程有效的电机配置供调用方只读使用。
 * 输入非空 motor，输出实例拥有的配置指针，量程未知或未初始化返回 NULL，指针寿命同实例。 */
const damiao_mit_config_t *damiao_motor_motion_config(const damiao_motor_t *motor);

/** 作用在启动阶段配置速度执行保护并建立初始确认状态。
 * 输入非空 motor、config、allow_motion、command_period_us 和 now_us，输出执行状态，满足运动条件返回 true。 */
bool damiao_motor_configure_execution(damiao_motor_t *motor,
                                      const damiao_motor_execution_config_t *config,
                                      bool allow_motion, uint64_t command_period_us,
                                      uint64_t now_us);

/** 作用发布反馈快照并处理故障、轮询间断、使能确认和确认超时。
 * 输入非空 motor 和同源 now_us（微秒），输出 motor.output 及执行保护状态，无返回值。 */
void damiao_motor_update_feedback(damiao_motor_t *motor, uint64_t now_us);

/** 作用在反馈阶段之后消费启停与有时效目标，推进设备保护并准备 CAN 统一发送。
 * 输入非空 motor、input 和同源 now_us（微秒），输出执行状态及至多一个软件队列请求，无返回值。 */
void damiao_motor_update_command(damiao_motor_t *motor, const damiao_motor_input_t *input,
                                 uint64_t now_us);

/** 作用读取设备反馈、执行状态以及硬件接受和实际发送记录的快照。
 * 输入非空 motor 和同源 now_us（微秒），输出 damiao_motor_output_t 值拷贝。 */
damiao_motor_output_t damiao_motor_output(const damiao_motor_t *motor, uint64_t now_us);

/** 作用检查新鲜使能反馈并编码目标，截止时间收紧到反馈有效期后排队。
 * 输入非空 motor、command、now_us 和绝对 expires_us，输出 CAN 服务结果，QUEUED 仅表示软件入队。 */
can_service_result_t damiao_motor_command(damiao_motor_t *motor,
                                          const damiao_mit_command_t *command, uint64_t now_us,
                                          uint64_t expires_us);

/** 作用编码并排队设备动作，失能优先并撤销本实例旧运动请求，许可与确认由调用方管理。
 * 输入已初始化 motor、action 和绝对 expires_us，输出 CAN 服务结果，失能帧截止时间由服务忽略。 */
can_service_result_t damiao_motor_action(damiao_motor_t *motor, damiao_mit_action_t action,
                                         uint64_t expires_us);

/** 作用在调用方已确认失能后清除本实例尚未提交的失能重试帧。
 * 输入非空已初始化 motor，输出清理后的端点待发状态，无返回值。 */
void damiao_motor_confirm_disabled(damiao_motor_t *motor);

#endif /* FRAME_CDC_COMPONENTS_HARDWARE_DEVICES_DAMIAO_DAMIAO_MOTOR_H */
