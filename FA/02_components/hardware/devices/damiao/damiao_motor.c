/* 把每颗 MIT 电机的反馈和发送接到共享 CAN 服务。 */
#include "damiao_motor.h"

/** 作用根据反馈存在性、新鲜度和设备状态确定当前反馈状态。
 * 输入非空 motor 和同源 now_us（微秒），输出 damiao_mit_status_t 状态枚举。 */
static damiao_mit_status_t feedback_status(const damiao_motor_t *motor, uint64_t now_us)
{
    if (!motor->feedback.has_feedback) {
        return DAMIAO_MIT_FEEDBACK_UNSEEN;
    }
    if (now_us - motor->feedback.time_us >= motor->mit.config.feedback_timeout_us) {
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

/** 作用解码本实例反馈，拒绝倒序时间并在失能或故障时撤销运动帧。
 * 输入有效设备 context 和非空 frame，输出设备反馈、计数及本端点撤销请求，无返回值。 */
static void receive_feedback(void *context, const can_frame_t *frame)
{
    damiao_motor_t *motor = context;
    if ((motor->feedback.has_feedback && frame->rx_time_us < motor->feedback.time_us) ||
        !damiao_mit_decode_feedback(&motor->mit, frame, &motor->feedback)) {
        return;
    }
    const uint8_t state = motor->feedback.state_raw;
    if (state == 0U) {
        ++motor->disabled_count;
    } else if (state != 1U) {
        ++motor->fault_count;
    }
    if (state != 1U) {
        can_service_cancel(motor->endpoint, CAN_SERVICE_TX_NORMAL);
    }
}

/** 作用初始化软件设备并注册独立反馈端点，主循环持有的实例和总线须地址稳定。
 * 输入非空零初始化 motor、未启动 bus 和 config，输出初始化实例，成功返回 true，重复或注册失败返回 false。 */
bool damiao_motor_init(damiao_motor_t *motor, can_service_t *bus, const damiao_mit_config_t *config)
{
    damiao_mit_t mit;
    if (motor->endpoint != NULL || !damiao_mit_init(&mit, config)) {
        return false;
    }
    can_service_endpoint_t *endpoint =
        can_service_register(bus, config->master_id, receive_feedback, motor);
    if (endpoint == NULL) {
        return false;
    }
    *motor = (damiao_motor_t){.mit = mit, .endpoint = endpoint};
    return true;
}

/** 作用取得已注册且量程有效的电机配置供调用方只读使用。
 * 输入非空 motor，输出实例拥有的配置指针，量程未知或未初始化返回 NULL，指针寿命同实例。 */
const damiao_mit_config_t *damiao_motor_motion_config(const damiao_motor_t *motor)
{
    return motor->endpoint != NULL && motor->mit.has_valid_ranges ? &motor->mit.config : NULL;
}

/** 作用读取设备反馈、执行状态以及硬件接受和实际发送记录的快照。
 * 输入非空 motor 和同源 now_us（微秒），输出 damiao_motor_output_t 值拷贝。 */
damiao_motor_output_t damiao_motor_output(const damiao_motor_t *motor, uint64_t now_us)
{
    const can_service_endpoint_t *endpoint = motor->endpoint;
    return (damiao_motor_output_t){
        .feedback = motor->feedback,
        .status = feedback_status(motor, now_us),
        .bus_status = endpoint != NULL ? endpoint->bus->status : MC02_CAN_NOT_READY,
        .submission = endpoint != NULL ? endpoint->last_submission : (can_service_submission_t){0},
        .transmission =
            endpoint != NULL ? endpoint->last_transmission : (can_service_transmission_t){0},
        .state = motor->output.state,
        .fault_count = motor->fault_count,
        .disabled_count = motor->disabled_count,
        .has_scaled_feedback = motor->feedback.has_feedback && motor->mit.has_valid_ranges,
    };
}

/** 作用检查新鲜使能反馈并编码目标，截止时间收紧到反馈有效期后排队。
 * 输入非空 motor、command、now_us 和绝对 expires_us，输出 CAN 服务结果，QUEUED 仅表示软件入队。 */
can_service_result_t damiao_motor_command(damiao_motor_t *motor,
                                          const damiao_mit_command_t *command, uint64_t now_us,
                                          uint64_t expires_us)
{
    if (motor->endpoint == NULL || now_us >= expires_us ||
        feedback_status(motor, now_us) != DAMIAO_MIT_FEEDBACK_ENABLED) {
        return CAN_SERVICE_INVALID;
    }
    can_frame_t frame;
    if (!damiao_mit_encode_command(&motor->mit, command, &frame)) {
        return CAN_SERVICE_INVALID;
    }
    const uint64_t feedback_expires_us =
        motor->feedback.time_us + motor->mit.config.feedback_timeout_us;
    if (expires_us > feedback_expires_us) {
        expires_us = feedback_expires_us;
    }
    return can_service_queue(motor->endpoint, CAN_SERVICE_TX_NORMAL, &frame, expires_us);
}

/** 作用编码并排队设备动作，失能优先并撤销本实例旧运动请求，许可与确认由调用方管理。
 * 输入已初始化 motor、action 和绝对 expires_us，输出 CAN 服务结果，失能帧截止时间由服务忽略。 */
can_service_result_t damiao_motor_action(damiao_motor_t *motor, damiao_mit_action_t action,
                                         uint64_t expires_us)
{
    can_frame_t frame;
    if (!damiao_mit_encode_action(&motor->mit, action, &frame)) {
        return CAN_SERVICE_INVALID;
    }
    can_service_tx_kind_t kind = CAN_SERVICE_TX_BACKGROUND;
    if (action == DAMIAO_MIT_DISABLE) {
        kind = CAN_SERVICE_TX_URGENT;
    } else if (action == DAMIAO_MIT_ENABLE) {
        kind = CAN_SERVICE_TX_HIGH;
    }
    return can_service_queue(motor->endpoint, kind, &frame, expires_us);
}

/** 作用在调用方已确认失能后清除本实例尚未提交的失能重试帧。
 * 输入非空已初始化 motor，输出清理后的端点待发状态，无返回值。 */
void damiao_motor_confirm_disabled(damiao_motor_t *motor)
{
    can_service_clear_queued(motor->endpoint, CAN_SERVICE_TX_URGENT);
}

/** 作用在启动阶段配置速度执行保护并建立初始确认状态。
 * 输入非空 motor、config、allow_motion、command_period_us 和 now_us，输出执行状态，满足运动条件返回 true。 */
bool damiao_motor_configure_execution(damiao_motor_t *motor,
                                      const damiao_motor_execution_config_t *config,
                                      bool allow_motion, uint64_t command_period_us,
                                      uint64_t now_us)
{
    motor->execution = *config;
    motor->last_poll_us = now_us;
    motor->arm_status_delay_us = command_period_us;
    motor->can_move = allow_motion && damiao_motor_motion_config(motor) != NULL &&
                      command_period_us > 0U &&
                      command_period_us < motor->mit.config.feedback_timeout_us &&
                      config->enable_timeout_us > command_period_us &&
                      config->disable_retry_us >= 2U && config->status_period_us > 0U &&
                      config->kd_n_m_s_per_rad >= DAMIAO_MIT_KD_MAX / DAMIAO_MIT_MASK_12 &&
                      config->kd_n_m_s_per_rad <= DAMIAO_MIT_KD_MAX;
    motor->output.state = motor->can_move ? DAMIAO_MOTOR_STOPPING : DAMIAO_MOTOR_DISARMED;
    return motor->can_move;
}

/** 作用进入停止确认并清除旧目标有效期及请求关联，安排立即处理失能。
 * 输入非空 motor 和同源 now_us（微秒），输出 STOPPING 状态与重试时刻，无返回值。 */
static void begin_stop(damiao_motor_t *motor, uint64_t now_us)
{
    motor->output.state = DAMIAO_MOTOR_STOPPING;
    motor->target_expires_us = 0U;
    motor->has_request_submission = false;
    motor->should_probe_stop = false;
    motor->next_action_us = now_us;
}

/** 作用消费新的硬件接受序号并关联使能确认截止或失能查询重试时序。
 * 输入非空 motor 及其 output.submission，输出请求标志、关联时间和下次动作时刻，无返回值。 */
static void observe_submission(damiao_motor_t *motor)
{
    const can_service_submission_t *submission = &motor->output.submission;
    if (motor->submission_sequence == submission->sequence) {
        return;
    }
    motor->submission_sequence = submission->sequence;
    const damiao_motor_execution_config_t *config = &motor->execution;
    if (motor->output.state == DAMIAO_MOTOR_ARMING && submission->kind == CAN_SERVICE_TX_HIGH) {
        motor->has_request_submission = true;
        motor->request_us = submission->time_us;
        motor->arm_deadline_us = submission->time_us + config->enable_timeout_us;
    } else if (motor->output.state == DAMIAO_MOTOR_STOPPING) {
        if (submission->kind == CAN_SERVICE_TX_URGENT) {
            if (!motor->has_request_submission) {
                motor->request_us = submission->time_us;
                motor->has_request_submission = true;
            }
            motor->should_probe_stop = config->has_status_request;
            motor->next_action_us =
                submission->time_us + (motor->should_probe_stop ? config->disable_retry_us / 2U
                                                                : config->disable_retry_us);
        } else if (submission->kind == CAN_SERVICE_TX_BACKGROUND && motor->should_probe_stop) {
            motor->should_probe_stop = false;
            motor->next_action_us = submission->time_us + config->disable_retry_us / 2U;
        }
    }
}

/** 作用发布反馈快照并处理故障、轮询间断、使能确认和确认超时。
 * 输入非空 motor 和同源 now_us（微秒），输出 motor.output 及执行保护状态，无返回值。 */
void damiao_motor_update_feedback(damiao_motor_t *motor, uint64_t now_us)
{
    const damiao_motor_output_t feedback = damiao_motor_output(motor, now_us);
    const bool has_new_fault = feedback.fault_count != motor->output.fault_count;
    const bool has_new_disable = feedback.disabled_count != motor->output.disabled_count;
    const bool has_poll_gap = now_us - motor->last_poll_us >= motor->mit.config.feedback_timeout_us;
    motor->last_poll_us = now_us;
    motor->output = feedback;
    observe_submission(motor);
    if (!motor->can_move) {
        return;
    }

    motor->is_arm_safe = feedback.bus_status == MC02_CAN_OK && !has_poll_gap && !has_new_fault;
    const damiao_motor_state_t state = feedback.state;
    const bool is_active = state == DAMIAO_MOTOR_ARMED || state == DAMIAO_MOTOR_ARMING;
    if ((is_active && (!motor->is_arm_safe || feedback.status >= DAMIAO_MIT_FEEDBACK_UNSEEN)) ||
        (state == DAMIAO_MOTOR_ARMED &&
         (feedback.status != DAMIAO_MIT_FEEDBACK_ENABLED || has_new_disable)) ||
        (state == DAMIAO_MOTOR_DISARMED && feedback.status == DAMIAO_MIT_FEEDBACK_ENABLED)) {
        begin_stop(motor, now_us);
    } else if (state == DAMIAO_MOTOR_ARMING) {
        if (now_us >= motor->arm_deadline_us) {
            begin_stop(motor, now_us);
        } else if (motor->has_request_submission &&
                   feedback.status == DAMIAO_MIT_FEEDBACK_ENABLED &&
                   feedback.feedback.time_us > motor->request_us) {
            motor->output.state = DAMIAO_MOTOR_ARMED;
        }
    }
}

/** 作用检查新鲜失能确认，或到期排队下一次失能和状态查询请求。
 * 输入非空 motor 和同源 now_us（微秒），输出失能确认状态或软件队列请求，无返回值。 */
static void poll_stop(damiao_motor_t *motor, uint64_t now_us)
{
    const damiao_motor_output_t *output = &motor->output;
    if (output->bus_status == MC02_CAN_OK && output->status == DAMIAO_MIT_FEEDBACK_DISABLED &&
        motor->has_request_submission && output->feedback.time_us > motor->request_us) {
        damiao_motor_confirm_disabled(motor);
        motor->output.state = DAMIAO_MOTOR_DISARMED;
        return;
    }
    if (now_us < motor->next_action_us) {
        return;
    }
    if (motor->should_probe_stop && output->bus_status == MC02_CAN_OK) {
        /* 下轮根据硬件实际提交时刻更新重试时间。 */
        (void)damiao_motor_action(motor, DAMIAO_MIT_READ_STATUS,
                                  now_us + motor->execution.status_period_us);
    } else {
        motor->should_probe_stop = false;
        (void)damiao_motor_action(motor, DAMIAO_MIT_DISABLE, UINT64_MAX);
    }
}

/** 作用在总线健康且查询周期到期时排队只读状态请求。
 * 输入非空 motor 和同源 now_us（微秒），输出下次查询时刻及可能的队列请求，无返回值。 */
static void poll_status_request(damiao_motor_t *motor, uint64_t now_us)
{
    if (motor->execution.has_status_request && motor->execution.status_period_us > 0U &&
        motor->output.bus_status == MC02_CAN_OK && now_us >= motor->next_action_us) {
        motor->next_action_us = now_us + motor->execution.status_period_us;
        (void)damiao_motor_action(motor, DAMIAO_MIT_READ_STATUS, motor->next_action_us);
    }
}

/** 作用在反馈阶段之后消费启停与有时效目标，推进设备保护并准备 CAN 统一发送。
 * 输入非空 motor、input 和同源 now_us（微秒），输出执行状态及至多一个软件队列请求，无返回值。 */
void damiao_motor_update_command(damiao_motor_t *motor, const damiao_motor_input_t *input,
                                 uint64_t now_us)
{
    if (!motor->can_move) {
        poll_status_request(motor, now_us);
        return;
    }
    if (input->should_stop) {
        begin_stop(motor, now_us);
    }
    switch (motor->output.state) {
        case DAMIAO_MOTOR_STOPPING:
            poll_stop(motor, now_us);
            break;
        case DAMIAO_MOTOR_DISARMED: {
            const uint64_t deadline_us = now_us + motor->execution.enable_timeout_us;
            if (input->should_enable && motor->is_arm_safe &&
                motor->output.status == DAMIAO_MIT_FEEDBACK_DISABLED &&
                damiao_motor_action(motor, DAMIAO_MIT_ENABLE, deadline_us) == CAN_SERVICE_QUEUED) {
                motor->arm_deadline_us = deadline_us;
                motor->target_expires_us = 0U;
                motor->has_request_submission = false;
                motor->next_action_us = now_us + motor->arm_status_delay_us;
                motor->output.state = DAMIAO_MOTOR_ARMING;
            } else {
                poll_status_request(motor, now_us);
            }
            break;
        }
        case DAMIAO_MOTOR_ARMING:
            if (motor->has_request_submission) {
                poll_status_request(motor, now_us);
            }
            break;
        case DAMIAO_MOTOR_ARMED:
            if (!input->has_target && now_us >= motor->target_expires_us) {
                begin_stop(motor, now_us);
                poll_stop(motor, now_us);
                break;
            }
            if (input->has_target) {
                motor->target_expires_us = input->target_expires_us;
                const damiao_mit_command_t mit = {
                    .speed_rad_s = input->target_speed_rad_s,
                    .kd_n_m_s_per_rad = motor->execution.kd_n_m_s_per_rad,
                };
                if (damiao_motor_command(motor, &mit, now_us, input->target_expires_us) !=
                    CAN_SERVICE_QUEUED) {
                    begin_stop(motor, now_us);
                    poll_stop(motor, now_us);
                }
            }
            break;
    }
}
