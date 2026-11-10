/* 把每颗 MIT 电机的反馈和发送接到共享 CAN 服务。 */
#include "damiao_motor.h"

static void receive_feedback(void *context, const can_frame_t *frame)
{
    damiao_motor_t *motor = context;
    if (!damiao_mit_accept_feedback(&motor->mit, frame)) {
        return;
    }
    const uint8_t state = motor->mit.feedback.state_raw;
    if (state == 0U) {
        ++motor->disabled_count;
    } else if (state != 1U) {
        ++motor->fault_count;
    }
    if (state != 1U) {
        can_service_cancel_motion(motor->endpoint);
    }
}

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

const damiao_mit_config_t *damiao_motor_motion_config(const damiao_motor_t *motor)
{
    return motor->endpoint != NULL && motor->mit.has_valid_ranges ? &motor->mit.config : NULL;
}

damiao_motor_output_t damiao_motor_output(const damiao_motor_t *motor, uint64_t now_us)
{
    const can_service_endpoint_t *endpoint = motor->endpoint;
    return (damiao_motor_output_t){
        .feedback = motor->mit.feedback,
        .status = damiao_mit_status(&motor->mit, now_us),
        .bus_status = endpoint != NULL ? endpoint->bus->status : MC02_CAN_NOT_READY,
        .submission = endpoint != NULL ? endpoint->last_submission : (can_service_submission_t){0},
        .transmission =
            endpoint != NULL ? endpoint->last_transmission : (can_service_transmission_t){0},
        .fault_count = motor->fault_count,
        .disabled_count = motor->disabled_count,
        .has_scaled_feedback = motor->mit.feedback.has_feedback && motor->mit.has_valid_ranges,
    };
}

can_service_result_t damiao_motor_command(damiao_motor_t *motor,
                                          const damiao_mit_command_t *command, uint64_t now_us,
                                          uint64_t expires_us)
{
    if (motor->endpoint == NULL || now_us >= expires_us ||
        damiao_mit_status(&motor->mit, now_us) != DAMIAO_MIT_FEEDBACK_ENABLED) {
        return CAN_SERVICE_INVALID;
    }
    can_frame_t frame;
    if (!damiao_mit_encode_command(&motor->mit, command, &frame)) {
        return CAN_SERVICE_INVALID;
    }
    const uint64_t feedback_expires_us =
        motor->mit.feedback.time_us + motor->mit.config.feedback_timeout_us;
    if (expires_us > feedback_expires_us) {
        expires_us = feedback_expires_us;
    }
    return can_service_queue(motor->endpoint, CAN_SERVICE_TX_MOTION, &frame, expires_us);
}

can_service_result_t damiao_motor_action(damiao_motor_t *motor, damiao_mit_action_t action,
                                         uint64_t expires_us)
{
    can_frame_t frame;
    if (!damiao_mit_encode_action(&motor->mit, action, &frame)) {
        return CAN_SERVICE_INVALID;
    }
    can_service_tx_kind_t kind = CAN_SERVICE_TX_QUERY;
    if (action == DAMIAO_MIT_DISABLE) {
        kind = CAN_SERVICE_TX_DISABLE;
    } else if (action == DAMIAO_MIT_ENABLE) {
        kind = CAN_SERVICE_TX_ENABLE;
    }
    return can_service_queue(motor->endpoint, kind, &frame, expires_us);
}

void damiao_motor_confirm_disabled(damiao_motor_t *motor)
{
    can_service_clear_queued(motor->endpoint, CAN_SERVICE_TX_DISABLE);
}
