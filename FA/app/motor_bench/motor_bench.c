/* 单电机台架模式、速度目标和启停状态机。 */
#include "motor_bench.h"

#include <math.h>

#include "math_utils.h"

#define MOTOR_BENCH_STATUS_PERIOD_US 20000U
#define MOTOR_BENCH_DISABLE_RETRY_US 20000U

static bool is_motion_config_valid(const motor_bench_config_t *config,
                                   const damiao_mit_config_t *motion)
{
    const bool is_motion_mode =
        config->mode == MOTOR_BENCH_FIXED_SPEED || config->mode == MOTOR_BENCH_PITCH_SPEED;
    return is_motion_mode && motion != NULL && config->control_period_us > 0U &&
           config->control_period_us < motion->feedback_timeout_us &&
           config->enable_timeout_us > config->control_period_us &&
           config->attitude_timeout_us > 0U && config->speed_limit_rad_s > 0.0 &&
           config->speed_limit_rad_s <= motion->speed_max_rad_s &&
           fabs(config->fixed_speed_rad_s) <= config->speed_limit_rad_s &&
           isfinite(config->center_pitch_rad) && isfinite(config->pitch_deadband_rad) &&
           config->pitch_deadband_rad >= 0.0 && isfinite(config->pitch_to_speed_per_s) &&
           isfinite(config->accel_limit_rad_s2) && config->accel_limit_rad_s2 > 0.0 &&
           isfinite(config->decel_limit_rad_s2) && config->decel_limit_rad_s2 > 0.0 &&
           config->kd_n_m_s_per_rad >= 5.0 / 4095.0 && config->kd_n_m_s_per_rad <= 5.0;
}

void motor_bench_init(motor_bench_t *control, damiao_motor_t *motor,
                      const motor_bench_config_t *config, uint64_t now_us)
{
    *control = (motor_bench_t){
        .config = *config,
        .motor = motor,
        .last_poll_us = now_us,
    };
    const damiao_mit_config_t *motion = damiao_motor_motion_config(motor);
    control->can_move = is_motion_config_valid(config, motion);
    if (control->can_move) {
        control->feedback_timeout_us = motion->feedback_timeout_us;
    }
    control->output.state = control->can_move ? MOTOR_BENCH_STOPPING : MOTOR_BENCH_DISARMED;
}

operator_command_t motor_bench_poll_operator(motor_bench_t *bench, uint64_t now_us,
                                             int (*read_byte)(void))
{
    return read_byte != NULL ? operator_input_poll(&bench->operator_input, now_us, read_byte)
                             : OPERATOR_COMMAND_NONE;
}

static double desired_speed(const motor_bench_t *control)
{
    if (!control->is_running) {
        return 0.0;
    }
    const motor_bench_config_t *config = &control->config;
    if (config->mode == MOTOR_BENCH_FIXED_SPEED) {
        return config->fixed_speed_rad_s;
    }
    const double offset_rad = control->output.pitch_rad - config->center_pitch_rad;
    const double outside_rad =
        copysign(fmax(fabs(offset_rad) - config->pitch_deadband_rad, 0.0), offset_rad);
    return math_utils_clamp(outside_rad * config->pitch_to_speed_per_s, -config->speed_limit_rad_s,
                            config->speed_limit_rad_s);
}

static void update_target_speed(motor_bench_t *control)
{
    const motor_bench_config_t *config = &control->config;
    const double current_rad_s = control->output.target_speed_rad_s;
    double target_rad_s = desired_speed(control);
    /* 反向先按台架减速限制归零，再按加速限制启动。 */
    if (current_rad_s * target_rad_s < 0.0) {
        target_rad_s = 0.0;
    }
    const double rate_rad_s2 = fabs(target_rad_s) > fabs(current_rad_s)
                                   ? config->accel_limit_rad_s2
                                   : config->decel_limit_rad_s2;
    const double max_delta_rad_s = rate_rad_s2 * ((double)config->control_period_us * 1e-6);
    control->output.target_speed_rad_s =
        current_rad_s +
        math_utils_clamp(target_rad_s - current_rad_s, -max_delta_rad_s, max_delta_rad_s);
}

static void begin_stop(motor_bench_t *control, uint64_t now_us)
{
    control->output.state = MOTOR_BENCH_STOPPING;
    control->output.target_speed_rad_s = 0.0;
    control->is_running = false;
    control->has_request_submission = false;
    control->should_probe_stop = false;
    control->next_tx_us = now_us;
}

static void observe_submission(motor_bench_t *control)
{
    const can_service_submission_t *submission = &control->output.motor.submission;
    if (control->submission_sequence == submission->sequence) {
        return;
    }
    control->submission_sequence = submission->sequence;
    if (control->output.state == MOTOR_BENCH_ARMING && submission->kind == CAN_SERVICE_TX_ENABLE) {
        control->has_request_submission = true;
        control->request_us = submission->time_us;
        control->arm_deadline_us = submission->time_us + control->config.enable_timeout_us;
    } else if (control->output.state == MOTOR_BENCH_STOPPING &&
               submission->kind == CAN_SERVICE_TX_DISABLE) {
        if (!control->has_request_submission) {
            control->request_us = submission->time_us;
            control->has_request_submission = true;
        }
        control->should_probe_stop = control->config.has_status_request;
        control->next_tx_us =
            submission->time_us + (control->should_probe_stop ? MOTOR_BENCH_DISABLE_RETRY_US / 2U
                                                              : MOTOR_BENCH_DISABLE_RETRY_US);
    } else if (control->output.state == MOTOR_BENCH_STOPPING &&
               submission->kind == CAN_SERVICE_TX_QUERY && control->should_probe_stop) {
        control->should_probe_stop = false;
        control->next_tx_us = submission->time_us + MOTOR_BENCH_DISABLE_RETRY_US / 2U;
    }
}

static void poll_stop(motor_bench_t *control, uint64_t now_us)
{
    const damiao_motor_output_t *motor = &control->output.motor;
    if (motor->bus_status == MC02_CAN_OK && motor->status == DAMIAO_MIT_FEEDBACK_DISABLED &&
        control->has_request_submission && motor->feedback.time_us > control->request_us) {
        damiao_motor_confirm_disabled(control->motor);
        control->output.state = MOTOR_BENCH_DISARMED;
        return;
    }
    if (now_us < control->next_tx_us) {
        return;
    }
    if (control->should_probe_stop && motor->bus_status == MC02_CAN_OK) {
        (void)damiao_motor_action(control->motor, DAMIAO_MIT_READ_STATUS,
                                  now_us + MOTOR_BENCH_STATUS_PERIOD_US);
    } else {
        control->should_probe_stop = false;
        (void)damiao_motor_action(control->motor, DAMIAO_MIT_DISABLE, UINT64_MAX);
    }
}

static void poll_status_request(motor_bench_t *control, uint64_t now_us)
{
    if (control->config.has_status_request && control->output.motor.bus_status == MC02_CAN_OK &&
        now_us >= control->next_tx_us) {
        control->next_tx_us = now_us + MOTOR_BENCH_STATUS_PERIOD_US;
        (void)damiao_motor_action(control->motor, DAMIAO_MIT_READ_STATUS, control->next_tx_us);
    }
}

void motor_bench_update(motor_bench_t *control, const attitude_sample_t *pose,
                        operator_command_t command, uint64_t now_us)
{
    motor_bench_output_t *output = &control->output;
    const motor_bench_config_t *config = &control->config;
    const damiao_motor_output_t motor = damiao_motor_output(control->motor, now_us);
    const bool has_new_fault = motor.fault_count != output->motor.fault_count;
    const bool has_new_disable = motor.disabled_count != output->motor.disabled_count;
    const bool has_poll_gap = now_us - control->last_poll_us >= control->feedback_timeout_us;
    control->last_poll_us = now_us;
    output->motor = motor;
    output->pitch_rad = 0.0;
    output->is_pitch_valid = pose->is_sample_valid &&
                             now_us - pose->sample_time_us < config->attitude_timeout_us &&
                             attitude_get_pitch_rad(&pose->attitude, &output->pitch_rad);
    observe_submission(control);
    if (!control->can_move) {
        poll_status_request(control, now_us);
        return;
    }

    const bool is_active =
        output->state == MOTOR_BENCH_ARMED || output->state == MOTOR_BENCH_ARMING;
    const bool is_safe = motor.bus_status == MC02_CAN_OK && !has_poll_gap && !has_new_fault &&
                         (config->mode != MOTOR_BENCH_PITCH_SPEED || output->is_pitch_valid);
    if (command == OPERATOR_COMMAND_DISABLE ||
        (is_active && (!is_safe || motor.status >= DAMIAO_MIT_FEEDBACK_UNSEEN)) ||
        (output->state == MOTOR_BENCH_ARMED &&
         (motor.status != DAMIAO_MIT_FEEDBACK_ENABLED || has_new_disable)) ||
        (output->state == MOTOR_BENCH_DISARMED && motor.status == DAMIAO_MIT_FEEDBACK_ENABLED)) {
        begin_stop(control, now_us);
    }
    if (output->state == MOTOR_BENCH_STOPPING) {
        poll_stop(control, now_us);
        return;
    }
    if (output->state == MOTOR_BENCH_DISARMED) {
        const uint64_t deadline_us = now_us + config->enable_timeout_us;
        if (command == OPERATOR_COMMAND_ARM && is_safe &&
            motor.status == DAMIAO_MIT_FEEDBACK_DISABLED &&
            damiao_motor_action(control->motor, DAMIAO_MIT_ENABLE, deadline_us) ==
                CAN_SERVICE_QUEUED) {
            control->arm_deadline_us = deadline_us;
            control->has_request_submission = false;
            control->next_tx_us = now_us + config->control_period_us;
            output->state = MOTOR_BENCH_ARMING;
        } else {
            poll_status_request(control, now_us);
        }
        return;
    }
    if (output->state == MOTOR_BENCH_ARMING) {
        if (now_us >= control->arm_deadline_us) {
            begin_stop(control, now_us);
            poll_stop(control, now_us);
            return;
        }
        if (!control->has_request_submission || motor.status != DAMIAO_MIT_FEEDBACK_ENABLED ||
            motor.feedback.time_us <= control->request_us) {
            if (control->has_request_submission) {
                poll_status_request(control, now_us);
            }
            return;
        }
        output->state = MOTOR_BENCH_ARMED;
        control->next_tx_us = now_us;
        command = OPERATOR_COMMAND_NONE;
    }
    if (command == OPERATOR_COMMAND_RUN) {
        control->is_running = true;
    } else if (command == OPERATOR_COMMAND_ZERO) {
        control->is_running = false;
    }
    if (now_us < control->next_tx_us) {
        return;
    }

    /* 延误时每轮最多推进一个控制周期的变化量。 */
    control->next_tx_us = now_us + config->control_period_us;
    update_target_speed(control);
    const damiao_mit_command_t mit = {
        .speed_rad_s = output->target_speed_rad_s,
        .kd_n_m_s_per_rad = config->kd_n_m_s_per_rad,
    };
    if (damiao_motor_command(control->motor, &mit, now_us, control->next_tx_us) !=
        CAN_SERVICE_QUEUED) {
        begin_stop(control, now_us);
        poll_stop(control, now_us);
    }
}
