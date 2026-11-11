#include "speed_controller.h"

#include <math.h>

/** 作用初始化或重新配置已停止的控制组件，清除运行和轨迹历史。
 * 输入非空 control 和 config，输出复制配置、运动许可与清零状态的 control，无返回值。 */
void speed_controller_init(speed_controller_t *control, const speed_controller_config_t *config)
{
    const pitch_speed_config_t *pitch = &config->pitch;
    const speed_trajectory_config_t *trajectory = &config->trajectory;
    const double dt_s = (double)config->control_period_us * 1e-6;
    *control = (speed_controller_t){.config = *config};
    control->can_move =
        (config->mode == SPEED_CONTROLLER_FIXED_SPEED ||
         config->mode == SPEED_CONTROLLER_PITCH_SPEED) &&
        config->control_period_us > 0U && config->attitude_timeout_us > 0U &&
        isfinite(pitch->center_pitch_rad) && isfinite(pitch->pitch_deadband_rad) &&
        pitch->pitch_deadband_rad >= 0.0 && isfinite(pitch->pitch_to_speed_per_s) &&
        isfinite(trajectory->speed_limit_rad_s) && trajectory->speed_limit_rad_s > 0.0 &&
        fabs(config->fixed_speed_rad_s) <= trajectory->speed_limit_rad_s &&
        isfinite(trajectory->accel_limit_rad_s2) && trajectory->accel_limit_rad_s2 > 0.0 &&
        isfinite(trajectory->decel_limit_rad_s2) && trajectory->decel_limit_rad_s2 > 0.0 &&
        isfinite(trajectory->accel_limit_rad_s2 * dt_s) &&
        isfinite(trajectory->decel_limit_rad_s2 * dt_s);
}

/** 作用消费操作和观测，调用轨迹算法并发布速度目标及启停请求。
 * 输入非空 control、pose、指令 command、确认状态 execution 和 now_us，输出 control.output 与内部状态，无返回值。 */
void speed_controller_update(speed_controller_t *control, const attitude_sample_t *pose,
                             operator_command_t command, speed_controller_execution_t execution,
                             uint64_t now_us)
{
    const speed_controller_config_t *config = &control->config;
    speed_controller_output_t *output = &control->output;
    output->pitch_rad = 0.0;
    output->is_pitch_valid = pose->is_sample_valid &&
                             now_us - pose->sample_time_us < config->attitude_timeout_us &&
                             attitude_get_pitch_rad(&pose->attitude, &output->pitch_rad);
    const bool has_required_observation =
        config->mode != SPEED_CONTROLLER_PITCH_SPEED || output->is_pitch_valid;
    const bool is_armed = execution == SPEED_CONTROLLER_ENABLED;
    output->has_target = false;
    output->should_enable =
        control->can_move && has_required_observation && command == OPERATOR_COMMAND_ARM;
    output->should_stop = control->can_move &&
                          (command == OPERATOR_COMMAND_DISABLE ||
                           (execution != SPEED_CONTROLLER_INACTIVE && !has_required_observation));

    if (!is_armed || !control->was_armed || output->should_stop) {
        control->trajectory = (speed_trajectory_t){0};
        control->is_running = false;
        output->target_speed_rad_s = 0.0;
        control->next_target_us = now_us;
        /* 使能确认当轮只发送零目标，不接受 RUN。 */
        command = OPERATOR_COMMAND_NONE;
    }
    control->was_armed = is_armed && !output->should_stop;
    if (!control->can_move || !control->was_armed) {
        return;
    }

    if (command == OPERATOR_COMMAND_RUN) {
        control->is_running = true;
    } else if (command == OPERATOR_COMMAND_ZERO) {
        control->is_running = false;
    }
    if (now_us < control->next_target_us) {
        return;
    }

    double target_rad_s = 0.0;
    if (control->is_running) {
        target_rad_s = config->mode == SPEED_CONTROLLER_FIXED_SPEED
                           ? config->fixed_speed_rad_s
                           : pitch_speed_target(&config->pitch, output->pitch_rad);
    }
    /* 延误时每轮最多推进一个控制周期的变化量。 */
    const double dt_s = (double)config->control_period_us * 1e-6;
    output->target_speed_rad_s =
        speed_trajectory_update(&control->trajectory, &config->trajectory, target_rad_s, dt_s);
    control->next_target_us = now_us + config->control_period_us;
    output->target_expires_us = control->next_target_us;
    output->has_target = true;
}
