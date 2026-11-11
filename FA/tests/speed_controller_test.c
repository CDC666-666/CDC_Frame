#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "speed_controller.h"

/** 作用验证独立控制组件的目标生成、复位、延误步长和观测超时。
 * 输入无参数，使用局部控制实例及合成姿态，输出断言和通过信息，成功返回 0。 */
int main(void)
{
    speed_controller_t first;
    speed_controller_t second;
    speed_controller_config_t config = {
        .mode = SPEED_CONTROLLER_FIXED_SPEED,
        .control_period_us = 10000U,
        .attitude_timeout_us = 25000U,
        .fixed_speed_rad_s = 0.5,
        .pitch = {.pitch_deadband_rad = 0.05, .pitch_to_speed_per_s = 4.0},
        .trajectory = {1.0, 1.0, 2.0},
    };
    const attitude_sample_t pose = {
        .attitude = {.q_odom_from_body = {1.0},
                     .is_initialized = true,
                     .status = ATTITUDE_STATUS_OK},
        .sample_time_us = 0U,
        .is_sample_valid = true,
    };
    speed_controller_init(&first, &config);
    speed_controller_init(&second, &config);
    assert(first.can_move && second.can_move);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_ARM, SPEED_CONTROLLER_INACTIVE, 0U);
    assert(first.output.should_enable && !first.output.has_target);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_RUN, SPEED_CONTROLLER_ENABLED, 1000U);
    assert(first.output.has_target && first.output.target_speed_rad_s == 0.0);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_RUN, SPEED_CONTROLLER_ENABLED, 11000U);
    assert(first.output.target_speed_rad_s == 0.01);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_NONE, SPEED_CONTROLLER_ENABLED, 41000U);
    assert(first.output.target_speed_rad_s == 0.02); /* 延误只推进一个配置周期。 */
    assert(second.output.target_speed_rad_s == 0.0);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_ZERO, SPEED_CONTROLLER_ENABLED, 51000U);
    assert(first.output.target_speed_rad_s == 0.0);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_DISABLE, SPEED_CONTROLLER_ENABLED,
                            52000U);
    assert(first.output.should_stop && !first.output.has_target);
    config.mode = SPEED_CONTROLLER_PITCH_SPEED;
    speed_controller_init(&first, &config);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_ARM, SPEED_CONTROLLER_INACTIVE, 25000U);
    assert(!first.output.should_enable && !first.output.should_stop);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_NONE, SPEED_CONTROLLER_ENABLING,
                            25000U);
    assert(first.output.should_stop);
    config.mode = SPEED_CONTROLLER_FEEDBACK;
    speed_controller_init(&first, &config);
    speed_controller_update(&first, &pose, OPERATOR_COMMAND_ARM, SPEED_CONTROLLER_INACTIVE, 0U);
    assert(!first.output.should_enable && !first.output.has_target);
    config.mode = SPEED_CONTROLLER_FIXED_SPEED;
    config.trajectory.accel_limit_rad_s2 = NAN;
    speed_controller_init(&first, &config);
    assert(!first.can_move);
    puts("Controller target generation passed without CAN or Damiao linkage");
    return 0;
}
