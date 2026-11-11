/* 手写的台架场景接线，生产者各自持有输出。 */
#include "motor_bench.h"

#include "mc02_board.h"

/** 作用装配台架实例并按配置启动 CAN 和操作输入，实例地址须保持稳定。
 * 输入非空零初始化 bench、配置 config 和同源 now_us（微秒），输出装配后的 bench，无返回值。 */
void motor_bench_init(motor_bench_t *bench, const frame_config_t *config, uint64_t now_us)
{
    can_service_init(&bench->can, MC02_CAN_PORT_1);
    const bool is_registered = damiao_motor_init(&bench->motor, &bench->can, &config->motor);
    speed_controller_init(&bench->controller, &config->controller);
    const damiao_mit_config_t *motion = damiao_motor_motion_config(&bench->motor);
    const bool allow_motion =
        bench->controller.can_move && motion != NULL &&
        config->controller.trajectory.speed_limit_rad_s <= motion->speed_max_rad_s;
    const bool can_move =
        damiao_motor_configure_execution(&bench->motor, &config->execution, allow_motion,
                                         config->controller.control_period_us, now_us);
    const bool is_started =
        is_registered && can_service_start(&bench->can, config->can.bitrate_hz, now_us);
    bench->read_byte = is_started && can_move ? mc02_board_uart_read : NULL;
    if (bench->read_byte != NULL) {
        mc02_board_uart_enable_rx();
    }
}

/** 作用在 CAN 接收及 IMU/INS 更新后执行台架控制、设备命令和统一发送。
 * 输入非空 bench、姿态 pose 和同源 now_us（微秒），输出各实例状态与通信请求，无返回值。 */
void motor_bench_update(motor_bench_t *bench, const attitude_sample_t *pose, uint64_t now_us)
{
    damiao_motor_update_feedback(&bench->motor, now_us);
    const operator_command_t command =
        bench->read_byte != NULL
            ? operator_input_poll(&bench->operator_input, now_us, bench->read_byte)
            : OPERATOR_COMMAND_NONE;
    speed_controller_execution_t execution = SPEED_CONTROLLER_INACTIVE;
    if (bench->motor.output.state == DAMIAO_MOTOR_ARMING) {
        execution = SPEED_CONTROLLER_ENABLING;
    } else if (bench->motor.output.state == DAMIAO_MOTOR_ARMED) {
        execution = SPEED_CONTROLLER_ENABLED;
    }
    speed_controller_update(&bench->controller, pose, command, execution, now_us);
    const speed_controller_output_t *target = &bench->controller.output;
    const damiao_motor_input_t input = {
        .target_speed_rad_s = target->target_speed_rad_s,
        .target_expires_us = target->target_expires_us,
        .has_target = target->has_target,
        .should_enable = target->should_enable,
        .should_stop = target->should_stop,
    };
    damiao_motor_update_command(&bench->motor, &input, now_us);
    /* 软件排队、硬件接受、发送完成分别由服务记录，设备在后续周期消费。 */
    (void)can_service_poll_tx(&bench->can);
}
