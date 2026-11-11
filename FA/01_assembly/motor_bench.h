#ifndef FRAME_CDC_ASSEMBLY_MOTOR_BENCH_H
#define FRAME_CDC_ASSEMBLY_MOTOR_BENCH_H

#include "damiao_motor.h"
#include "frame_config.h"
#include "speed_controller.h"

typedef struct {
    can_service_t can;
    damiao_motor_t motor;
    speed_controller_t controller;
    operator_input_t operator_input;
    int (*read_byte)(void);
} motor_bench_t;

/** 作用装配台架实例并按配置启动 CAN 和操作输入，实例地址须保持稳定。
 * 输入非空零初始化 bench、配置 config 和同源 now_us（微秒），输出装配后的 bench，无返回值。 */
void motor_bench_init(motor_bench_t *bench, const frame_config_t *config, uint64_t now_us);

/** 作用在 CAN 接收及 IMU/INS 更新后执行台架控制、设备命令和统一发送。
 * 输入非空 bench、姿态 pose 和同源 now_us（微秒），输出各实例状态与通信请求，无返回值。 */
void motor_bench_update(motor_bench_t *bench, const attitude_sample_t *pose, uint64_t now_us);

#endif /* FRAME_CDC_ASSEMBLY_MOTOR_BENCH_H */
