#ifndef FRAME_CDC_APP_MOTOR_BENCH_MOTOR_BENCH_H
#define FRAME_CDC_APP_MOTOR_BENCH_MOTOR_BENCH_H

#include "damiao_motor.h"
#include "ins.h"
#include "operator_input.h"

typedef enum {
    MOTOR_BENCH_FEEDBACK = 0,
    MOTOR_BENCH_FIXED_SPEED,
    MOTOR_BENCH_PITCH_SPEED,
} motor_bench_mode_t;

typedef enum {
    MOTOR_BENCH_DISARMED = 0,
    MOTOR_BENCH_STOPPING,
    MOTOR_BENCH_ARMING,
    MOTOR_BENCH_ARMED,
} motor_bench_state_t;

typedef struct {
    motor_bench_mode_t mode;
    bool has_status_request;
    uint64_t control_period_us;
    uint64_t attitude_timeout_us;
    uint64_t enable_timeout_us;
    double center_pitch_rad;
    double pitch_deadband_rad;
    double pitch_to_speed_per_s;
    double fixed_speed_rad_s;
    double speed_limit_rad_s;
    double accel_limit_rad_s2;
    double decel_limit_rad_s2;
    double kd_n_m_s_per_rad;
} motor_bench_config_t;

typedef struct {
    damiao_motor_output_t motor;
    motor_bench_state_t state;
    double pitch_rad;
    double target_speed_rad_s;
    bool is_pitch_valid;
} motor_bench_output_t;

typedef struct {
    motor_bench_config_t config;
    damiao_motor_t *motor;
    motor_bench_output_t output;
    operator_input_t operator_input;
    uint64_t feedback_timeout_us;
    uint64_t next_tx_us;
    uint64_t request_us;
    uint64_t arm_deadline_us;
    uint64_t last_poll_us;
    uint32_t submission_sequence;
    bool can_move;
    bool is_running;
    bool has_request_submission;
    bool should_probe_stop;
} motor_bench_t;

/** @brief 绑定已注册的电机、台架配置及板级微秒时钟。每实例独立。 */
void motor_bench_init(motor_bench_t *control, damiao_motor_t *motor,
                      const motor_bench_config_t *config, uint64_t now_us);

/** @brief 从已绑定的字节源轮询台架指令。read_byte 为 NULL 时无指令。 */
operator_command_t motor_bench_poll_operator(motor_bench_t *bench, uint64_t now_us,
                                             int (*read_byte)(void));

/** @brief 消费最新反馈、姿态和操作指令，更新输出并排队本实例的 CAN 请求。 */
void motor_bench_update(motor_bench_t *control, const attitude_sample_t *pose,
                        operator_command_t command, uint64_t now_us);

#endif /* FRAME_CDC_APP_MOTOR_BENCH_MOTOR_BENCH_H */
