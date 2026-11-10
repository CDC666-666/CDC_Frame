#ifndef FRAME_CDC_APP_SINGLE_MOTOR_CONTROL_SINGLE_MOTOR_CONTROL_H
#define FRAME_CDC_APP_SINGLE_MOTOR_CONTROL_SINGLE_MOTOR_CONTROL_H

#include "damiao_motor.h"
#include "ins.h"
#include "operator_input.h"

typedef enum {
    SINGLE_MOTOR_FEEDBACK = 0,
    SINGLE_MOTOR_FIXED_SPEED,
    SINGLE_MOTOR_PITCH_SPEED,
} single_motor_mode_t;

typedef enum {
    SINGLE_MOTOR_DISARMED = 0,
    SINGLE_MOTOR_STOPPING,
    SINGLE_MOTOR_ARMING,
    SINGLE_MOTOR_ARMED,
} single_motor_state_t;

typedef struct {
    single_motor_mode_t mode;
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
} single_motor_control_config_t;

typedef struct {
    damiao_motor_output_t motor;
    single_motor_state_t state;
    double pitch_rad;
    double target_speed_rad_s;
    bool is_pitch_valid;
} single_motor_control_output_t;

typedef struct {
    single_motor_control_config_t config;
    damiao_motor_t *motor;
    single_motor_control_output_t output;
    uint64_t next_tx_us;
    uint64_t request_us;
    uint64_t arm_deadline_us;
    uint64_t last_poll_us;
    uint32_t submission_sequence;
    bool can_move;
    bool is_running;
    bool has_request_submission;
    bool should_probe_stop;
} single_motor_control_t;

/** @brief 绑定已注册的电机、台架配置及板级微秒时钟。每实例独立。 */
void single_motor_control_init(single_motor_control_t *control, damiao_motor_t *motor,
                               const single_motor_control_config_t *config, uint64_t now_us);

/** @brief 消费最新反馈、姿态和操作指令，更新输出并排队本实例的 CAN 请求。 */
void single_motor_control_update(single_motor_control_t *control, const attitude_sample_t *pose,
                                 operator_command_t command, uint64_t now_us);

#endif /* FRAME_CDC_APP_SINGLE_MOTOR_CONTROL_SINGLE_MOTOR_CONTROL_H */
