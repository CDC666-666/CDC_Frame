#ifndef FRAME_CDC_COMPONENTS_CONTROLLER_SPEED_CONTROLLER_H
#define FRAME_CDC_COMPONENTS_CONTROLLER_SPEED_CONTROLLER_H

#include "ins.h"
#include "operator_input.h"
#include "speed_trajectory.h"

typedef enum {
    SPEED_CONTROLLER_FEEDBACK = 0,
    SPEED_CONTROLLER_FIXED_SPEED,
    SPEED_CONTROLLER_PITCH_SPEED,
} speed_controller_mode_t;

typedef enum {
    SPEED_CONTROLLER_INACTIVE = 0,
    SPEED_CONTROLLER_ENABLING,
    SPEED_CONTROLLER_ENABLED,
} speed_controller_execution_t;

typedef struct {
    speed_controller_mode_t mode;
    uint64_t control_period_us;
    uint64_t attitude_timeout_us;
    double fixed_speed_rad_s;
    pitch_speed_config_t pitch;
    speed_trajectory_config_t trajectory;
} speed_controller_config_t;

typedef struct {
    double pitch_rad;
    double target_speed_rad_s;
    uint64_t target_expires_us;
    bool is_pitch_valid;
    bool has_target; /* 仅在本轮发布新目标时置位。 */
    bool should_enable;
    bool should_stop;
} speed_controller_output_t;

typedef struct {
    speed_controller_config_t config;
    speed_controller_output_t output;
    speed_trajectory_t trajectory;
    uint64_t next_target_us;
    bool can_move;
    bool was_armed;
    bool is_running;
} speed_controller_t;

/** 作用初始化或重新配置已停止的控制组件，清除运行和轨迹历史。
 * 输入非空 control 和 config，输出复制配置、运动许可与清零状态的 control，无返回值。 */
void speed_controller_init(speed_controller_t *control, const speed_controller_config_t *config);

/** 作用消费操作和观测，调用轨迹算法并发布速度目标及启停请求。
 * 输入非空 control、pose、指令 command、确认状态 execution 和 now_us，输出 control.output 与内部状态，无返回值。 */
void speed_controller_update(speed_controller_t *control, const attitude_sample_t *pose,
                             operator_command_t command, speed_controller_execution_t execution,
                             uint64_t now_us);

#endif /* FRAME_CDC_COMPONENTS_CONTROLLER_SPEED_CONTROLLER_H */
