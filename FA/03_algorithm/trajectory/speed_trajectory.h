#ifndef FRAME_CDC_ALGORITHM_TRAJECTORY_SPEED_TRAJECTORY_H
#define FRAME_CDC_ALGORITHM_TRAJECTORY_SPEED_TRAJECTORY_H

typedef struct {
    double center_pitch_rad;
    double pitch_deadband_rad;
    double pitch_to_speed_per_s;
} pitch_speed_config_t;

typedef struct {
    double speed_limit_rad_s;
    double accel_limit_rad_s2;
    double decel_limit_rad_s2;
} speed_trajectory_config_t;

typedef struct {
    double speed_rad_s;
} speed_trajectory_t;

/** 作用将已验证的俯仰角按中心偏置、死区和比例映射为有符号速度。
 * 输入非空 config 和 pitch_rad（rad），输出未限幅的目标速度（rad/s）。 */
double pitch_speed_target(const pitch_speed_config_t *config, double pitch_rad);

/** 作用统一限幅与加减速推进，反向先降至零，零初始化状态表示静止。
 * 输入非空 state/config、正有限配置限值、有限 target_rad_s 及非负 dt_s，输出并保存本实例速度（rad/s）。 */
double speed_trajectory_update(speed_trajectory_t *state, const speed_trajectory_config_t *config,
                               double target_rad_s, double dt_s);

#endif /* FRAME_CDC_ALGORITHM_TRAJECTORY_SPEED_TRAJECTORY_H */
