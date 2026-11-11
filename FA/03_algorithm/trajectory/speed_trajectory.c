#include "speed_trajectory.h"

#include <math.h>

#include "math_utils.h"

/** 作用将已验证的俯仰角按中心偏置、死区和比例映射为有符号速度。
 * 输入非空 config 和 pitch_rad（rad），输出未限幅的目标速度（rad/s）。 */
double pitch_speed_target(const pitch_speed_config_t *config, double pitch_rad)
{
    const double offset_rad = pitch_rad - config->center_pitch_rad;
    const double outside_rad =
        copysign(fmax(fabs(offset_rad) - config->pitch_deadband_rad, 0.0), offset_rad);
    return outside_rad * config->pitch_to_speed_per_s;
}

/** 作用统一限幅与加减速推进，反向先降至零，零初始化状态表示静止。
 * 输入非空 state/config、正有限配置限值、有限 target_rad_s 及非负 dt_s，输出并保存本实例速度（rad/s）。 */
double speed_trajectory_update(speed_trajectory_t *state, const speed_trajectory_config_t *config,
                               double target_rad_s, double dt_s)
{
    const double current_rad_s = state->speed_rad_s;
    target_rad_s =
        math_utils_clamp(target_rad_s, -config->speed_limit_rad_s, config->speed_limit_rad_s);
    if (current_rad_s * target_rad_s < 0.0) {
        target_rad_s = 0.0;
    }
    const double rate_rad_s2 = fabs(target_rad_s) > fabs(current_rad_s)
                                   ? config->accel_limit_rad_s2
                                   : config->decel_limit_rad_s2;
    const double max_delta_rad_s = rate_rad_s2 * dt_s;
    state->speed_rad_s = current_rad_s + math_utils_clamp(target_rad_s - current_rad_s,
                                                          -max_delta_rad_s, max_delta_rad_s);
    return state->speed_rad_s;
}
