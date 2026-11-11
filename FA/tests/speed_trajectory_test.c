#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "speed_trajectory.h"

/** 作用验证独立倾角映射、死区、限幅、反向归零和轨迹实例隔离。
 * 输入无参数，使用局部数值配置和状态，输出断言和通过信息，成功返回 0。 */
int main(void)
{
    const pitch_speed_config_t pitch = {0.1, 0.05, 4.0};
    const speed_trajectory_config_t config = {1.0, 1.0, 2.0};
    speed_trajectory_t first = {0};
    speed_trajectory_t second = {0};
    assert(fabs(pitch_speed_target(&pitch, 0.3) - 0.6) < 1e-12);
    assert(pitch_speed_target(&pitch, 0.12) == 0.0);
    for (size_t i = 0U; i < 200U; ++i) {
        (void)speed_trajectory_update(&first, &config, 100.0, 0.01);
    }
    assert(first.speed_rad_s == 1.0 && second.speed_rad_s == 0.0);
    assert(speed_trajectory_update(&first, &config, -1.0, 0.01) == 0.98);
    for (size_t i = 0U; i < 49U; ++i) {
        (void)speed_trajectory_update(&first, &config, -1.0, 0.01);
    }
    assert(first.speed_rad_s == 0.0);
    assert(speed_trajectory_update(&first, &config, -1.0, 0.01) == -0.01);
    assert(speed_trajectory_update(&second, &config, 0.5, 0.1) == 0.1);
    assert(first.speed_rad_s == -0.01);
    puts("Independent trajectory, deadband, limit and reversal checks passed");
    return 0;
}
