/* 仅用陀螺仪和重力方向的 Mahony 型比例互补滤波。 */
#include "attitude.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "math_utils.h"

static const double s_gravity_m_s2 = 9.80665;
static const double s_max_step_s = 0.025;
static const double s_gravity_gain_per_s = 2.0;
static const double s_max_initial_rate_rad_s = 0.15;

/** 作用将估计器初始化为单位四元数和等待状态。
 * 输入非空 attitude，输出已初始化的四元数、估计状态和初始化标志，无返回值。 */
void attitude_init(attitude_t *attitude)
{
    *attitude = (attitude_t){.q_odom_from_body = {1.0}, .status = ATTITUDE_STATUS_WAITING};
}

/** 作用计算三维向量模长的平方。
 * 输入非空三元素 vector，输出三个分量平方和。 */
static double norm_squared(const double vector[3])
{
    return vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
}

/** 作用用近静止重力方向初始化姿态并将启动水平航向设为 odom 航向。
 * 输入非空 attitude 和 body 系 accel_body_m_s2，输出四元数、OK 状态及初始化标志，无返回值。 */
static void initialize_from_gravity(attitude_t *attitude, const double accel_body_m_s2[3])
{
    const double horizontal_m_s2 = hypot(accel_body_m_s2[1], accel_body_m_s2[2]);
    const double half_roll_rad = 0.5 * atan2(accel_body_m_s2[1], accel_body_m_s2[2]);
    const double half_pitch_rad = 0.5 * atan2(-accel_body_m_s2[0], horizontal_m_s2);
    const double cr = cos(half_roll_rad);
    const double sr = sin(half_roll_rad);
    const double cp = cos(half_pitch_rad);
    const double sp = sin(half_pitch_rad);
    /* Rz(0) * Ry(pitch) * Rx(roll)，启动姿态定义 odom 的水平航向。 */
    double *q = attitude->q_odom_from_body;
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
    attitude->is_initialized = true;
    attitude->status = ATTITUDE_STATUS_OK;
}

/** 作用利用比力与预测重力方向的偏差修正积分角速度。
 * 输入q、body 比力、正 accel_norm_m_s2 及 rate_body_rad_s，输出原位修正的 rad/s 角速度，无返回值。 */
static void correct_gravity(const double q[4], const double accel_body_m_s2[3],
                            double accel_norm_m_s2, double rate_body_rad_s[3])
{
    /* 将 odom 的 +Z 旋回 body，和归一化比力做叉乘。 */
    const double up_body[3] = {
        2.0 * (q[1] * q[3] - q[0] * q[2]),
        2.0 * (q[0] * q[1] + q[2] * q[3]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]),
    };
    const double gain = s_gravity_gain_per_s / accel_norm_m_s2;
    const double *a = accel_body_m_s2;
    rate_body_rad_s[0] += gain * (a[1] * up_body[2] - a[2] * up_body[1]);
    rate_body_rad_s[1] += gain * (a[2] * up_body[0] - a[0] * up_body[2]);
    rate_body_rad_s[2] += gain * (a[0] * up_body[1] - a[1] * up_body[0]);
}

/** 作用融合陀螺仪与重力观测更新姿态，异常观测或间隔保留姿态并更新状态。
 * 输入非空 attitude、observation 及真实 dt_s（0 表示首次或中断后），输出估计器状态，无返回值。 */
void attitude_update(attitude_t *attitude, const attitude_observation_t *observation, double dt_s)
{
    const double *a = observation->accel_body_m_s2;
    const double accel_norm_m_s2 = sqrt(norm_squared(a));
    const double gyro_norm_squared = norm_squared(observation->gyro_body_rad_s);
    if (!observation->is_valid || !isfinite(accel_norm_m_s2) || !isfinite(gyro_norm_squared) ||
        gyro_norm_squared > 100.0 * 100.0) {
        attitude->status = ATTITUDE_STATUS_INVALID_SAMPLE;
        return;
    }
    if (!isfinite(dt_s) || dt_s < 0.0 || dt_s > s_max_step_s) {
        attitude->status = ATTITUDE_STATUS_GAP;
        return;
    }

    const bool has_gravity = fabs(accel_norm_m_s2 - s_gravity_m_s2) <= 0.2 * s_gravity_m_s2;
    if (!attitude->is_initialized) {
        if (has_gravity &&
            gyro_norm_squared <= s_max_initial_rate_rad_s * s_max_initial_rate_rad_s) {
            initialize_from_gravity(attitude, a);
        } else {
            attitude->status = ATTITUDE_STATUS_WAITING;
        }
        return;
    }
    if (dt_s == 0.0) {
        attitude->status = ATTITUDE_STATUS_GAP;
        return;
    }

    double *q = attitude->q_odom_from_body;
    double rate_body_rad_s[3];
    memcpy(rate_body_rad_s, observation->gyro_body_rad_s, sizeof(rate_body_rad_s));
    if (has_gravity) {
        correct_gravity(q, a, accel_norm_m_s2, rate_body_rad_s);
    }

    /* q_dot = 0.5 * q ⊗ [0, omega_body]；小步 Euler 后归一化。 */
    const double x = 0.5 * dt_s * rate_body_rad_s[0];
    const double y = 0.5 * dt_s * rate_body_rad_s[1];
    const double z = 0.5 * dt_s * rate_body_rad_s[2];
    const double next[4] = {
        q[0] - q[1] * x - q[2] * y - q[3] * z,
        q[1] + q[0] * x + q[2] * z - q[3] * y,
        q[2] + q[0] * y - q[1] * z + q[3] * x,
        q[3] + q[0] * z + q[1] * y - q[2] * x,
    };
    const double inverse_norm = 1.0 / sqrt(next[0] * next[0] + norm_squared(next + 1));
    for (size_t i = 0; i < 4; ++i) {
        q[i] = next[i] * inverse_norm;
    }
    attitude->status = has_gravity ? ATTITUDE_STATUS_OK : ATTITUDE_STATUS_GYRO_ONLY;
}

/** 作用读取 body 到 odom 的 Z-Y-X 俯仰主值，绕 +Y 为正。
 * 输入非空 attitude 和 pitch_rad，输出 [-π/2,π/2] rad 俯仰，成功返回 true，失败保留原输出并返回 false。 */
bool attitude_get_pitch_rad(const attitude_t *attitude, double *pitch_rad)
{
    if (!attitude->is_initialized ||
        (attitude->status != ATTITUDE_STATUS_OK && attitude->status != ATTITUDE_STATUS_GYRO_ONLY)) {
        return false;
    }
    const double *q = attitude->q_odom_from_body;
    const double sin_pitch = 2.0 * (q[0] * q[2] - q[1] * q[3]);
    if (!isfinite(sin_pitch)) {
        return false;
    }
    *pitch_rad = asin(math_utils_clamp(sin_pitch, -1.0, 1.0));
    return true;
}
