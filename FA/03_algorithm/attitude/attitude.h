#ifndef FRAME_CDC_ALGORITHM_ATTITUDE_ATTITUDE_H
#define FRAME_CDC_ALGORITHM_ATTITUDE_ATTITUDE_H

#include <stdbool.h>

typedef enum {
    ATTITUDE_STATUS_OK = 0,
    ATTITUDE_STATUS_WAITING,
    ATTITUDE_STATUS_GYRO_ONLY,
    ATTITUDE_STATUS_INVALID_SAMPLE,
    ATTITUDE_STATUS_GAP,
} attitude_status_t;

typedef struct {
    double accel_body_m_s2[3];
    double gyro_body_rad_s[3];
    bool is_valid;
} attitude_observation_t;

typedef struct {
    double q_odom_from_body[4]; /* Hamilton，wxyz，将 body 向量转到 odom。 */
    attitude_status_t status;
    bool is_initialized;
} attitude_t;

/** 作用将估计器初始化为单位四元数和等待状态。
 * 输入非空 attitude，输出已初始化的四元数、估计状态和初始化标志，无返回值。 */
void attitude_init(attitude_t *attitude);

/** 作用融合陀螺仪与重力观测更新姿态，异常观测或间隔保留姿态并更新状态。
 * 输入非空 attitude、observation 及真实 dt_s（0 表示首次或中断后），输出估计器状态，无返回值。 */
void attitude_update(attitude_t *attitude, const attitude_observation_t *observation, double dt_s);

/** 作用读取 body 到 odom 的 Z-Y-X 俯仰主值，绕 +Y 为正。
 * 输入非空 attitude 和 pitch_rad，输出 [-π/2,π/2] rad 俯仰，成功返回 true，失败保留原输出并返回 false。 */
bool attitude_get_pitch_rad(const attitude_t *attitude, double *pitch_rad);

#endif /* FRAME_CDC_ALGORITHM_ATTITUDE_ATTITUDE_H */
