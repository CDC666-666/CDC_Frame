#ifndef FRAME_CDC_ALGORITHMS_ATTITUDE_ATTITUDE_H
#define FRAME_CDC_ALGORITHMS_ATTITUDE_ATTITUDE_H

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

/** @brief 将非空估计器初始化为单位四元数和等待状态。 */
void attitude_init(attitude_t *attitude);

/** @brief 根据当前观测更新姿态。指针非空，主机与 MCU 使用相同接口。
 * @param dt_s 相邻连续观测的真实间隔。0 表示首样本或中断后的首样本。
 * @details 无效观测或间隔保留上次姿态并更新状态；初始化需要近静止的重力观测。
 */
void attitude_update(attitude_t *attitude, const attitude_observation_t *observation, double dt_s);

#endif /* FRAME_CDC_ALGORITHMS_ATTITUDE_ATTITUDE_H */
