#ifndef FRAME_CDC_COMPONENTS_HARDWARE_IMU_IMU_OBSERVER_H
#define FRAME_CDC_COMPONENTS_HARDWARE_IMU_IMU_OBSERVER_H

#include "bmi088.h"
#include "imu_observation.h"

typedef struct {
    imu_observation_t observation;
    double temperature_deg_c;
    double sample_dt_s; /* 相邻成功采样的间隔，当前失败或首次成功时为 0。 */
    uint32_t missed_samples;
    bmi088_status_t sample_status;
    bmi088_status_t accel_init_status;
    bmi088_status_t gyro_init_status;
} imu_observer_output_t;

typedef struct {
    bmi088_t *sensor;
    imu_observer_output_t output;
    uint64_t next_sample_us;
    uint64_t sample_period_us;
    uint32_t slot_sequence;
    bool has_previous_sample;
} imu_observer_t;

/** 作用绑定已初始化的 BMI088 并建立采样调度，主循环期间实例与设备地址须稳定。
 * 输入非空 observer、sensor、first_sample_us 和 sample_period_us，输出初始化的 observer，无返回值。 */
void imu_observer_init(imu_observer_t *observer, bmi088_t *sensor, uint64_t first_sample_us,
                       uint64_t sample_period_us);

/** 作用按采样槽尝试采集并发布 body 观测、时间和采样诊断。
 * 输入非空 observer 和同源 now_us（微秒），输出 observer.output 及调度状态，尝试采样返回 true。 */
bool imu_observer_update(imu_observer_t *observer, uint64_t now_us);

#endif /* FRAME_CDC_COMPONENTS_HARDWARE_IMU_IMU_OBSERVER_H */
