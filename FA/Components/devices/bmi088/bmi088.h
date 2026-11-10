#ifndef FRAME_CDC_DEVICES_BMI088_BMI088_H
#define FRAME_CDC_DEVICES_BMI088_BMI088_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BMI088_STATUS_OK = 0,
    BMI088_STATUS_NOT_INITIALIZED,
    BMI088_STATUS_BUS_ERROR,
    BMI088_STATUS_BAD_ID,
    BMI088_STATUS_CONFIG_ERROR,
    BMI088_STATUS_NOT_READY,
} bmi088_status_t;

typedef struct {
    double accel_sensor_m_s2[3];
    double gyro_sensor_rad_s[3];
    double temperature_deg_c;
    uint64_t time_us;
    uint32_t sequence;
    bool is_valid;
    bmi088_status_t status;
} bmi088_sample_t;

typedef struct {
    bmi088_status_t accel_init_status;
    bmi088_status_t gyro_init_status;
    bmi088_sample_t sample;
} bmi088_t;

/** @brief 分别复位、识别和配置两个芯片，填入各自的初始化状态。
 * @param device 非空实例。只在主循环启动前调用，包含上电等待。
 */
void bmi088_init(bmi088_t *device);

/** @brief 读取本轮配对观测，使用传感器坐标系和 SI 单位。
 * @param device 非空且已初始化。失败仅更新 status 和 is_valid，保留最后一组数据。
 * @param time_us MCU 从 TIM2 取得的本次读取起始时间。
 * @param sequence 本次调度槽编号，成功后随样本发布。
 */
void bmi088_sample(bmi088_t *device, uint64_t time_us, uint32_t sequence);

#endif /* FRAME_CDC_DEVICES_BMI088_BMI088_H */
