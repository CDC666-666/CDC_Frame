#ifndef FRAME_CDC_APP_IMU_IMU_APP_H
#define FRAME_CDC_APP_IMU_IMU_APP_H

/** @brief 初始化首条验证链。仅在主循环启动前调用，传感器上电期间会等待。 */
void imu_app_init(void);

/** @brief 在主循环推进采样、估计和串口发送。串口路径不等待硬件。 */
void imu_app_poll(void);

#endif /* FRAME_CDC_APP_IMU_IMU_APP_H */
