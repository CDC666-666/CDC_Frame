#ifndef FRAME_CDC_ASSEMBLY_FRAME_CONFIG_H
#define FRAME_CDC_ASSEMBLY_FRAME_CONFIG_H

#include "damiao_motor.h"
#include "speed_controller.h"

typedef struct {
    uint64_t sample_period_us;
} imu_observer_config_t;

typedef struct {
    uint64_t period_us;
} telemetry_config_t;

typedef struct {
    uint32_t bitrate_hz; /* 0 表示尚未核实，禁止启动总线。 */
} frame_can_config_t;

typedef struct {
    imu_observer_config_t imu;
    telemetry_config_t telemetry;
    frame_can_config_t can;
    damiao_mit_config_t motor;
    damiao_motor_execution_config_t execution;
    speed_controller_config_t controller;
} frame_config_t;

/** 构建目录中的自动生成 const 数据，参数权威来源为选定 YAML。 */
extern const frame_config_t g_frame_config;

#endif /* FRAME_CDC_ASSEMBLY_FRAME_CONFIG_H */
