#ifndef FRAME_CDC_COMPONENTS_TELEMETRY_TELEMETRY_ENCODING_H
#define FRAME_CDC_COMPONENTS_TELEMETRY_TELEMETRY_ENCODING_H

#include <stdint.h>

enum {
    IMU_TELEMETRY_CHANNEL_COUNT = 21,
    IMU_TELEMETRY_FRAME_SIZE = 92,
    MOTOR_TELEMETRY_CHANNEL_COUNT = 16,
    MOTOR_TELEMETRY_FRAME_SIZE = 72,
};

/* 各快照的通道按协议规定的顺序排列，数值已经转换为线上 float32。 */
typedef struct {
    float channels[IMU_TELEMETRY_CHANNEL_COUNT];
} imu_telemetry_fields_t;

typedef struct {
    float channels[MOTOR_TELEMETRY_CHANNEL_COUNT];
} motor_telemetry_fields_t;

/** 作用将按 IM 顺序排列的 21 路通道编码成 IMU 遥测帧。
 * 输入非空 92 字节 frame 和 fields，输出包含帧头、通道和帧尾的 frame，无返回值。 */
void telemetry_encode_imu(uint8_t frame[IMU_TELEMETRY_FRAME_SIZE],
                          const imu_telemetry_fields_t *fields);

/** 作用将按 DM 顺序排列的 16 路通道编码成电机遥测帧。
 * 输入非空 72 字节 frame 和 fields，输出包含帧头、通道和帧尾的 frame，无返回值。 */
void telemetry_encode_motor(uint8_t frame[MOTOR_TELEMETRY_FRAME_SIZE],
                            const motor_telemetry_fields_t *fields);

#endif /* FRAME_CDC_COMPONENTS_TELEMETRY_TELEMETRY_ENCODING_H */
