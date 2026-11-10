/* 单电机调试数据的 UART7 二进制协议。 */
#include "motor_telemetry.h"

#include "mc02_board.h"
#include "telemetry_encoding.h"

void motor_telemetry_init(motor_telemetry_t *telemetry)
{
    *telemetry = (motor_telemetry_t){
        .frame = {0xA5, 0x5A, 0x44, 0x4D, [70] = 0x80, [71] = 0x7F},
        .offset = MOTOR_TELEMETRY_FRAME_SIZE,
    };
}

void motor_telemetry_submit(motor_telemetry_t *telemetry,
                            const motor_telemetry_snapshot_t *snapshot)
{
    if (telemetry->offset != sizeof(telemetry->frame)) {
        return;
    }
    const damiao_motor_output_t *motor = &snapshot->motor;
    const float channels[] = {
        (float)snapshot->pitch_rad,
        (float)snapshot->target_speed_rad_s,
        (float)motor->feedback.speed_rad_s,
        (float)motor->status,
        (float)snapshot->is_armed,
        (float)motor->feedback.state_raw,
        (float)motor->bus_status,
        (float)snapshot->bench_state,
        (float)snapshot->is_pitch_valid,
        (float)motor->has_scaled_feedback,
        (float)snapshot->imu_sample_dt_s,
        (float)snapshot->imu_missed_samples,
        (float)motor->feedback.position_rad,
        (float)motor->feedback.torque_n_m,
        (float)motor->feedback.mos_deg_c,
        (float)motor->feedback.rotor_deg_c,
    };
    _Static_assert(sizeof(channels) + 8U == MOTOR_TELEMETRY_FRAME_SIZE, "motor telemetry size");
    for (size_t i = 0U; i < sizeof(channels) / sizeof(channels[0]); ++i) {
        telemetry_encoding_write_float32_le(&telemetry->frame[4U + 4U * i], channels[i]);
    }
    telemetry->offset = 0U;
}

void motor_telemetry_poll(motor_telemetry_t *telemetry)
{
    if (telemetry->offset < sizeof(telemetry->frame)) {
        telemetry->offset += mc02_board_uart_write(telemetry->frame + telemetry->offset,
                                                   sizeof(telemetry->frame) - telemetry->offset);
    }
}
