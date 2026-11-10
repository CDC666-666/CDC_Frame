/* 电机台架遥测的周期、通道快照和非阻塞发送。 */
#include "motor_telemetry.h"

#include "mc02_board.h"

#define MOTOR_TELEMETRY_PERIOD_US 20000U

void motor_telemetry_init(motor_telemetry_t *telemetry, uint64_t first_frame_us)
{
    telemetry->offset = sizeof(telemetry->frame);
    telemetry->next_frame_us = first_frame_us;
}

static void submit_frame(motor_telemetry_t *telemetry, const single_motor_control_output_t *control,
                         const imu_observer_output_t *imu)
{
    if (telemetry->offset != sizeof(telemetry->frame)) {
        return;
    }
    const damiao_motor_output_t *motor = &control->motor;
    const motor_telemetry_fields_t fields = {
        .channels =
            {
                (float)control->pitch_rad,
                (float)control->target_speed_rad_s,
                (float)motor->feedback.speed_rad_s,
                (float)motor->status,
                control->state == SINGLE_MOTOR_ARMED ? 1.0f : 0.0f,
                (float)motor->feedback.state_raw,
                (float)motor->bus_status,
                (float)control->state,
                (float)control->is_pitch_valid,
                (float)motor->has_scaled_feedback,
                (float)imu->sample_dt_s,
                (float)imu->missed_samples,
                (float)motor->feedback.position_rad,
                (float)motor->feedback.torque_n_m,
                (float)motor->feedback.mos_deg_c,
                (float)motor->feedback.rotor_deg_c,
            },
    };
    telemetry_encode_motor(telemetry->frame, &fields);
    telemetry->offset = 0U;
}

void motor_telemetry_update(motor_telemetry_t *telemetry,
                            const single_motor_control_output_t *control,
                            const imu_observer_output_t *imu, uint64_t now_us)
{
    if (now_us >= telemetry->next_frame_us) {
        telemetry->next_frame_us = now_us + MOTOR_TELEMETRY_PERIOD_US;
        submit_frame(telemetry, control, imu);
    }
    if (telemetry->offset < sizeof(telemetry->frame)) {
        telemetry->offset += mc02_board_uart_write(telemetry->frame + telemetry->offset,
                                                   sizeof(telemetry->frame) - telemetry->offset);
    }
}
