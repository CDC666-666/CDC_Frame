/* 电机台架遥测的周期、通道快照和非阻塞发送。 */
#include "motor_telemetry.h"

/** 作用初始化电机遥测帧缓冲、周期调度和非阻塞写入接口。
 * 输入非空 telemetry、同源 first_frame_us、正 period_us 和 write_bytes，输出就绪的 telemetry，无返回值。 */
void motor_telemetry_init(motor_telemetry_t *telemetry, uint64_t first_frame_us, uint64_t period_us,
                          size_t (*write_bytes)(const uint8_t *, size_t))
{
    telemetry->offset = sizeof(telemetry->frame);
    telemetry->next_frame_us = first_frame_us;
    telemetry->period_us = period_us;
    telemetry->write_bytes = write_bytes;
}

/** 作用在缓冲可用时从独立输入选取 16 路电机遥测并编码。
 * 输入非空 telemetry、motor、control、imu 和 pose，输出快照帧及发送偏移，无返回值。 */
static void submit_frame(motor_telemetry_t *telemetry, const damiao_motor_output_t *motor,
                         const speed_controller_output_t *control, const imu_observer_output_t *imu,
                         const attitude_sample_t *pose)
{
    if (telemetry->offset != sizeof(telemetry->frame)) {
        return;
    }

    double pitch_rad = 0.0;
    if (control->is_pitch_valid) {
        (void)attitude_get_pitch_rad(&pose->attitude, &pitch_rad);
    }
    const motor_telemetry_fields_t fields = {
        .channels =
            {
                (float)pitch_rad,
                (motor->state == DAMIAO_MOTOR_ARMED ? (float)control->target_speed_rad_s : 0.0f),
                (float)motor->feedback.speed_rad_s,
                (float)motor->status,
                motor->state == DAMIAO_MOTOR_ARMED ? 1.0f : 0.0f,
                (float)motor->feedback.state_raw,
                (float)motor->bus_status,
                (float)motor->state,
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

/** 作用按周期选取设备、目标、IMU 和 INS 快照并非阻塞推进输出，输入本次同步消费。
 * 输入非空 telemetry、motor、control、imu、pose 和 now_us，输出快照帧、发送偏移和调度状态，无返回值。 */
void motor_telemetry_update(motor_telemetry_t *telemetry, const damiao_motor_output_t *motor,
                            const speed_controller_output_t *control,
                            const imu_observer_output_t *imu, const attitude_sample_t *pose,
                            uint64_t now_us)
{
    if (telemetry->period_us == 0U || telemetry->write_bytes == NULL) {
        return;
    }
    if (now_us >= telemetry->next_frame_us) {
        telemetry->next_frame_us = now_us + telemetry->period_us;
        submit_frame(telemetry, motor, control, imu, pose);
    }
    if (telemetry->offset < sizeof(telemetry->frame)) {
        telemetry->offset += telemetry->write_bytes(telemetry->frame + telemetry->offset,
                                                    sizeof(telemetry->frame) - telemetry->offset);
    }
}
