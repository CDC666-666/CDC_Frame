/* IMU 遥测的时槽、通道快照和非阻塞发送。 */
#include "imu_telemetry.h"

/** 作用初始化 IMU 遥测帧缓冲、周期调度和非阻塞写入接口。
 * 输入非空 telemetry、同源 first_frame_us、正 period_us 和 write_bytes，输出就绪的 telemetry，无返回值。 */
void imu_telemetry_init(imu_telemetry_t *telemetry, uint64_t first_frame_us, uint64_t period_us,
                        size_t (*write_bytes)(const uint8_t *, size_t))
{
    telemetry->frame_offset = sizeof(telemetry->frame);
    telemetry->next_frame_us = first_frame_us;
    telemetry->period_us = period_us;
    telemetry->write_bytes = write_bytes;
    telemetry->dropped_frames = 0U;
}

/** 作用在缓冲可用时编码 IMU 快照，忙时记录丢帧。
 * 输入非空 telemetry、imu 和 pose，输出待发送帧及偏移或丢帧计数，无返回值。 */
static void submit_frame(imu_telemetry_t *telemetry, const imu_observer_output_t *imu,
                         const attitude_sample_t *pose)
{
    if (telemetry->frame_offset != sizeof(telemetry->frame)) {
        ++telemetry->dropped_frames;
        return;
    }

    const imu_observation_t *observation = &imu->observation;
    const double *q = pose->attitude.q_odom_from_body;
    const double *a = observation->body.accel_body_m_s2;
    const double *g = observation->body.gyro_body_rad_s;
    const imu_telemetry_fields_t fields = {
        .channels =
            {
                (float)q[0],
                (float)q[1],
                (float)q[2],
                (float)q[3],
                (float)a[0],
                (float)a[1],
                (float)a[2],
                (float)g[0],
                (float)g[1],
                (float)g[2],
                (float)imu->temperature_deg_c,
                (float)imu->sample_dt_s,
                observation->body.is_valid ? 1.0f : 0.0f,
                (float)pose->attitude.status,
                (float)imu->sample_status,
                (float)observation->sample_sequence,
                (float)((double)observation->sample_time_us * 1e-6),
                (float)imu->missed_samples,
                (float)telemetry->dropped_frames,
                (float)imu->accel_init_status,
                (float)imu->gyro_init_status,
            },
    };
    telemetry_encode_imu(telemetry->frame, &fields);
    telemetry->frame_offset = 0U;
}

/** 作用按时槽选取 IMU/INS 快照并非阻塞推进输出，输入只在主循环本次调用使用。
 * 输入非空 telemetry、imu、pose 和 now_us，输出帧缓冲、发送偏移及丢帧统计，无返回值。 */
void imu_telemetry_update(imu_telemetry_t *telemetry, const imu_observer_output_t *imu,
                          const attitude_sample_t *pose, uint64_t now_us)
{
    if (telemetry->period_us == 0U || telemetry->write_bytes == NULL) {
        return;
    }
    if (now_us >= telemetry->next_frame_us) {
        const uint64_t slots = (now_us - telemetry->next_frame_us) / telemetry->period_us + 1U;
        telemetry->next_frame_us += slots * telemetry->period_us;
        telemetry->dropped_frames += (uint32_t)(slots - 1U);
        submit_frame(telemetry, imu, pose);
    }

    const size_t remaining = sizeof(telemetry->frame) - telemetry->frame_offset;
    telemetry->frame_offset +=
        telemetry->write_bytes(telemetry->frame + telemetry->frame_offset, remaining);
}
