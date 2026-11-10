/* IMU 遥测的时槽、通道快照和非阻塞发送。 */
#include "imu_telemetry.h"

#include "mc02_board.h"

#define IMU_TELEMETRY_PERIOD_US 20000U

void imu_telemetry_init(imu_telemetry_t *telemetry, uint64_t first_frame_us)
{
    telemetry->frame_offset = sizeof(telemetry->frame);
    telemetry->next_frame_us = first_frame_us;
    telemetry->dropped_frames = 0U;
}

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

void imu_telemetry_update(imu_telemetry_t *telemetry, const imu_observer_output_t *imu,
                          const attitude_sample_t *pose, uint64_t now_us)
{
    if (now_us >= telemetry->next_frame_us) {
        const uint64_t slots = (now_us - telemetry->next_frame_us) / IMU_TELEMETRY_PERIOD_US + 1U;
        telemetry->next_frame_us += slots * IMU_TELEMETRY_PERIOD_US;
        telemetry->dropped_frames += (uint32_t)(slots - 1U);
        submit_frame(telemetry, imu, pose);
    }

    const size_t remaining = sizeof(telemetry->frame) - telemetry->frame_offset;
    telemetry->frame_offset +=
        mc02_board_uart_write(telemetry->frame + telemetry->frame_offset, remaining);
}
