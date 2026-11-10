/* 组装 MC02 的板级、设备、算法、台架应用及 UART7 输出。 */
#include "frame_system.h"

#include <string.h>

#include "attitude.h"
#include "imu_device.h"
#include "mc02_board.h"

#ifdef FRAME_CDC_MOTOR_BENCH
#include "single_motor_control.h"
#endif
#ifdef FRAME_CDC_MOTOR_TELEMETRY
#include "motor_telemetry.h"
#else
#include "imu_telemetry.h"
#endif

#define FRAME_SYSTEM_TELEMETRY_PERIOD_US 20000U

static bool s_board_is_ready;
static imu_device_t s_imu;
static attitude_sample_t s_pose;
static uint64_t s_next_telemetry_us;

#ifdef FRAME_CDC_MOTOR_BENCH
static can_service_t s_can1;
static damiao_motor_t s_motor;
static single_motor_control_t s_control;
static operator_input_t s_operator;
static bool s_has_operator;

static const uint32_t s_can1_bitrate_hz = 0U; /* 待读实物 CAN 波特率。 */
static const damiao_mit_config_t s_motor_config = {
    .motor_id = 0x01U,
    .master_id = 0x11U,
    .position_max_rad = 0.0, /* 待读实物 P_MAX。 */
    .speed_max_rad_s = 0.0,  /* 待读实物 V_MAX。 */
    .torque_max_n_m = 0.0,   /* 待读实物 T_MAX。 */
    .feedback_timeout_us = 50000U,
};

static const single_motor_control_config_t s_control_config = {
    .mode = SINGLE_MOTOR_FEEDBACK,
    .has_status_request = false, /* 核对固件支持 0x7FF/0xCC 后开启。 */
    .control_period_us = 10000U,
    .attitude_timeout_us = 25000U,
    .enable_timeout_us = 100000U,
    .center_pitch_rad = 0.0,
    .pitch_deadband_rad = 0.05,
    .pitch_to_speed_per_s = 4.0, /* 电机安装反向时取负值。 */
    .fixed_speed_rad_s = 0.5,
    .speed_limit_rad_s = 1.0,
    .accel_limit_rad_s2 = 1.0,
    .decel_limit_rad_s2 = 2.0,
    .kd_n_m_s_per_rad = 0.1,
};
#endif

#ifdef FRAME_CDC_MOTOR_TELEMETRY
static motor_telemetry_t s_telemetry;
#else
static imu_telemetry_t s_telemetry;
#endif

void frame_system_init(void)
{
    s_board_is_ready = mc02_board_init();
    if (!s_board_is_ready) {
        return;
    }
    imu_device_init(&s_imu);
    attitude_init(&s_pose.attitude);
    s_next_telemetry_us = s_imu.next_sample_us;
#ifdef FRAME_CDC_MOTOR_BENCH
    can_service_init(&s_can1, MC02_CAN_PORT_1);
    const bool is_registered = damiao_motor_register(&s_motor, &s_can1, &s_motor_config);
    const uint64_t now_us = mc02_board_time_us();
    const bool is_started = is_registered && can_service_start(&s_can1, s_can1_bitrate_hz, now_us);
    single_motor_control_init(&s_control, &s_motor, &s_control_config, now_us);
    s_has_operator = is_started && s_control.can_move;
    if (s_has_operator) {
        mc02_board_uart_enable_rx();
    }
#endif
#ifdef FRAME_CDC_MOTOR_TELEMETRY
    motor_telemetry_init(&s_telemetry);
#else
    imu_telemetry_init(&s_telemetry);
#endif
}

static void poll_attitude(uint64_t now_us)
{
    if (!imu_device_poll(&s_imu, now_us)) {
        return;
    }
    const imu_device_output_t *imu = &s_imu.output;
    attitude_observation_t observation = {.is_valid = imu->is_valid};
    memcpy(observation.accel_body_m_s2, imu->accel_body_m_s2, sizeof(observation.accel_body_m_s2));
    memcpy(observation.gyro_body_rad_s, imu->gyro_body_rad_s, sizeof(observation.gyro_body_rad_s));
    const double dt_s =
        s_pose.attitude.status == ATTITUDE_STATUS_INVALID_SAMPLE ? 0.0 : imu->continuous_dt_s;
    attitude_update(&s_pose.attitude, &observation, dt_s);
    s_pose.sample_time_us = imu->sample_time_us;
    s_pose.is_sample_valid = imu->is_valid;
}

static void poll_telemetry(uint64_t now_us)
{
    if (now_us >= s_next_telemetry_us) {
#ifdef FRAME_CDC_MOTOR_TELEMETRY
        s_next_telemetry_us = now_us + FRAME_SYSTEM_TELEMETRY_PERIOD_US;
        const single_motor_control_output_t *output = &s_control.output;
        const motor_telemetry_snapshot_t snapshot = {
            .motor = output->motor,
            .pitch_rad = output->pitch_rad,
            .target_speed_rad_s = output->target_speed_rad_s,
            .imu_sample_dt_s = s_imu.output.sample_dt_s,
            .imu_missed_samples = s_imu.output.missed_samples,
            .bench_state = (uint8_t)output->state,
            .is_armed = output->state == SINGLE_MOTOR_ARMED,
            .is_pitch_valid = output->is_pitch_valid,
        };
        motor_telemetry_submit(&s_telemetry, &snapshot);
#else
        const uint64_t slots =
            (now_us - s_next_telemetry_us) / FRAME_SYSTEM_TELEMETRY_PERIOD_US + 1U;
        s_next_telemetry_us += slots * FRAME_SYSTEM_TELEMETRY_PERIOD_US;
        imu_telemetry_submit(&s_telemetry, &s_imu.output, &s_pose.attitude, (uint32_t)(slots - 1U));
#endif
    }
#ifdef FRAME_CDC_MOTOR_TELEMETRY
    motor_telemetry_poll(&s_telemetry);
#else
    imu_telemetry_poll(&s_telemetry);
#endif
}

void frame_system_poll(void)
{
    if (!s_board_is_ready) {
        return;
    }
#ifdef FRAME_CDC_MOTOR_BENCH
    can_service_poll_rx(&s_can1, mc02_board_time_us());
#endif
    const uint64_t sample_now_us = mc02_board_time_us();
    poll_attitude(sample_now_us);
#ifdef FRAME_CDC_MOTOR_BENCH
    const uint64_t control_now_us = mc02_board_time_us();
    const operator_command_t command =
        s_has_operator ? operator_input_poll(&s_operator, control_now_us) : OPERATOR_COMMAND_NONE;
    single_motor_control_update(&s_control, &s_pose, command, control_now_us);
    /* 提交事件由 CAN 服务记录，下轮控制再消费。 */
    (void)can_service_poll_tx(&s_can1);
#endif
#ifdef FRAME_CDC_MOTOR_TELEMETRY
    poll_telemetry(control_now_us);
#else
    poll_telemetry(sample_now_us);
#endif
}
