/* 组装 MC02 的板级、设备、算法、台架应用及 UART7 输出。 */
#include "frame_system.h"

#include "bmi088.h"
#include "imu_observer.h"
#include "ins.h"
#include "mc02_board.h"

#ifdef FRAME_CDC_MOTOR_BENCH
#include "single_motor_control.h"
#endif
#ifdef FRAME_CDC_MOTOR_TELEMETRY
#include "motor_telemetry.h"
#else
#include "imu_telemetry.h"
#endif

static bool s_board_is_ready;
static bmi088_t s_bmi088;
static imu_observer_t s_imu;
static ins_t s_ins;

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
    bmi088_init(&s_bmi088);
    const uint64_t first_sample_us = mc02_board_time_us();
    imu_observer_init(&s_imu, &s_bmi088, first_sample_us);
    ins_init(&s_ins);
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
    motor_telemetry_init(&s_telemetry, first_sample_us);
#else
    imu_telemetry_init(&s_telemetry, first_sample_us);
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
    if (imu_observer_update(&s_imu, sample_now_us)) {
        ins_update(&s_ins, &s_imu.output.observation);
    }
#ifdef FRAME_CDC_MOTOR_BENCH
    const uint64_t control_now_us = mc02_board_time_us();
    const operator_command_t command =
        s_has_operator ? operator_input_poll(&s_operator, control_now_us, mc02_board_uart_read)
                       : OPERATOR_COMMAND_NONE;
    single_motor_control_update(&s_control, &s_ins.output, command, control_now_us);
    /* 提交事件由 CAN 服务记录，下轮控制再消费。 */
    (void)can_service_poll_tx(&s_can1);
#endif
#ifdef FRAME_CDC_MOTOR_TELEMETRY
    motor_telemetry_update(&s_telemetry, &s_control.output, &s_imu.output, control_now_us);
#else
    imu_telemetry_update(&s_telemetry, &s_imu.output, &s_ins.output, sample_now_us);
#endif
}
