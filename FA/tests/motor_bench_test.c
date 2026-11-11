#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mc02_board.h"
#include "motor_bench.h"
#include "motor_telemetry.h"

static const damiao_mit_config_t s_motor = {1U, 0x11U, 12.5, 10.0, 6.0, 50000U};
static const frame_config_t s_control_config = {
    .can = {.bitrate_hz = 500000U},
    .motor = {1U, 0x11U, 12.5, 10.0, 6.0, 50000U},
    .execution =
        {
            .has_status_request = true,
            .enable_timeout_us = 100000U,
            .status_period_us = 20000U,
            .disable_retry_us = 20000U,
            .kd_n_m_s_per_rad = 0.1,
        },
    .controller =
        {
            .control_period_us = 10000U,
            .attitude_timeout_us = 25000U,
            .fixed_speed_rad_s = 0.5,
            .pitch = {.pitch_deadband_rad = 0.05, .pitch_to_speed_per_s = 4.0},
            .trajectory =
                {
                    .speed_limit_rad_s = 1.0,
                    .accel_limit_rad_s2 = 1.0,
                    .decel_limit_rad_s2 = 2.0,
                },
        },
};
static motor_bench_t s_control;
static uint64_t s_now_us;
static mc02_can_status_t s_bus;
static bool s_has_tx;
static bool s_tx_pending;
static bool s_tx_transmitted;
static bool s_finish_after_pending_read;
static bool s_fail_submit;
static can_frame_t s_rx[MC02_CAN_RX_DEPTH];
static size_t s_rx_count;
static size_t s_rx_offset;
static can_frame_t s_tx;
static const char *s_command;
static size_t s_cancel_count;
static size_t s_telemetry_count;
static size_t s_filter_count;
static size_t s_init_count;
static uint16_t s_filter_ids[MC02_CAN_FILTER_CAPACITY];
static uint8_t s_wire[MOTOR_TELEMETRY_FRAME_SIZE];

/** 作用模拟台架启用串口接收的板级入口。
 * 输入无参数，输出无，无返回值，测试字节由串口读取替身提供。 */
void mc02_board_uart_enable_rx(void)
{
}

/** 作用提供由测试控制的假板级时钟。
 * 输入无参数，读取全局 s_now_us，输出当前假时间（微秒）。 */
uint64_t mc02_board_time_us(void)
{
    return s_now_us;
}

/** 作用从当前测试命令字符串提供一个非阻塞字节。
 * 输入全局 s_command，输出下一字节并推进游标，字符串结束返回 -1。 */
int mc02_board_uart_read(void)
{
    return *s_command != '\0' ? (unsigned char)*s_command++ : -1;
}

/** 作用接收假电机遥测写入并核对帧头、帧尾和长度。
 * 输入非空 data 和 size，输出保存的 s_wire 及帧计数，返回全部接收的字节数。 */
size_t mc02_board_uart_write(const uint8_t *data, size_t size)
{
    const uint8_t header[] = {0xA5, 0x5A, 0x44, 0x4D};
    const uint8_t footer[] = {0x00, 0x00, 0x80, 0x7F};
    assert(size == 72U && memcmp(data, header, 4U) == 0 && memcmp(data + 68U, footer, 4U) == 0);
    memcpy(s_wire, data, sizeof(s_wire));
    ++s_telemetry_count;
    return size;
}

/** 作用核对假 CAN 初始化参数并记录过滤 ID。
 * 输入port、bitrate_hz、非空 ids 和 count，输出初始化计数与过滤表，返回 true。 */
bool mc02_can_init(mc02_can_port_t port, uint32_t bitrate_hz, const uint16_t *ids, size_t count)
{
    assert(port == MC02_CAN_PORT_1 && bitrate_hz == 500000U && count <= MC02_CAN_FILTER_CAPACITY);
    ++s_init_count;
    s_filter_count = count;
    memcpy(s_filter_ids, ids, count * sizeof(*ids));
    return true;
}

/** 作用提供当前假 CAN 总线状态。
 * 输入测试端口 port，输出对应测试预设的 mc02_can_status_t。 */
mc02_can_status_t mc02_can_status(mc02_can_port_t port)
{
    assert(port == MC02_CAN_PORT_1);
    return s_bus;
}

/** 作用按测试接收游标提供下一帧假 CAN 反馈。
 * 输入port 和非空 frame，输出收到的 frame 并推进游标，成功返回 true，队空返回 false。 */
bool mc02_can_receive(mc02_can_port_t port, can_frame_t *frame)
{
    assert(port == MC02_CAN_PORT_1);
    if (s_rx_offset == s_rx_count) {
        return false;
    }
    *frame = s_rx[s_rx_offset++];
    return true;
}

/** 作用模拟丢弃当前假 CAN 接收积压。
 * 输入测试端口 port，输出推进至队尾的全局接收游标，无返回值。 */
void mc02_can_discard_rx(mc02_can_port_t port)
{
    assert(port == MC02_CAN_PORT_1);
    s_rx_offset = s_rx_count;
}

/** 作用模拟读取缓冲挂起标志期间发生发送完成的竞态。
 * 输入port 和全局测试标志，输出读取时的挂起值，并按预设更新完成标志。 */
bool mc02_can_tx_pending(mc02_can_port_t port)
{
    assert(port == MC02_CAN_PORT_1);
    const bool was_pending = s_tx_pending;
    if (s_finish_after_pending_read) {
        assert(was_pending);
        s_finish_after_pending_read = false;
        s_tx_pending = false;
        s_tx_transmitted = true;
    }
    return was_pending;
}

/** 作用读取假发送缓冲记录的最近一次成功发送标志。
 * 输入port 和非挂起的假缓冲，输出 s_tx_transmitted。 */
bool mc02_can_tx_transmitted(mc02_can_port_t port)
{
    assert(port == MC02_CAN_PORT_1 && !s_tx_pending);
    return s_tx_transmitted;
}

/** 作用模拟 CAN 硬件接受、忙或故障并保存接受的帧。
 * 输入port 和非空 frame，输出假发送缓冲状态及 mc02_can_tx_result_t。 */
mc02_can_tx_result_t mc02_can_submit(mc02_can_port_t port, const can_frame_t *frame)
{
    assert(port == MC02_CAN_PORT_1);
    if (s_fail_submit) {
        return MC02_CAN_TX_FAULT;
    }
    if (s_tx_pending) {
        return MC02_CAN_TX_BUSY;
    }
    s_tx = *frame;
    s_has_tx = true;
    s_tx_pending = true;
    s_tx_transmitted = false;
    return MC02_CAN_TX_SUBMITTED;
}

/** 作用记录针对假挂起缓冲的撤销请求。
 * 输入port 和全局挂起状态，输出递增的取消计数，无返回值。 */
void mc02_can_cancel_tx(mc02_can_port_t port)
{
    assert(port == MC02_CAN_PORT_1 && s_tx_pending);
    ++s_cancel_count;
}

/** 作用验证 MIT 量程、ID、帧格式、编解码边界与原始反馈。
 * 输入无参数，使用测试常量，输出断言验证结果，无返回值，失败中止测试。 */
static void test_codec(void)
{
    damiao_mit_t motor;
    damiao_mit_feedback_t feedback = {0};
    assert(damiao_mit_init(&motor, &s_motor));
    const can_frame_t good = {
        .id = 0x11U,
        .rx_time_us = 1000U,
        .length = 8U,
        .data = {0x11, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 42, 57},
    };
    const can_frame_t invalid[] = {
        {.id = 1U, .length = 8U, .data = {0x11}},
        {.id = 0x11U, .length = 7U, .data = {0x11}},
        {.id = 0x11U, .length = 9U, .data = {0x11}},
        {.id = 0x11U, .length = 8U, .data = {0x11}, .is_extended = true},
        {.id = 0x11U, .length = 8U, .data = {0x11}, .is_remote = true},
        {.id = 0x11U, .length = 8U, .data = {0x11}, .is_fd = true},
        {.id = 0x11U, .length = 8U, .data = {0x12}},
    };
    for (size_t i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        assert(!damiao_mit_decode_feedback(&motor, &invalid[i], &feedback));
    }
    assert(damiao_mit_decode_feedback(&motor, &good, &feedback));
    assert(feedback.position_rad == 12.5 && feedback.speed_rad_s == -10.0 &&
           feedback.torque_n_m == 6.0);
    assert(feedback.mos_deg_c == 42U && feedback.rotor_deg_c == 57U);
    can_frame_t frame = good;
    frame.rx_time_us = 999U;
    assert(damiao_mit_decode_feedback(&motor, &frame, &feedback));
    assert(feedback.time_us == 999U);
    frame.rx_time_us = 50999U;
    frame.length = 7U;
    assert(!damiao_mit_decode_feedback(&motor, &frame, &feedback));
    frame = good;
    frame.data[0] = 0xD1U;
    frame.rx_time_us = 51001U;
    assert(damiao_mit_decode_feedback(&motor, &frame, &feedback));

    damiao_mit_command_t command = {0};
    const uint8_t zero[] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
    assert(damiao_mit_encode_command(&motor, &command, &frame));
    assert(frame.id == 1U && frame.length == 8U && memcmp(zero, frame.data, 8U) == 0);
    command = (damiao_mit_command_t){-12.5, 10.0, 250.0, 2.5, 6.0};
    const uint8_t mixed[] = {0x00, 0x00, 0xFF, 0xF7, 0xFF, 0x7F, 0xFF, 0xFF};
    assert(damiao_mit_encode_command(&motor, &command, &frame));
    assert(memcmp(mixed, frame.data, 8U) == 0);
    command.speed_rad_s = 10.01;
    assert(!damiao_mit_encode_command(&motor, &command, &frame));
    command.speed_rad_s = NAN;
    assert(!damiao_mit_encode_command(&motor, &command, &frame));
    command = (damiao_mit_command_t){12.5, 10.0, 500.0, 5.0, 6.0};
    assert(!damiao_mit_encode_command(&motor, &command, &frame));
    assert(damiao_mit_encode_action(&motor, DAMIAO_MIT_READ_STATUS, &frame));
    const uint8_t query[] = {1, 0, 0xCC, 0, 0, 0, 0, 0};
    assert(frame.id == 0x7FFU && frame.length == 8U && memcmp(query, frame.data, 8U) == 0);

    damiao_mit_config_t unscaled = s_motor;
    unscaled.speed_max_rad_s = 0.0;
    assert(damiao_mit_init(&motor, &unscaled) && !motor.has_valid_ranges);
    assert(damiao_mit_decode_feedback(&motor, &good, &feedback));
    command = (damiao_mit_command_t){0};
    assert(!damiao_mit_encode_command(&motor, &command, &frame));
}

/** 作用把指定设备状态帧追加到测试接收队列。
 * 输入反馈 id、motor_id、state 和 time_us，输出全局接收队列及计数，无返回值。 */
static void receive(uint16_t id, uint8_t motor_id, uint8_t state, uint64_t time_us)
{
    assert(s_rx_count < MC02_CAN_RX_DEPTH);
    s_rx[s_rx_count++] = (can_frame_t){
        .id = id,
        .rx_time_us = time_us,
        .length = 8U,
        .data = {(uint8_t)(16U * state + motor_id)},
    };
}

/** 作用按指定模式装配主机台架并绑定测试字节源。
 * 输入控制 mode，输出初始化的 s_control 和假总线过滤配置，无返回值。 */
static void init_control(speed_controller_mode_t mode)
{
    frame_config_t config = s_control_config;
    config.controller.mode = mode;
    motor_bench_init(&s_control, &config, 0U);
    /* 主机直接提供字节源，反馈模式仍由组件拒绝运动。 */
    s_control.read_byte = mc02_board_uart_read;
    assert(s_filter_count == 1U && s_filter_ids[0] == 0x11U);
}

/** 作用推进一轮台架接收、姿态、操作和命令处理并模拟相邻轮次发送完成。
 * 输入now_us、state、command、pitch_rad 和 is_fresh，输出全局测试状态及待核对发送帧，无返回值。 */
static void poll(uint64_t now_us, int state, const char *command, double pitch_rad, bool is_fresh)
{
    s_now_us = now_us;
    s_command = command;
    s_has_tx = false;
    s_rx_count = 0U;
    s_rx_offset = 0U;
    if (state >= 0) {
        receive(0x11U, 1U, (uint8_t)state, now_us - 100U);
    }
    const attitude_sample_t pose = {
        .attitude =
            {
                .q_odom_from_body = {cos(pitch_rad / 2.0), 0.0, sin(pitch_rad / 2.0), 0.0},
                .status = ATTITUDE_STATUS_OK,
                .is_initialized = true,
            },
        .sample_time_us = is_fresh ? now_us : now_us - 25000U,
        .is_sample_valid = true,
    };
    can_service_poll_rx(&s_control.can, now_us);
    motor_bench_update(&s_control, &pose, now_us);
    /* 单电机回归中，发送在相邻轮次之间完成。 */
    s_tx_transmitted = s_has_tx;
    s_tx_pending = false;
}

/** 作用断言最新发送帧是指定的达妙特殊动作。
 * 输入动作尾字节 action 和全局 s_tx，输出断言结果，无返回值，失败中止测试。 */
static void expect_action(uint8_t action)
{
    const uint8_t prefix[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    assert(s_has_tx && s_tx.id == 1U && s_tx.length == 8U);
    assert(memcmp(s_tx.data, prefix, 7U) == 0 && s_tx.data[7] == action);
}

/** 作用断言指定电机的最新 MIT 速度命令及固定增益字段。
 * 输入motor_id、speed_rad_s 和全局 s_tx，输出断言结果，无返回值。 */
static void expect_motor_speed(uint16_t motor_id, double speed_rad_s)
{
    assert(s_has_tx && s_tx.id == motor_id && s_tx.length == 8U);
    const unsigned int raw = ((unsigned int)s_tx.data[2] << 4) | (s_tx.data[3] >> 4);
    assert(raw == (unsigned int)((speed_rad_s + 10.0) / 20.0 * 4095.0));
    assert((s_tx.data[3] & 0xFU) == 0U && s_tx.data[4] == 0U);
    assert(s_tx.data[5] == 5U && s_tx.data[6] == 0x17U && s_tx.data[7] == 0xFFU);
}

/** 作用按测试第一颗电机的协议字段核对目标速度。
 * 输入speed_rad_s（rad/s）和全局发送帧，输出断言结果，无返回值。 */
static void expect_speed(double speed_rad_s)
{
    expect_motor_speed(1U, speed_rad_s);
}

/** 作用验证定速或倾角目标、启停、反馈过期、故障和总线恢复。
 * 输入is_fixed 选择模式，输出全局测试状态及协议断言结果，无返回值。 */
static void test_motion(bool is_fixed)
{
    init_control(is_fixed ? SPEED_CONTROLLER_FIXED_SPEED : SPEED_CONTROLLER_PITCH_SPEED);
    poll(1000U, -1, "arm\n", 0.3, true);
    expect_action(0xFDU);
    poll(2000U, 0, "", 0.3, true);
    assert(!s_has_tx);
    poll(2500U, 0, "xarm\n", 0.3, true);
    assert(!s_has_tx);
    poll(3000U, 0, "arm\r\n", 0.3, true);
    expect_action(0xFCU);
    poll(4000U, -1, "run\n", 0.3, true);
    assert(!s_has_tx);
    poll(5000U, 1, "", 0.3, true);
    expect_speed(0.0);
    poll(15000U, 1, "run\n", 0.02, true);
    expect_speed(is_fixed ? 0.01 : 0.0);
    poll(25000U, 1, "", 0.3, true);
    expect_speed(is_fixed ? 0.02 : 0.01);
    poll(35000U, 1, "", -0.3, true);
    expect_speed(is_fixed ? 0.03 : 0.0);
    poll(45000U, 1, "", -0.3, true);
    expect_speed(is_fixed ? 0.04 : -0.01);
    poll(55000U, 1, "zero\n", -0.3, true);
    expect_speed(is_fixed ? 0.02 : 0.0);
    poll(65000U, 1, is_fixed ? "disable\n" : "", 0.3, false);
    expect_action(0xFDU);
    assert(s_control.motor.output.state == DAMIAO_MOTOR_STOPPING);

    poll(67000U, 0, "", 0.3, true);
    poll(68000U, 0, "run\n", 0.3, true);
    assert(!s_has_tx);
    poll(69000U, 0, "arm\n", 0.3, true);
    expect_action(0xFCU);
    poll(70000U, 1, "", 0.3, true);
    expect_speed(0.0);
    poll(71000U, 0xD, "", 0.3, true);
    expect_action(0xFDU);
    poll(72000U, 0, "", 0.3, true);
    poll(73000U, 0, "arm\n", 0.3, true);
    expect_action(0xFCU);
    poll(74000U, 1, "", 0.3, true);
    expect_speed(0.0);
    for (uint64_t time_us = 84000U; time_us < 124000U; time_us += 10000U) {
        poll(time_us, -1, "", 0.3, true);
        expect_speed(0.0);
    }
    poll(124000U, -1, "", 0.3, true);
    expect_action(0xFDU);
    poll(125000U, 0, "", 0.3, true);
    poll(126000U, 0, "arm\n", 0.3, true);
    expect_action(0xFCU);
    poll(127000U, 1, "", 0.3, true);
    expect_speed(0.0);
    s_bus = MC02_CAN_BUS_OFF;
    poll(128000U, -1, "", 0.3, true);
    assert(!s_has_tx && s_control.motor.output.state == DAMIAO_MOTOR_STOPPING);
    s_bus = MC02_CAN_OK;
    poll(129000U, -1, "run\n", 0.3, true);
    expect_action(0xFDU);
    poll(139000U, -1, "", 0.3, true);
    assert(s_has_tx && s_tx.id == 0x7FFU && s_tx.data[2] == 0xCCU);
    poll(140000U, 0, "", 0.3, true);
    poll(141000U, 0, "run\n", 0.3, true);
    assert(!s_has_tx);
}

/** 作用验证失能和使能确认只接受硬件提交之后的新鲜反馈。
 * 输入无参数，控制假发送缓冲与反馈，输出确认状态和发送序列断言结果，无返回值。 */
static void test_submission_confirmation(void)
{
    init_control(SPEED_CONTROLLER_FIXED_SPEED);
    s_tx_pending = true;
    poll(1000U, -1, "", 0.0, true);
    assert(!s_has_tx && s_control.motor.output.state == DAMIAO_MOTOR_STOPPING);
    poll(2000U, 0, "", 0.0, true);
    expect_action(0xFDU);
    poll(2100U, -1, "", 0.0, true);
    assert(!s_has_tx && s_control.motor.output.state == DAMIAO_MOTOR_STOPPING);
    poll(3000U, 0, "", 0.0, true);
    assert(s_control.motor.output.state == DAMIAO_MOTOR_DISARMED);

    s_tx_pending = true;
    poll(4000U, 0, "arm\n", 0.0, true);
    assert(!s_has_tx && s_control.motor.output.state == DAMIAO_MOTOR_ARMING);
    poll(5000U, 1, "", 0.0, true);
    expect_action(0xFCU);
    poll(6000U, -1, "", 0.0, true);
    assert(!s_has_tx && s_control.motor.output.state == DAMIAO_MOTOR_ARMING);
    poll(7000U, 1, "", 0.0, true);
    expect_speed(0.0);
    assert(s_control.motor.output.state == DAMIAO_MOTOR_ARMED);
}

/** 作用验证重复反馈 ID、过滤容量和重复注册会阻止错误总线启动。
 * 输入无参数，使用局部实例和假总线，输出注册及启动断言结果，无返回值。 */
static void test_registration(void)
{
    can_service_t bus;
    damiao_motor_t motors[MC02_CAN_FILTER_CAPACITY + 1U] = {0};
    can_service_init(&bus, MC02_CAN_PORT_1);
    assert(damiao_motor_init(&motors[0], &bus, &s_motor));
    assert(!damiao_motor_init(&motors[1], &bus, &s_motor));
    assert(!can_service_start(&bus, 500000U, 0U));
    assert(s_init_count == 0U);

    can_service_init(&bus, MC02_CAN_PORT_1);
    motors[0] = (damiao_motor_t){0};
    for (size_t i = 0U; i < MC02_CAN_FILTER_CAPACITY; ++i) {
        damiao_mit_config_t config = s_motor;
        config.motor_id += (uint16_t)i;
        config.master_id += (uint16_t)i;
        assert(damiao_motor_init(&motors[i], &bus, &config));
    }
    damiao_mit_config_t overflow = s_motor;
    overflow.master_id = 0x30U;
    assert(!damiao_motor_init(&motors[MC02_CAN_FILTER_CAPACITY], &bus, &overflow));
    assert(!can_service_start(&bus, 500000U, 0U));
    assert(s_init_count == 0U);
}

/** 作用验证共享 CAN 的两颗直接设备在反馈、目标、提交、取消和故障时互不串扰。
 * 输入无参数，使用两颗局部电机与假总线，输出隔离及发送状态断言结果，无返回值。 */
static void test_two_motors(void)
{
    test_registration();
    can_service_t bus;
    damiao_motor_t first = {0};
    damiao_motor_t second = {0};
    damiao_mit_config_t second_config = s_motor;
    second_config.motor_id = 2U;
    second_config.master_id = 0x12U;
    can_service_init(&bus, MC02_CAN_PORT_1);
    assert(damiao_motor_init(&first, &bus, &s_motor));
    assert(damiao_motor_init(&second, &bus, &second_config));
    assert(!damiao_motor_init(&first, &bus, &second_config));
    assert(damiao_motor_motion_config(&first)->motor_id == 1U);
    assert(damiao_motor_motion_config(&second)->motor_id == 2U);
    assert(!s_has_tx && s_init_count == 0U);
    assert(can_service_start(&bus, 500000U, 0U));
    assert(!can_service_start(&bus, 500000U, 0U));
    assert(s_init_count == 1U && s_filter_count == 2U && s_filter_ids[0] == 0x11U &&
           s_filter_ids[1] == 0x12U);

    const damiao_mit_command_t second_target = {.speed_rad_s = -0.5, .kd_n_m_s_per_rad = 0.1};
    assert(damiao_motor_command(&second, &second_target, 0U, 11000U) == CAN_SERVICE_INVALID);
    receive(0x12U, 2U, 1U, 500U);
    receive(0x11U, 1U, 1U, 600U);
    receive(0x12U, 1U, 0xDU, 700U); /* 反馈地址正确，帧内设备 ID 错误。 */
    receive(0x12U, 2U, 1U, 800U);
    s_now_us = 1000U;
    can_service_poll_rx(&bus, s_now_us);
    assert(damiao_motor_output(&first, s_now_us).feedback.time_us == 600U);
    assert(damiao_motor_output(&second, s_now_us).feedback.time_us == 800U);
    assert(damiao_motor_output(&second, s_now_us).fault_count == 0U);

    s_rx_count = 0U;
    s_rx_offset = 0U;
    receive(0x11U, 1U, 0xDU, 599U); /* 设备拒绝倒序反馈。 */
    can_service_poll_rx(&bus, s_now_us);
    assert(damiao_motor_output(&first, s_now_us).feedback.time_us == 600U);
    assert(damiao_motor_output(&first, s_now_us).fault_count == 0U);

    damiao_mit_command_t command = {.speed_rad_s = 0.25, .kd_n_m_s_per_rad = 0.1};
    s_tx_pending = true;
    assert(damiao_motor_command(&first, &command, s_now_us, 11000U) == CAN_SERVICE_QUEUED);
    command.speed_rad_s = 0.5;
    assert(damiao_motor_command(&first, &command, s_now_us, 11000U) == CAN_SERVICE_QUEUED);
    assert(damiao_motor_command(&second, &second_target, s_now_us, 11000U) == CAN_SERVICE_QUEUED);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_BUSY);
    assert(damiao_motor_output(&first, s_now_us).submission.sequence == 0U);
    assert(damiao_motor_output(&second, s_now_us).submission.sequence == 0U);

    s_tx_pending = false;
    s_now_us = 2000U;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED);
    expect_speed(0.5);
    const damiao_motor_output_t first_submitted = damiao_motor_output(&first, s_now_us);
    assert(first_submitted.submission.time_us == 2000U);
    assert(first_submitted.transmission.kind == CAN_SERVICE_TX_NONE);
    /* 缓冲在读取 pending 之后才空闲，服务要先回收发送结果。 */
    s_finish_after_pending_read = true;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_BUSY);
    assert(damiao_motor_output(&second, s_now_us).submission.kind == CAN_SERVICE_TX_NONE);
    assert(damiao_motor_action(&second, DAMIAO_MIT_DISABLE, UINT64_MAX) == CAN_SERVICE_QUEUED);
    assert(s_cancel_count == 0U);
    assert(damiao_motor_command(&second, &command, s_now_us, 11000U) == CAN_SERVICE_BUSY);
    s_tx_pending = false;
    s_now_us = 3000U;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED);
    assert(s_tx.id == 2U && s_tx.data[7] == 0xFDU);
    assert(damiao_motor_output(&second, s_now_us).submission.time_us == 3000U);
    const can_service_transmission_t first_sent =
        damiao_motor_output(&first, s_now_us).transmission;
    assert(first_sent.kind == CAN_SERVICE_TX_NORMAL);
    assert(first_sent.submission_sequence == first_submitted.submission.sequence);
    s_tx_pending = false;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_IDLE);

    command.speed_rad_s = 0.75;
    assert(damiao_motor_command(&first, &command, s_now_us, 11000U) == CAN_SERVICE_QUEUED);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED);
    assert(damiao_motor_command(&second, &second_target, s_now_us, 11000U) == CAN_SERVICE_QUEUED);
    assert(damiao_motor_action(&first, DAMIAO_MIT_DISABLE, UINT64_MAX) == CAN_SERVICE_QUEUED);
    assert(s_cancel_count == 1U && can_service_poll_tx(&bus) == CAN_SERVICE_BUSY);
    s_tx_pending = false;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED);
    expect_action(0xFDU);
    assert(damiao_motor_output(&first, s_now_us).transmission.submission_sequence ==
           first_sent.submission_sequence); /* 取消的运动帧没有发送确认。 */
    s_tx_pending = false;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED);
    expect_motor_speed(2U, -0.5);
    const damiao_motor_output_t second_submitted = damiao_motor_output(&second, s_now_us);
    assert(second_submitted.transmission.kind == CAN_SERVICE_TX_NONE);
    s_tx_pending = false;
    s_tx_transmitted = true;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_IDLE);
    const can_service_transmission_t second_sent =
        damiao_motor_output(&second, s_now_us).transmission;
    assert(second_sent.kind == CAN_SERVICE_TX_NORMAL);
    assert(second_sent.submission_sequence == second_submitted.submission.sequence);
    assert(damiao_motor_output(&first, s_now_us).transmission.submission_sequence ==
           first_sent.submission_sequence);

    /* A 的待发目标到期时，B 刚收到反馈且仍可发送。 */
    assert(damiao_motor_command(&first, &command, s_now_us, 60000U) == CAN_SERVICE_QUEUED);
    s_rx_count = 0U;
    s_rx_offset = 0U;
    receive(0x12U, 2U, 1U, 50500U);
    s_now_us = 50600U;
    can_service_poll_rx(&bus, s_now_us);
    assert(damiao_motor_output(&first, s_now_us).status == DAMIAO_MIT_FEEDBACK_STALE);
    assert(damiao_motor_output(&second, s_now_us).status == DAMIAO_MIT_FEEDBACK_ENABLED);
    assert(damiao_motor_command(&first, &command, s_now_us, 60000U) == CAN_SERVICE_INVALID);
    assert(damiao_motor_command(&second, &command, s_now_us, 60000U) == CAN_SERVICE_QUEUED);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_tx.id == 2U);
    s_tx_pending = false;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_IDLE);

    /* 总线故障清除两颗电机的旧目标，包括在途帧，恢复后只允许保留的失能。 */
    s_rx_count = 0U;
    s_rx_offset = 0U;
    receive(0x11U, 1U, 1U, 50900U);
    s_now_us = 51000U;
    can_service_poll_rx(&bus, s_now_us);
    assert(damiao_motor_command(&first, &command, s_now_us, 60000U) == CAN_SERVICE_QUEUED);
    assert(damiao_motor_command(&second, &command, s_now_us, 60000U) == CAN_SERVICE_QUEUED);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_tx.id == 1U);
    s_bus = MC02_CAN_BUS_OFF;
    can_service_poll_rx(&bus, ++s_now_us);
    assert(s_cancel_count == 2U && can_service_poll_tx(&bus) == CAN_SERVICE_BUS_FAULT);
    assert(damiao_motor_command(&second, &command, s_now_us, 60000U) == CAN_SERVICE_BUS_FAULT);
    assert(damiao_motor_action(&second, DAMIAO_MIT_DISABLE, UINT64_MAX) == CAN_SERVICE_QUEUED);
    s_bus = MC02_CAN_OK;
    s_tx_pending = false;
    can_service_poll_rx(&bus, ++s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_tx.id == 2U &&
           s_tx.data[7] == 0xFDU);
    s_tx_pending = false;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_IDLE);

    assert(damiao_motor_command(&first, &command, s_now_us, 60000U) == CAN_SERVICE_QUEUED);
    s_fail_submit = true;
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_BUS_FAULT);
    s_fail_submit = false;
    can_service_poll_rx(&bus, ++s_now_us);
    assert(damiao_motor_output(&first, s_now_us).bus_status == MC02_CAN_NOT_READY);
    can_service_poll_rx(&bus, ++s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_IDLE);
}

/** 作用核对电机遥测的周期、帧头尾及全部 16 路字段。
 * 输入无参数，使用固定观测和目标，输出假串口帧及逐通道断言结果，无返回值。 */
static void test_motor_telemetry(void)
{
    motor_telemetry_t telemetry;
    motor_telemetry_init(&telemetry, 1000U, 20000U, mc02_board_uart_write);
    const damiao_motor_output_t motor = {
        .feedback =
            {
                .speed_rad_s = 2.0,
                .state_raw = 13U,
                .position_rad = 1.25,
                .torque_n_m = -2.0,
                .mos_deg_c = 42U,
                .rotor_deg_c = 57U,
            },
        .status = DAMIAO_MIT_FEEDBACK_FAULT,
        .bus_status = MC02_CAN_ERROR_PASSIVE,
        .has_scaled_feedback = true,
        .state = DAMIAO_MOTOR_ARMED,
    };
    const speed_controller_output_t control = {
        .pitch_rad = 0.125,
        .target_speed_rad_s = -0.5,
        .is_pitch_valid = true,
    };
    const attitude_sample_t pose = {
        .attitude =
            {
                .q_odom_from_body = {cos(0.125 / 2.0), 0.0, sin(0.125 / 2.0), 0.0},
                .status = ATTITUDE_STATUS_OK,
                .is_initialized = true,
            },
        .is_sample_valid = true,
    };
    const imu_observer_output_t imu = {.sample_dt_s = 0.005, .missed_samples = 3U};
    motor_telemetry_update(&telemetry, &motor, &control, &imu, &pose, 999U);
    assert(s_telemetry_count == 0U);
    motor_telemetry_update(&telemetry, &motor, &control, &imu, &pose, 1000U);
    motor_telemetry_update(&telemetry, &motor, &control, &imu, &pose, 20999U);
    assert(s_telemetry_count == 1U);
    const float expected[] = {0.125f, -0.5f, 2.0f,   4.0f, 1.0f,  13.0f, 3.0f,  3.0f,
                              1.0f,   1.0f,  0.005f, 3.0f, 1.25f, -2.0f, 42.0f, 57.0f};
    for (size_t i = 0U; i < sizeof(expected) / sizeof(expected[0]); ++i) {
        const uint8_t *bytes = &s_wire[4U + 4U * i];
        const uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                              ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
        float value;
        memcpy(&value, &bits, sizeof(value));
        assert(value == expected[i]);
    }
}

/** 作用执行所选电机台架回归并核对 MIT 与遥测行为。
 * 输入argc/argv 指定 feedback、fixed、pitch、submission 或 two，输出断言与通过信息，成功返回 0。 */
int main(int argc, char *argv[])
{
    assert(argc == 2);
    test_codec();
    if (strcmp(argv[1], "feedback") == 0) {
        init_control(SPEED_CONTROLLER_FEEDBACK);
        poll(1000U, -1, "arm\n", 0.0, true);
        assert(s_has_tx && s_tx.id == 0x7FFU && s_tx.data[2] == 0xCCU);
        poll(2000U, 0, "run\n", 0.0, true);
        assert(!s_has_tx);
    } else if (strcmp(argv[1], "two") == 0) {
        test_two_motors();
        puts("MIT codec and two independent device instances passed");
        return 0;
    } else if (strcmp(argv[1], "submission") == 0) {
        test_submission_confirmation();
    } else {
        assert(strcmp(argv[1], "fixed") == 0 || strcmp(argv[1], "pitch") == 0);
        test_motion(strcmp(argv[1], "fixed") == 0);
    }
    test_motor_telemetry();
    printf("MIT codec and %s bench checks passed\n", argv[1]);
    return 0;
}
