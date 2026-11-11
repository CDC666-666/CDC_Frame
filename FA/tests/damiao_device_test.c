/* 只链接达妙设备、MIT 和共享 CAN 服务，验证两个受保护执行实例。 */
#include <assert.h>
#include <stdio.h>

#include "damiao_motor.h"

static uint64_t s_now_us;
static can_frame_t s_rx[2];
static size_t s_rx_offset;
static can_frame_t s_sent;
static bool s_pending;

/** 作用提供由测试控制的假板级时钟。
 * 输入无参数，读取全局 s_now_us，输出当前假时间（微秒）。 */
uint64_t mc02_board_time_us(void)
{
    return s_now_us;
}

/** 作用核对独立设备测试使用的端口、波特率和两个反馈 ID。
 * 输入port、bitrate_hz、ids 和 count，输出参数断言结果并返回 true。 */
bool mc02_can_init(mc02_can_port_t port, uint32_t bitrate_hz, const uint16_t *ids, size_t count)
{
    assert(port == MC02_CAN_PORT_1 && bitrate_hz == 500000U);
    assert(count == 2U && ids[0] == 0x11U && ids[1] == 0x12U);
    return true;
}

/** 作用为独立设备测试提供健康总线状态。
 * 输入port 为接口占位输入，输出 MC02_CAN_OK。 */
mc02_can_status_t mc02_can_status(mc02_can_port_t port)
{
    (void)port;
    return MC02_CAN_OK;
}

/** 作用按测试接收游标提供下一帧假 CAN 反馈。
 * 输入port 和非空 frame，输出收到的 frame 并推进游标，成功返回 true，队空返回 false。 */
bool mc02_can_receive(mc02_can_port_t port, can_frame_t *frame)
{
    (void)port;
    if (s_rx_offset == 2U) {
        return false;
    }
    *frame = s_rx[s_rx_offset++];
    return true;
}

/** 作用模拟丢弃当前假 CAN 接收积压。
 * 输入测试端口 port，输出推进至队尾的全局接收游标，无返回值。 */
void mc02_can_discard_rx(mc02_can_port_t port)
{
    (void)port;
    s_rx_offset = 2U;
}

/** 作用读取独立设备测试的假发送缓冲是否挂起。
 * 输入port 为接口占位输入，输出 s_pending。 */
bool mc02_can_tx_pending(mc02_can_port_t port)
{
    (void)port;
    return s_pending;
}

/** 作用模拟假发送缓冲空闲时传输已经成功完成。
 * 输入port 为接口占位输入，输出 s_pending 的反值。 */
bool mc02_can_tx_transmitted(mc02_can_port_t port)
{
    (void)port;
    return !s_pending;
}

/** 作用模拟立即清除假发送缓冲的挂起标志。
 * 输入port 为接口占位输入，输出 s_pending 为 false，无返回值。 */
void mc02_can_cancel_tx(mc02_can_port_t port)
{
    (void)port;
    s_pending = false;
}

/** 作用模拟单发送缓冲接受一帧或报告忙。
 * 输入port 和非空 frame，输出保存的发送帧与挂起标志，返回 SUBMITTED 或 BUSY。 */
mc02_can_tx_result_t mc02_can_submit(mc02_can_port_t port, const can_frame_t *frame)
{
    (void)port;
    if (s_pending) {
        return MC02_CAN_TX_BUSY;
    }
    s_pending = true;
    s_sent = *frame;
    return MC02_CAN_TX_SUBMITTED;
}

/** 作用推进假时钟并向两颗设备投递带时间戳的反馈。
 * 输入bus、first、second 及两设备原始状态，输出总线与设备反馈状态，无返回值。 */
static void feedback(can_service_t *bus, damiao_motor_t *first, damiao_motor_t *second,
                     uint8_t first_state, uint8_t second_state)
{
    s_now_us += 1000U;
    s_pending = false;
    s_rx_offset = 0U;
    s_rx[0] = (can_frame_t){.id = 0x11U,
                            .rx_time_us = s_now_us - 100U,
                            .length = 8U,
                            .data = {(uint8_t)(16U * first_state + 1U)}};
    s_rx[1] = (can_frame_t){.id = 0x12U,
                            .rx_time_us = s_now_us - 100U,
                            .length = 8U,
                            .data = {(uint8_t)(16U * second_state + 2U)}};
    can_service_poll_rx(bus, s_now_us);
    damiao_motor_update_feedback(first, s_now_us);
    damiao_motor_update_feedback(second, s_now_us);
}

/** 作用验证设备有持续失能反馈时使能确认仍会在截止时间到期。
 * 输入无参数，使用两颗局部设备和假 CAN，输出 STOPPING 与失能帧断言结果，无返回值。 */
static void test_enable_timeout(void)
{
    can_service_t bus;
    damiao_motor_t first = {0};
    damiao_motor_t second = {0};
    damiao_mit_config_t config = {1U, 0x11U, 12.5, 10.0, 6.0, 50000U};
    const damiao_motor_execution_config_t execution = {100000U, 20000U, 20000U, 0.1, true};
    s_now_us = 0U;
    s_pending = false;
    can_service_init(&bus, MC02_CAN_PORT_1);
    assert(damiao_motor_init(&first, &bus, &config));
    config.motor_id = 2U;
    config.master_id = 0x12U;
    assert(damiao_motor_init(&second, &bus, &config));
    assert(damiao_motor_configure_execution(&first, &execution, true, 10000U, 0U));
    assert(can_service_start(&bus, 500000U, 0U));
    const damiao_motor_input_t idle = {0};
    feedback(&bus, &first, &second, 0U, 0U);
    damiao_motor_update_command(&first, &idle, s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED);
    feedback(&bus, &first, &second, 0U, 0U);
    damiao_motor_update_command(&first, &idle, s_now_us);
    assert(first.output.state == DAMIAO_MOTOR_DISARMED);
    feedback(&bus, &first, &second, 0U, 0U);
    const damiao_motor_input_t enable = {.should_enable = true};
    damiao_motor_update_command(&first, &enable, s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_sent.data[7] == 0xFCU);
    do {
        feedback(&bus, &first, &second, 0U, 0U);
        damiao_motor_update_command(&first, &idle, s_now_us);
        (void)can_service_poll_tx(&bus);
    } while (first.output.state == DAMIAO_MOTOR_ARMING);
    assert(s_now_us == 103000U);
    assert(first.output.state == DAMIAO_MOTOR_STOPPING && s_sent.data[7] == 0xFDU);
    assert(second.output.state == DAMIAO_MOTOR_DISARMED);
}

/** 作用验证受保护设备的双实例隔离、确认、过期目标、故障和使能超时。
 * 输入无参数，使用假 CAN 与局部配置，输出断言和通过信息，成功返回 0。 */
int main(void)
{
    can_service_t bus;
    damiao_motor_t first = {0};
    damiao_motor_t second = {0};
    damiao_mit_config_t config = {1U, 0x11U, 12.5, 10.0, 6.0, 50000U};
    const damiao_motor_execution_config_t execution = {100000U, 20000U, 20000U, 0.1, true};
    can_service_init(&bus, MC02_CAN_PORT_1);
    assert(damiao_motor_init(&first, &bus, &config));
    config.motor_id = 2U;
    config.master_id = 0x12U;
    assert(damiao_motor_init(&second, &bus, &config));
    assert(damiao_motor_configure_execution(&first, &execution, true, 10000U, 0U));
    assert(damiao_motor_configure_execution(&second, &execution, true, 10000U, 0U));
    assert(!s_pending && first.endpoint->pending_kind == CAN_SERVICE_TX_NONE);
    assert(can_service_start(&bus, 500000U, 0U));
    const damiao_motor_input_t idle = {0};
    feedback(&bus, &first, &second, 0U, 0U);
    damiao_motor_update_command(&first, &idle, s_now_us);
    damiao_motor_update_command(&second, &idle, s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_sent.id == 1U);
    feedback(&bus, &first, &second, 0U, 0U);
    damiao_motor_update_command(&first, &idle, s_now_us);
    damiao_motor_update_command(&second, &idle, s_now_us);
    assert(first.output.state == DAMIAO_MOTOR_DISARMED);
    assert(second.output.state == DAMIAO_MOTOR_STOPPING);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_sent.id == 2U);
    feedback(&bus, &first, &second, 0U, 0U);
    damiao_motor_update_command(&second, &idle, s_now_us);
    assert(second.output.state == DAMIAO_MOTOR_DISARMED);

    const damiao_motor_input_t enable = {.should_enable = true};
    damiao_motor_update_command(&first, &enable, s_now_us);
    damiao_motor_update_command(&second, &enable, s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_sent.id == 1U);
    feedback(&bus, &first, &second, 1U, 1U);
    assert(first.output.state == DAMIAO_MOTOR_ARMED);
    assert(second.output.state == DAMIAO_MOTOR_ARMING);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_sent.id == 2U);
    feedback(&bus, &first, &second, 1U, 1U);
    assert(second.output.state == DAMIAO_MOTOR_ARMED);
    const damiao_motor_input_t target = {
        .target_speed_rad_s = -0.5,
        .target_expires_us = s_now_us + 10000U,
        .has_target = true,
    };
    damiao_motor_update_command(&second, &target, s_now_us);
    assert(can_service_poll_tx(&bus) == CAN_SERVICE_SUBMITTED && s_sent.id == 2U);
    const damiao_motor_input_t expired = {
        .target_expires_us = s_now_us,
        .has_target = true,
    };
    damiao_motor_update_command(&first, &expired, s_now_us);
    assert(first.output.state == DAMIAO_MOTOR_STOPPING);
    assert(second.output.state == DAMIAO_MOTOR_ARMED);
    s_now_us = target.target_expires_us;
    can_service_poll_rx(&bus, s_now_us);
    damiao_motor_update_feedback(&second, s_now_us);
    damiao_motor_update_command(&second, &idle, s_now_us);
    assert(second.output.state == DAMIAO_MOTOR_STOPPING); /* 连续轮询中的目标过期。 */
    feedback(&bus, &first, &second, 0U, 13U);
    assert(second.output.state == DAMIAO_MOTOR_STOPPING && second.output.fault_count == 1U);
    assert(first.output.fault_count == 0U);
    test_enable_timeout();
    puts("Two managed devices passed without controller or INS linkage");
    return 0;
}
