/* MC02 静态场景及主循环入口。 */
#include "frame_system.h"

#include "bmi088.h"
#include "frame_config.h"
#include "imu_observer.h"
#include "ins.h"
#include "mc02_board.h"

#ifdef FRAME_CDC_HAS_MOTOR_BENCH
#include "motor_bench.h"
#endif
#ifdef FRAME_CDC_HAS_MOTOR_TELEMETRY
#include "motor_telemetry.h"
#else
#include "imu_telemetry.h"
#endif

static bool s_board_is_ready;
static bmi088_t s_bmi088;
static imu_observer_t s_imu;
static ins_t s_ins;

#ifdef FRAME_CDC_HAS_MOTOR_BENCH
static motor_bench_t s_bench;
#endif

#ifdef FRAME_CDC_HAS_MOTOR_TELEMETRY
static motor_telemetry_t s_telemetry;
#else
static imu_telemetry_t s_telemetry;
#endif

/** 作用初始化 MC02 板级并装配静态组件，主循环前调用一次。
 * 输入无参数，读取生成配置，输出初始化后的内部实例和板级就绪状态，无返回值。 */
void frame_system_init(void)
{
    s_board_is_ready = mc02_board_init();
    if (!s_board_is_ready) {
        return;
    }
    bmi088_init(&s_bmi088);
    const uint64_t first_sample_us = mc02_board_time_us();
    imu_observer_init(&s_imu, &s_bmi088, first_sample_us, g_frame_config.imu.sample_period_us);
    ins_init(&s_ins);
#ifdef FRAME_CDC_HAS_MOTOR_BENCH
    motor_bench_init(&s_bench, &g_frame_config, mc02_board_time_us());
#endif
#ifdef FRAME_CDC_HAS_MOTOR_TELEMETRY
    motor_telemetry_init(&s_telemetry, first_sample_us, g_frame_config.telemetry.period_us,
                         mc02_board_uart_write);
#else
    imu_telemetry_init(&s_telemetry, first_sample_us, g_frame_config.telemetry.period_us,
                       mc02_board_uart_write);
#endif
}

/** 作用依次推进接收、观测、估计、控制、发送和遥测。
 * 输入无参数，读取内部实例与板级时钟，输出更新后的组件状态和通信请求，无返回值。 */
void frame_system_poll(void)
{
    if (!s_board_is_ready) {
        return;
    }
#ifdef FRAME_CDC_HAS_MOTOR_BENCH
    can_service_poll_rx(&s_bench.can, mc02_board_time_us());
#endif
    const uint64_t sample_now_us = mc02_board_time_us();
    if (imu_observer_update(&s_imu, sample_now_us)) {
        ins_update(&s_ins, &s_imu.output.observation);
    }
#ifdef FRAME_CDC_HAS_MOTOR_BENCH
    const uint64_t control_now_us = mc02_board_time_us();
    motor_bench_update(&s_bench, &s_ins.output, control_now_us);
#endif
#ifdef FRAME_CDC_HAS_MOTOR_TELEMETRY
    motor_telemetry_update(&s_telemetry, &s_bench.motor.output, &s_bench.controller.output,
                           &s_imu.output, &s_ins.output, control_now_us);
#else
    imu_telemetry_update(&s_telemetry, &s_imu.output, &s_ins.output, sample_now_us);
#endif
}
