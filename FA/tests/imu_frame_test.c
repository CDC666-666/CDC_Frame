/* 用确定性的 BMI088 样本和 UART FIFO 替身运行生产装配。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "frame_system.h"
#include "mc02_board.h"

enum { IMU_FRAME_SIZE = 92, FRAME_COUNT = 5 };

static uint64_t s_now_us;
static uint32_t s_last_slot;
static size_t s_wire_size;
static uint8_t s_wire[FRAME_COUNT * IMU_FRAME_SIZE];

/** 作用模拟板级初始化成功以运行主机装配。
 * 输入无参数，输出 true 表示假板级初始化成功。 */
bool mc02_board_init(void)
{
    return true;
}

/** 作用提供由测试控制的假板级时钟。
 * 输入无参数，读取全局 s_now_us，输出当前假时间（微秒）。 */
uint64_t mc02_board_time_us(void)
{
    return s_now_us;
}

/** 作用模拟每轮最多接收 23 字节的 UART 以覆盖遥测分次发送。
 * 输入非空 data 和 size，输出追加后的 s_wire 及长度，返回本轮接收字节数。 */
size_t mc02_board_uart_write(const uint8_t *data, size_t size)
{
    /* 一帧至少需要四轮主循环才能发送完。 */
    const size_t sent = size > 23U ? 23U : size;
    assert(s_wire_size + sent <= sizeof(s_wire));
    memcpy(s_wire + s_wire_size, data, sent);
    s_wire_size += sent;
    return sent;
}

/** 作用初始化 BMI088 主机替身并模拟初始化耗时。
 * 输入非空 device，输出清零的设备及推进后的假时钟，无返回值。 */
void bmi088_init(bmi088_t *device)
{
    *device = (bmi088_t){0};
    s_now_us += 1000U;
}

/** 作用生成确定性 IMU 样本并在指定时槽模拟采样失败。
 * 输入非空 device、time_us 和 sequence，输出假样本、有效性及槽记录，无返回值。 */
void bmi088_sample(bmi088_t *device, uint64_t time_us, uint32_t sequence)
{
    assert(sequence > s_last_slot);
    s_last_slot = sequence;
    if (sequence == 5U) {
        device->sample.is_valid = false;
        device->sample.status = BMI088_STATUS_NOT_READY;
        return;
    }
    device->sample = (bmi088_sample_t){
        .accel_sensor_m_s2 = {0.0, 0.0, 9.80665},
        .gyro_sensor_rad_s = {0.0, 0.0, sequence == 1U ? 0.0 : 0.2},
        .temperature_deg_c = 24.25,
        .time_us = time_us,
        .sequence = sequence,
        .is_valid = true,
    };
}

/** 作用从 IMU 帧中按小端 float32 读取指定通道。
 * 输入非空 frame 和从零开始的 index，输出对应通道的 float 数值。 */
static float read_channel(const uint8_t *frame, size_t index)
{
    const uint8_t *bytes = frame + 4U + 4U * index;
    const uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8U) |
                          ((uint32_t)bytes[2] << 16U) | ((uint32_t)bytes[3] << 24U);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/** 作用通过生产装配生成 IMU 回归帧并核对采样、丢样及遥测字段。
 * 输入argc/argv 指定输出文件路径，输出 460 字节帧文件和通过信息，成功返回 0。 */
int main(int argc, char *argv[])
{
    assert(argc == 2);
    frame_system_init();
    for (s_now_us = 1000U; s_now_us <= 121000U; s_now_us += 1000U) {
        /* 第一帧占用缓冲时跨过第二个遥测槽。随后跨过另一个遥测槽。 */
        if (s_now_us == 2000U) {
            s_now_us = 21000U;
        } else if (s_now_us == 45000U) {
            s_now_us = 81000U;
        }
        frame_system_poll();
    }
    /* 刷新最后一帧的剩余字节，不推进采样或遥测时钟。 */
    for (size_t i = 0U; i < 3U; ++i) {
        s_now_us = 121000U;
        frame_system_poll();
    }
    assert(s_wire_size == sizeof(s_wire));
    const float sequence[] = {1.0f, 9.0f, 17.0f, 21.0f, 25.0f};
    const float missed[] = {0.0f, 4.0f, 11.0f, 11.0f, 11.0f};
    const float dropped[] = {0.0f, 1.0f, 2.0f, 2.0f, 2.0f};
    const float status[] = {0.0f, 0.0f, 4.0f, 0.0f, 0.0f};
    for (size_t i = 0U; i < FRAME_COUNT; ++i) {
        const uint8_t *frame = s_wire + i * IMU_FRAME_SIZE;
        assert(memcmp(frame, "\xA5\x5A\x49\x4D", 4U) == 0);
        assert(memcmp(frame + IMU_FRAME_SIZE - 4U, "\x00\x00\x80\x7F", 4U) == 0);
        assert(read_channel(frame, 10U) == 24.25f);
        assert(read_channel(frame, 12U) == 1.0f);
        assert(read_channel(frame, 13U) == status[i]);
        assert(read_channel(frame, 15U) == sequence[i]);
        assert(read_channel(frame, 17U) == missed[i]);
        assert(read_channel(frame, 18U) == dropped[i]);
    }

    FILE *out = fopen(argv[1], "wb");
    assert(out != NULL);
    assert(fwrite(s_wire, 1U, sizeof(s_wire), out) == sizeof(s_wire));
    assert(fclose(out) == 0);
    printf("IMU frame checks passed (%zu bytes)\n", s_wire_size);
    return 0;
}
