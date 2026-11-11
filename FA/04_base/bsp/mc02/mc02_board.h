#ifndef FRAME_CDC_BASE_BSP_MC02_MC02_BOARD_H
#define FRAME_CDC_BASE_BSP_MC02_MC02_BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MC02_BOARD_SPI_SELECT_0 = 0,
    MC02_BOARD_SPI_SELECT_1,
} mc02_board_spi_select_t;

/** @brief 初始化 TIM2、SPI2、UART7，并开启板卡的对外 5 V。
 * @return 返回 false 表示时钟配置不受支持。
 */
bool mc02_board_init(void);

/** @brief 返回开机初始化后的单调微秒数。主循环调用，相邻调用须小于 TIM2 回绕周期。 */
uint64_t mc02_board_time_us(void);

/** @brief 初始化阶段的阻塞延时，单位 ms。 */
void mc02_board_delay_ms(uint32_t delay_ms);

/** @brief 片选有效期间全双工传输，传输等待上限 300 us。仅主循环调用。
 * @param size 字节数，范围 1..32，tx 和 rx 均须至少有 size 字节且非空。
 * @return 成功返回 true。失败时释放片选并恢复 SPI2，rx 内容丢弃。
 */
bool mc02_board_spi_exchange(mc02_board_spi_select_t select, const uint8_t *tx, uint8_t *rx,
                             size_t size);

/** @brief 主循环取走并清除陀螺仪 DRDY 边沿标志。多个边沿合并为一个标志。 */
bool mc02_board_take_gyro_ready(void);

/** @brief 非阻塞填充 UART7 TX FIFO，返回已写入字节数。
 * @param data 主循环持有的 size 字节。返回后串口硬件只持有已拷入 FIFO 的字节。
 */
size_t mc02_board_uart_write(const uint8_t *data, size_t size);

/** @brief 启动 UART7 接收，须在板级初始化后由主循环调用。 */
void mc02_board_uart_enable_rx(void);

/** @brief 非阻塞读取一个字节。-1 表示空，-2 表示接收错误且已清空 RX FIFO。 */
int mc02_board_uart_read(void);

#endif /* FRAME_CDC_BASE_BSP_MC02_MC02_BOARD_H */
