#ifndef FRAME_CDC_PLATFORM_MC02_MC02_CAN_H
#define FRAME_CDC_PLATFORM_MC02_MC02_CAN_H

#include <stddef.h>

#include "can_frame.h"

#define MC02_CAN_RX_DEPTH 4U
#define MC02_CAN_FILTER_CAPACITY 8U
/* 1 Mbit/s 时，16 位硬件时间戳约 1.049 s 回绕。 */
#define MC02_CAN_TIMESTAMP_MAX_POLL_US 1000000U

typedef enum {
    MC02_CAN_PORT_1 = 0,
} mc02_can_port_t;

typedef enum {
    MC02_CAN_OK = 0,
    MC02_CAN_NOT_READY,
    MC02_CAN_WARNING,
    MC02_CAN_ERROR_PASSIVE,
    MC02_CAN_BUS_OFF,
    MC02_CAN_RX_LOST,
} mc02_can_status_t;

typedef enum {
    MC02_CAN_TX_SUBMITTED = 0,
    MC02_CAN_TX_BUSY,
    MC02_CAN_TX_FAULT,
    MC02_CAN_TX_INVALID,
} mc02_can_tx_result_t;

/** @brief 按列表精确过滤标准 ID。每条物理总线只启动一次，当前支持 CAN1。 */
bool mc02_can_init(mc02_can_port_t port, uint32_t bitrate_hz, const uint16_t *ids, size_t count);

/** @brief 主循环读取总线状态，并消费 FIFO 丢帧标志。 */
mc02_can_status_t mc02_can_status(mc02_can_port_t port);

/** @brief 主循环取一帧。按硬件 SOF 时间戳换算至板级微秒时钟。
 * @details frame 非空。轮询间断时，调用方须先丢弃积压帧。
 */
bool mc02_can_receive(mc02_can_port_t port, can_frame_t *frame);

/** @brief 丢弃当前积压帧。主循环调用，执行次数受 FIFO 容量限制。 */
void mc02_can_discard_rx(mc02_can_port_t port);

/** @brief 检查唯一发送缓冲是否仍挂起。硬件在请求撤销期间也可能保持挂起。 */
bool mc02_can_tx_pending(mc02_can_port_t port);

/** @brief 非阻塞提交 Classic 标准数据帧。已提交只表示硬件接受请求。 */
mc02_can_tx_result_t mc02_can_submit(mc02_can_port_t port, const can_frame_t *frame);

/** @brief 请求撤销挂起的帧，下一次发送前仍检查缓冲是否空闲。 */
void mc02_can_cancel_tx(mc02_can_port_t port);

#endif /* FRAME_CDC_PLATFORM_MC02_MC02_CAN_H */
