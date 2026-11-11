/* MC02 V1.0 CAN1，轮询 FDCAN1 RX FIFO 和单个专用 TX 缓冲。 */
#include "mc02_can.h"

#include <string.h>

#include "stm32h7xx_hal.h"

#include "mc02_board.h"

#define MC02_CAN_KERNEL_HZ 24000000U
#define MC02_CAN_BIT_TQ 24U
#define MC02_CAN_TIMESTAMP_BITS 16U

static FDCAN_HandleTypeDef s_fdcan;
static uint32_t s_bitrate_hz;
static bool s_is_started;

bool mc02_can_init(mc02_can_port_t port, uint32_t bitrate_hz, const uint16_t *ids, size_t count)
{
    if (port != MC02_CAN_PORT_1 || s_is_started || bitrate_hz == 0U || bitrate_hz > 1000000U ||
        ids == NULL || count == 0U || count > MC02_CAN_FILTER_CAPACITY) {
        return false;
    }
    for (size_t i = 0U; i < count; ++i) {
        if (ids[i] > 0x7FFU) {
            return false;
        }
    }
    const uint32_t prescaler = MC02_CAN_KERNEL_HZ / (MC02_CAN_BIT_TQ * bitrate_hz);
    if (prescaler == 0U || prescaler > 512U ||
        prescaler * MC02_CAN_BIT_TQ * bitrate_hz != MC02_CAN_KERNEL_HZ) {
        return false;
    }

    /* 板载晶振为 24 MHz。单独启动 HSE，不切换 SYSCLK 或 PLL。 */
    RCC_OscInitTypeDef oscillator = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_ON,
        .PLL.PLLState = RCC_PLL_NONE,
    };
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        return false;
    }
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_HSE);
    if (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) != MC02_CAN_KERNEL_HZ) {
        return false;
    }
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_0 | GPIO_PIN_1,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF9_FDCAN1,
    };
    HAL_GPIO_Init(GPIOD, &pins);

    s_fdcan.Instance = FDCAN1;
    s_fdcan.Init = (FDCAN_InitTypeDef){
        .FrameFormat = FDCAN_FRAME_CLASSIC,
        .Mode = FDCAN_MODE_NORMAL,
        .AutoRetransmission = DISABLE,
        .NominalPrescaler = prescaler,
        .NominalSyncJumpWidth = 4U,
        .NominalTimeSeg1 = 17U,
        .NominalTimeSeg2 = 6U,
        .DataPrescaler = 1U,
        .DataSyncJumpWidth = 1U,
        .DataTimeSeg1 = 1U,
        .DataTimeSeg2 = 1U,
        .StdFiltersNbr = (uint32_t)count,
        .RxFifo0ElmtsNbr = MC02_CAN_RX_DEPTH,
        /* HAL 读取长度由 DLC 决定，给非预期 DLC 留足空间再过滤。 */
        .RxFifo0ElmtSize = FDCAN_DATA_BYTES_64,
        .TxBuffersNbr = 1U,
        .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
        .TxElmtSize = FDCAN_DATA_BYTES_8,
    };
    if (HAL_FDCAN_Init(&s_fdcan) != HAL_OK) {
        return false;
    }
    for (size_t i = 0U; i < count; ++i) {
        const FDCAN_FilterTypeDef filter = {
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = (uint32_t)i,
            .FilterType = FDCAN_FILTER_MASK,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = ids[i],
            .FilterID2 = 0x7FFU,
        };
        if (HAL_FDCAN_ConfigFilter(&s_fdcan, &filter) != HAL_OK) {
            return false;
        }
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&s_fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ConfigTimestampCounter(&s_fdcan, FDCAN_TIMESTAMP_PRESC_16) != HAL_OK ||
        HAL_FDCAN_EnableTimestampCounter(&s_fdcan, FDCAN_TIMESTAMP_INTERNAL) != HAL_OK ||
        HAL_FDCAN_Start(&s_fdcan) != HAL_OK) {
        return false;
    }
    s_bitrate_hz = bitrate_hz;
    s_is_started = true;
    return true;
}

mc02_can_status_t mc02_can_status(mc02_can_port_t port)
{
    FDCAN_ProtocolStatusTypeDef status;
    if (port != MC02_CAN_PORT_1 || !s_is_started ||
        HAL_FDCAN_GetProtocolStatus(&s_fdcan, &status) != HAL_OK) {
        return MC02_CAN_NOT_READY;
    }
    if (status.BusOff != 0U) {
        return MC02_CAN_BUS_OFF;
    }
    if (__HAL_FDCAN_GET_FLAG(&s_fdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST) != 0U) {
        __HAL_FDCAN_CLEAR_FLAG(&s_fdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        return MC02_CAN_RX_LOST;
    }
    if (status.ErrorPassive != 0U) {
        return MC02_CAN_ERROR_PASSIVE;
    }
    if (status.Warning != 0U) {
        return MC02_CAN_WARNING;
    }
    return MC02_CAN_OK;
}

bool mc02_can_receive(mc02_can_port_t port, can_frame_t *frame)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    if (port != MC02_CAN_PORT_1 || !s_is_started ||
        HAL_FDCAN_GetRxFifoFillLevel(&s_fdcan, FDCAN_RX_FIFO0) == 0U ||
        HAL_FDCAN_GetRxMessage(&s_fdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
        return false;
    }
    const uint64_t now_us = mc02_board_time_us();
    const uint16_t counter = HAL_FDCAN_GetTimestampCounter(&s_fdcan);
    const uint16_t age_ticks = (uint16_t)(counter - header.RxTimestamp);
    /* 留出一个硬件刻度，避免把提交前收到的帧估算到提交之后。 */
    const uint64_t age_us =
        ((uint64_t)age_ticks + 1U) * MC02_CAN_TIMESTAMP_BITS * 1000000U / s_bitrate_hz;
    *frame = (can_frame_t){
        .id = header.Identifier,
        .rx_time_us = age_us <= now_us ? now_us - age_us : 0U,
        .length = (uint8_t)header.DataLength,
        .is_extended = header.IdType == FDCAN_EXTENDED_ID,
        .is_remote = header.RxFrameType == FDCAN_REMOTE_FRAME,
        .is_fd = header.FDFormat == FDCAN_FD_CAN,
    };
    if (!frame->is_remote && frame->length <= sizeof(frame->data)) {
        memcpy(frame->data, data, frame->length);
    }
    return true;
}

void mc02_can_discard_rx(mc02_can_port_t port)
{
    can_frame_t frame;
    for (size_t i = 0U; i < MC02_CAN_RX_DEPTH && mc02_can_receive(port, &frame); ++i) {
        /* 主循环保持有界。 */
    }
}

bool mc02_can_tx_pending(mc02_can_port_t port)
{
    return port == MC02_CAN_PORT_1 && s_is_started &&
           HAL_FDCAN_IsTxBufferMessagePending(&s_fdcan, FDCAN_TX_BUFFER0) != 0U;
}

bool mc02_can_tx_transmitted(mc02_can_port_t port)
{
    return port == MC02_CAN_PORT_1 && s_is_started &&
           (s_fdcan.Instance->TXBTO & FDCAN_TX_BUFFER0) != 0U;
}

mc02_can_tx_result_t mc02_can_submit(mc02_can_port_t port, const can_frame_t *frame)
{
    if (port != MC02_CAN_PORT_1 || !s_is_started || (FDCAN1->PSR & FDCAN_PSR_BO) != 0U) {
        return MC02_CAN_TX_FAULT;
    }
    if (frame->id > 0x7FFU || frame->is_extended || frame->is_remote || frame->is_fd ||
        frame->length > sizeof(frame->data)) {
        return MC02_CAN_TX_INVALID;
    }
    if (mc02_can_tx_pending(port)) {
        return MC02_CAN_TX_BUSY;
    }
    const FDCAN_TxHeaderTypeDef header = {
        .Identifier = frame->id,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = frame->length,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    };
    if (HAL_FDCAN_AddMessageToTxBuffer(&s_fdcan, &header, frame->data, FDCAN_TX_BUFFER0) !=
            HAL_OK ||
        HAL_FDCAN_EnableTxBufferRequest(&s_fdcan, FDCAN_TX_BUFFER0) != HAL_OK) {
        return MC02_CAN_TX_FAULT;
    }
    return MC02_CAN_TX_SUBMITTED;
}

void mc02_can_cancel_tx(mc02_can_port_t port)
{
    if (mc02_can_tx_pending(port)) {
        /* 请求后由硬件处理，send 再次检查缓冲占用。 */
        (void)HAL_FDCAN_AbortTxRequest(&s_fdcan, FDCAN_TX_BUFFER0);
    }
}
