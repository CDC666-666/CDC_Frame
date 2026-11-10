/* 每条总线的过滤、反馈路由和单硬件缓冲发送仲裁。 */
#include "can_service.h"

#include "mc02_board.h"

void can_service_init(can_service_t *bus, mc02_can_port_t port)
{
    *bus = (can_service_t){.port = port, .status = MC02_CAN_NOT_READY};
}

can_service_endpoint_t *can_service_register(can_service_t *bus, uint16_t feedback_id,
                                             can_service_receive_fn receive, void *context)
{
    if (bus->is_started || bus->has_registration_error) {
        return NULL;
    }
    if (feedback_id > 0x7FFU || receive == NULL || bus->count >= MC02_CAN_FILTER_CAPACITY) {
        bus->has_registration_error = true;
        return NULL;
    }
    for (size_t i = 0U; i < bus->count; ++i) {
        if (bus->endpoints[i].feedback_id == feedback_id) {
            bus->has_registration_error = true;
            return NULL;
        }
    }

    can_service_endpoint_t *endpoint = &bus->endpoints[bus->count++];
    *endpoint = (can_service_endpoint_t){
        .bus = bus,
        .context = context,
        .receive = receive,
        .feedback_id = feedback_id,
    };
    return endpoint;
}

bool can_service_start(can_service_t *bus, uint32_t bitrate_hz, uint64_t now_us)
{
    if (bus->is_started || bus->has_registration_error || bus->count == 0U || bitrate_hz == 0U) {
        return false;
    }
    uint16_t ids[MC02_CAN_FILTER_CAPACITY];
    for (size_t i = 0U; i < bus->count; ++i) {
        ids[i] = bus->endpoints[i].feedback_id;
    }
    bus->is_started = mc02_can_init(bus->port, bitrate_hz, ids, bus->count);
    bus->last_poll_us = now_us;
    return bus->is_started;
}

static void cancel_inflight(can_service_t *bus)
{
    if (bus->inflight != NULL && !bus->is_cancelling && mc02_can_tx_pending(bus->port)) {
        mc02_can_cancel_tx(bus->port);
        bus->is_cancelling = true;
    }
}

static void drop_unsafe_tx(can_service_t *bus)
{
    for (size_t i = 0U; i < bus->count; ++i) {
        if (bus->endpoints[i].pending_kind != CAN_SERVICE_TX_DISABLE) {
            bus->endpoints[i].pending_kind = CAN_SERVICE_TX_NONE;
        }
    }
    if (bus->inflight_kind != CAN_SERVICE_TX_DISABLE) {
        cancel_inflight(bus);
    }
}

static void update_inflight(can_service_t *bus, uint64_t now_us)
{
    if (!mc02_can_tx_pending(bus->port)) {
        if (bus->inflight != NULL && mc02_can_tx_transmitted(bus->port)) {
            bus->inflight->last_transmission = (can_service_transmission_t){
                .kind = bus->inflight_kind,
                .submission_sequence = bus->inflight->last_submission.sequence,
            };
        }
        bus->inflight = NULL;
        bus->inflight_kind = CAN_SERVICE_TX_NONE;
        bus->is_cancelling = false;
    } else if (bus->inflight != NULL && bus->inflight_kind != CAN_SERVICE_TX_DISABLE &&
               now_us >= bus->inflight_expires_us) {
        cancel_inflight(bus);
    }
}

void can_service_poll_rx(can_service_t *bus, uint64_t now_us)
{
    if (!bus->is_started) {
        return;
    }
    update_inflight(bus, now_us);
    bus->status = mc02_can_status(bus->port);
    const bool has_rx_gap = now_us - bus->last_poll_us >= MC02_CAN_TIMESTAMP_MAX_POLL_US ||
                            bus->status == MC02_CAN_RX_LOST;
    bus->last_poll_us = now_us;
    if (bus->status == MC02_CAN_OK) {
        if (has_rx_gap) {
            bus->status = MC02_CAN_RX_LOST;
        } else if (bus->has_tx_fault) {
            bus->status = MC02_CAN_NOT_READY;
        }
    }
    bus->has_tx_fault = false;
    if (bus->status != MC02_CAN_OK) {
        drop_unsafe_tx(bus);
    }
    if (has_rx_gap) {
        mc02_can_discard_rx(bus->port);
        return;
    }

    can_frame_t frame;
    for (size_t i = 0U; i < MC02_CAN_RX_DEPTH && mc02_can_receive(bus->port, &frame); ++i) {
        for (size_t j = 0U; j < bus->count; ++j) {
            const can_service_endpoint_t *endpoint = &bus->endpoints[j];
            if (frame.id == endpoint->feedback_id) {
                endpoint->receive(endpoint->context, &frame);
                break;
            }
        }
    }
}

can_service_result_t can_service_queue(can_service_endpoint_t *endpoint, can_service_tx_kind_t kind,
                                       const can_frame_t *frame, uint64_t expires_us)
{
    if (endpoint == NULL || frame == NULL || kind <= CAN_SERVICE_TX_NONE ||
        kind > CAN_SERVICE_TX_DISABLE || frame->id > 0x7FFU || frame->is_extended ||
        frame->is_remote || frame->is_fd || frame->length > sizeof(frame->data)) {
        return CAN_SERVICE_INVALID;
    }
    can_service_t *bus = endpoint->bus;
    if (!bus->is_started || (bus->status != MC02_CAN_OK && kind != CAN_SERVICE_TX_DISABLE)) {
        return CAN_SERVICE_BUS_FAULT;
    }
    if (endpoint->pending_kind > kind) {
        return CAN_SERVICE_BUSY;
    }
    if (kind == CAN_SERVICE_TX_DISABLE && bus->inflight == endpoint &&
        bus->inflight_kind != CAN_SERVICE_TX_DISABLE) {
        cancel_inflight(bus);
    }
    endpoint->pending = *frame;
    endpoint->pending_kind = kind;
    endpoint->expires_us = expires_us;
    return CAN_SERVICE_QUEUED;
}

void can_service_cancel_motion(can_service_endpoint_t *endpoint)
{
    can_service_clear_queued(endpoint, CAN_SERVICE_TX_MOTION);
    can_service_t *bus = endpoint->bus;
    if (bus->inflight == endpoint && bus->inflight_kind == CAN_SERVICE_TX_MOTION) {
        cancel_inflight(bus);
    }
}

void can_service_clear_queued(can_service_endpoint_t *endpoint, can_service_tx_kind_t kind)
{
    if (endpoint->pending_kind == kind) {
        endpoint->pending_kind = CAN_SERVICE_TX_NONE;
    }
}

can_service_result_t can_service_poll_tx(can_service_t *bus)
{
    if (!bus->is_started || bus->status == MC02_CAN_BUS_OFF || bus->status == MC02_CAN_NOT_READY) {
        return CAN_SERVICE_BUS_FAULT;
    }
    const uint64_t now_us = mc02_board_time_us();
    update_inflight(bus, now_us);
    size_t selected = bus->count;
    for (size_t i = 0U; i < bus->count; ++i) {
        const size_t index = (bus->next_endpoint + i) % bus->count;
        can_service_endpoint_t *endpoint = &bus->endpoints[index];
        const bool is_disable = endpoint->pending_kind == CAN_SERVICE_TX_DISABLE;
        if (!is_disable && (now_us >= endpoint->expires_us || bus->status != MC02_CAN_OK)) {
            endpoint->pending_kind = CAN_SERVICE_TX_NONE;
        }
        if (endpoint->pending_kind != CAN_SERVICE_TX_NONE &&
            (selected == bus->count || is_disable)) {
            selected = index;
            if (is_disable) {
                break;
            }
        }
    }
    if (selected == bus->count) {
        return bus->status == MC02_CAN_OK ? CAN_SERVICE_IDLE : CAN_SERVICE_BUS_FAULT;
    }

    /* 等下一轮回收在途结果，避免新请求清除尚未读取的成功标志。 */
    if (bus->inflight != NULL) {
        return CAN_SERVICE_BUSY;
    }
    can_service_endpoint_t *endpoint = &bus->endpoints[selected];
    switch (mc02_can_submit(bus->port, &endpoint->pending)) {
        case MC02_CAN_TX_BUSY:
            return CAN_SERVICE_BUSY;
        case MC02_CAN_TX_FAULT:
            bus->has_tx_fault = true;
            drop_unsafe_tx(bus);
            return CAN_SERVICE_BUS_FAULT;
        case MC02_CAN_TX_INVALID:
            endpoint->pending_kind = CAN_SERVICE_TX_NONE;
            return CAN_SERVICE_INVALID;
        case MC02_CAN_TX_SUBMITTED:
            break;
    }

    bus->inflight = endpoint;
    bus->inflight_kind = endpoint->pending_kind;
    bus->inflight_expires_us = endpoint->expires_us;
    bus->is_cancelling = false;
    bus->next_endpoint = (selected + 1U) % bus->count;
    endpoint->last_submission = (can_service_submission_t){
        .kind = endpoint->pending_kind,
        .time_us = mc02_board_time_us(),
        .sequence = endpoint->last_submission.sequence + 1U,
    };
    endpoint->pending_kind = CAN_SERVICE_TX_NONE;
    return CAN_SERVICE_SUBMITTED;
}
