#ifndef FRAME_CDC_BASE_SERVICES_CAN_CAN_SERVICE_H
#define FRAME_CDC_BASE_SERVICES_CAN_CAN_SERVICE_H

#include "mc02_can.h"

typedef enum {
    CAN_SERVICE_TX_NONE = 0,
    CAN_SERVICE_TX_BACKGROUND,
    CAN_SERVICE_TX_NORMAL,
    CAN_SERVICE_TX_HIGH,
    CAN_SERVICE_TX_URGENT,
} can_service_tx_kind_t;

typedef enum {
    CAN_SERVICE_IDLE = 0,
    CAN_SERVICE_QUEUED,
    CAN_SERVICE_SUBMITTED,
    CAN_SERVICE_BUSY,
    CAN_SERVICE_BUS_FAULT,
    CAN_SERVICE_INVALID,
} can_service_result_t;

typedef struct {
    can_service_tx_kind_t kind;
    uint64_t time_us;
    uint32_t sequence;
} can_service_submission_t;

typedef struct {
    can_service_tx_kind_t kind;   /* NONE 表示尚未观察到成功发送。 */
    uint32_t submission_sequence; /* 对应提交事件的序号，按 32 位回绕。 */
} can_service_transmission_t;

typedef struct can_service can_service_t;
typedef void (*can_service_receive_fn)(void *context, const can_frame_t *frame);

typedef struct {
    can_service_t *bus;
    void *context;
    can_service_receive_fn receive;
    uint16_t feedback_id;
    can_frame_t pending;
    uint64_t expires_us;
    can_service_tx_kind_t pending_kind;
    can_service_submission_t last_submission;
    can_service_transmission_t last_transmission;
} can_service_endpoint_t;

struct can_service {
    can_service_endpoint_t endpoints[MC02_CAN_FILTER_CAPACITY];
    size_t count;
    size_t next_endpoint;
    can_service_endpoint_t *inflight;
    can_service_tx_kind_t inflight_kind;
    uint64_t inflight_expires_us;
    uint64_t last_poll_us;
    mc02_can_port_t port;
    mc02_can_status_t status;
    bool is_started;
    bool has_registration_error;
    bool has_tx_fault;
    bool is_cancelling;
};

/** @brief 初始化软件总线，随后注册设备。所有接口均由同一个主循环调用。 */
void can_service_init(can_service_t *bus, mc02_can_port_t port);

/** @brief 注册唯一反馈 ID。失败返回 NULL，配置有冲突时拒绝启动该总线。 */
can_service_endpoint_t *can_service_register(can_service_t *bus, uint16_t feedback_id,
                                             can_service_receive_fn receive, void *context);

/** @brief 从注册表生成硬件过滤列表并启动总线。bitrate_hz=0 时保持未启动。 */
bool can_service_start(can_service_t *bus, uint32_t bitrate_hz, uint64_t now_us);

/** @brief 采集总线状态并有界地分发反馈。now_us 使用板级单调时钟。 */
void can_service_poll_rx(can_service_t *bus, uint64_t now_us);

/** @brief 每轮至多提交一帧，紧急帧优先，其余端点轮转。SUBMITTED 表示硬件接受请求。 */
can_service_result_t can_service_poll_tx(can_service_t *bus);

/** @brief 复制待发帧。每个实例最多一帧，同级或更高优先级的新帧覆盖旧帧。
 * @details expires_us 是绝对时刻，达到时丢弃。紧急帧可跨总线故障保留至提交，不受截止时间限制，并撤销本端点在途的非紧急帧。
 *          返回 QUEUED 表示已排队，BUSY 表示本实例有更高优先级帧。
 */
can_service_result_t can_service_queue(can_service_endpoint_t *endpoint, can_service_tx_kind_t kind,
                                       const can_frame_t *frame, uint64_t expires_us);

/** @brief 仅撤销本端点上类别匹配的待发或在途帧。 */
void can_service_cancel(can_service_endpoint_t *endpoint, can_service_tx_kind_t kind);

/** @brief 只移除本实例尚未提交且类别匹配的帧。 */
void can_service_clear_queued(can_service_endpoint_t *endpoint, can_service_tx_kind_t kind);

#endif /* FRAME_CDC_BASE_SERVICES_CAN_CAN_SERVICE_H */
