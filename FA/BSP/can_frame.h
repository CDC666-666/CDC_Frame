#ifndef FRAME_CDC_PLATFORM_CAN_FRAME_H
#define FRAME_CDC_PLATFORM_CAN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

/* 板级交付的短帧。接收时间使用板级单调微秒时钟。 */
typedef struct {
    uint32_t id;
    uint64_t rx_time_us;
    uint8_t length;
    uint8_t data[8];
    bool is_extended;
    bool is_remote;
    bool is_fd;
} can_frame_t;

#endif /* FRAME_CDC_PLATFORM_CAN_FRAME_H */
