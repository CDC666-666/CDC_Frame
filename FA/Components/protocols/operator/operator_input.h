#ifndef FRAME_CDC_COMPONENTS_PROTOCOLS_OPERATOR_OPERATOR_INPUT_H
#define FRAME_CDC_COMPONENTS_PROTOCOLS_OPERATOR_OPERATOR_INPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    OPERATOR_COMMAND_NONE = 0,
    OPERATOR_COMMAND_ARM,
    OPERATOR_COMMAND_RUN,
    OPERATOR_COMMAND_ZERO,
    OPERATOR_COMMAND_DISABLE,
} operator_command_t;

typedef struct {
    char line[7];
    size_t size;
    bool is_invalid;
    uint64_t last_byte_us;
} operator_input_t;

/** @brief 非阻塞轮询 UART7，返回至多一条完整命令。实例初始为零。 */
operator_command_t operator_input_poll(operator_input_t *input, uint64_t now_us);

#endif /* FRAME_CDC_COMPONENTS_PROTOCOLS_OPERATOR_OPERATOR_INPUT_H */
