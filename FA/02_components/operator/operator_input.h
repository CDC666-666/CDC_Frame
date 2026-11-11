#ifndef FRAME_CDC_COMPONENTS_OPERATOR_OPERATOR_INPUT_H
#define FRAME_CDC_COMPONENTS_OPERATOR_OPERATOR_INPUT_H

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

/** 作用有界轮询非阻塞字节源并维护行解析与超时状态。
 * 输入非空 input、now_us 和 read_byte（字节或 -1 空、-2 错误），输出更新的 input 和至多一条操作枚举。 */
operator_command_t operator_input_poll(operator_input_t *input, uint64_t now_us,
                                       int (*read_byte)(void));

#endif /* FRAME_CDC_COMPONENTS_OPERATOR_OPERATOR_INPUT_H */
