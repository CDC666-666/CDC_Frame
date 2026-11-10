/* UART7 台架操作指令的行协议。 */
#include "operator_input.h"

#include <string.h>

#define OPERATOR_INPUT_TIMEOUT_US 1000000U

static operator_command_t parse_command(const operator_input_t *input)
{
    if (!input->is_invalid) {
        if (input->size == 3U && memcmp(input->line, "arm", 3U) == 0) {
            return OPERATOR_COMMAND_ARM;
        }
        if (input->size == 3U && memcmp(input->line, "run", 3U) == 0) {
            return OPERATOR_COMMAND_RUN;
        }
        if (input->size == 4U && memcmp(input->line, "zero", 4U) == 0) {
            return OPERATOR_COMMAND_ZERO;
        }
        if (input->size == 7U && memcmp(input->line, "disable", 7U) == 0) {
            return OPERATOR_COMMAND_DISABLE;
        }
    }
    return OPERATOR_COMMAND_NONE;
}

operator_command_t operator_input_poll(operator_input_t *input, uint64_t now_us,
                                       int (*read_byte)(void))
{
    if (input->size > 0U && now_us - input->last_byte_us >= OPERATOR_INPUT_TIMEOUT_US) {
        input->is_invalid = true;
    }
    for (size_t i = 0U; i < 16U; ++i) {
        const int byte = read_byte();
        if (byte == -1) {
            break;
        }
        if (byte == '\r' || byte == '\n') {
            const operator_command_t command = parse_command(input);
            input->size = 0U;
            input->is_invalid = false;
            if (command != OPERATOR_COMMAND_NONE) {
                return command;
            }
        } else if (byte < 0 || input->size >= sizeof(input->line)) {
            input->is_invalid = true;
        } else {
            input->line[input->size++] = (char)byte;
            input->last_byte_us = now_us;
        }
    }
    return OPERATOR_COMMAND_NONE;
}
