#ifndef FRAME_CDC_DRIVER_USB_DRV_USB_H_
#define FRAME_CDC_DRIVER_USB_DRV_USB_H_

#include <cstring>

#include "1_Middleware/Driver/UART/drv_uart.h"

using Struct_USB_Manage_Object = Struct_UART_Manage_Object;
inline constexpr size_t USB_BUFFER_SIZE = UART_BUFFER_SIZE;

// 通过现有CDCUart发送；输入显式对象和字节，输出提交状态。
uint8_t USB_Transmit_Data(Struct_USB_Manage_Object* driver, const uint8_t* data,
                          uint16_t size);

namespace frame_cdc {

// 单消费者CDC分流器；只在行首识别四个参考Kalman参数，其他输入原样交终端。
class UsbInputRouter {
 public:
  // 分流任意分包的输入；消费者同步借用缓冲，返回后不得保留指针。
  template <typename Console, typename Tune>
  void Feed(const uint8_t* data, size_t size, Console console, Tune tune) {
    for (size_t index = 0; index < size; ++index) {
      const uint8_t byte = data[index];
      if ((state_ == State::TUNE || state_ == State::DROP) &&
          (byte == '\r' || byte == '\n')) {
        state_ = State::DETECT;
        length_ = 0;
        console(&byte, 1);
        continue;
      }
      if (state_ == State::CONSOLE) {
        console(&byte, 1);
        if (byte == '\r' || byte == '\n') {
          state_ = State::DETECT;
        }
      } else if (state_ == State::DROP) {
        if (byte == '#') {
          state_ = State::DETECT;
          length_ = 0;
        }
      } else if (state_ == State::TUNE) {
        if (length_ == sizeof(buffer_)) {
          state_ = State::DROP;
        } else {
          buffer_[length_++] = byte;
          if (byte == '#') {
            tune(buffer_, length_);
            length_ = 0;
            state_ = State::DETECT;
          }
        }
        if (state_ == State::DROP && byte == '#') {
          length_ = 0;
          state_ = State::DETECT;
        }
      } else {
        if (length_ == 0 && (byte == '\r' || byte == '\n')) {
          console(&byte, 1);
          continue;
        }
        buffer_[length_++] = byte;
        constexpr const char* prefixes[] = {"q00=", "q11=", "r00=", "r11="};
        bool matched = false;
        for (const auto* prefix : prefixes) {
          if (std::memcmp(buffer_, prefix, length_) == 0) {
            matched = true;
          }
        }
        if (!matched) {
          console(buffer_, length_);
          length_ = 0;
          state_ =
              (byte == '\r' || byte == '\n') ? State::DETECT : State::CONSOLE;
        } else if (length_ == 4) {
          state_ = State::TUNE;
        }
      }
    }
  }

 private:
  enum class State { DETECT, CONSOLE, TUNE, DROP };
  State state_ = State::DETECT;
  uint8_t buffer_[100]{};
  size_t length_ = 0;
};

}  // namespace frame_cdc

#endif
