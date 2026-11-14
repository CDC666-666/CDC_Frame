#ifndef FRAME_CDC_DRIVER_UART_DRV_UART_H_
#define FRAME_CDC_DRIVER_UART_DRV_UART_H_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "stm32h7xx_hal.h"
#include "uart.hpp"

inline constexpr size_t UART_BUFFER_SIZE = 512;

// 有界文本/固定长度重组器；一个串口工作线程拥有，不把读取块当协议帧。
class StreamFramer {
 public:
  enum class Mode { TEXT, POWERMETER };
  // 选择协议并清空残包；输入模式，无输出。
  void Reset(Mode mode) {
    mode_ = mode;
    used_ = 0;
    discarding_ = false;
  }
  // 输入任意字节块；仅完整帧回调，溢出时丢到下一分隔符。
  template <typename Consumer>
  void Feed(const uint8_t* data, size_t size, Consumer consumer) {
    for (size_t index = 0; index < size; ++index) {
      const uint8_t byte = data[index];
      if (mode_ == Mode::POWERMETER && used_ == 0 && byte != 0xab) {
        continue;
      }
      if (mode_ == Mode::TEXT && (byte == '\r' || byte == '\n') && used_ == 0) {
        continue;
      }
      if (discarding_) {
        if (byte == '#') {
          discarding_ = false;
        }
        continue;
      }
      if (used_ == buffer_.size() - 1) {
        used_ = 0;
        discarding_ = byte != '#';
        continue;
      }
      buffer_[used_++] = byte;
      if ((mode_ == Mode::TEXT && byte == '#') ||
          (mode_ == Mode::POWERMETER && used_ == 14)) {
        buffer_[used_] = 0;
        consumer(buffer_.data(), static_cast<uint16_t>(used_));
        used_ = 0;
      }
    }
  }

 private:
  std::array<uint8_t, UART_BUFFER_SIZE> buffer_{};
  Mode mode_ = Mode::TEXT;
  size_t used_ = 0;
  bool discarding_ = false;
};

// 参考设备缓冲视图；接收时间是本地字节流读出时刻，不冒充线上IDLE时刻。
struct Struct_UART_Manage_Object {
  uint8_t Rx_Buffer_0[UART_BUFFER_SIZE]{};
  uint8_t* Rx_Buffer_Ready = Rx_Buffer_0;
  uint64_t Rx_Time_Stamp = 0;
  LibXR::Callback<const uint8_t*, uint16_t> on_receive;
  std::atomic<uint32_t> transmit_failed{0};

  // 绑定UART；输入已初始化接口，输出就绪状态，不重复启动DMA。
  bool Bind(LibXR::UART& uart);
  // 读取有界字节块；无输入，返回字节数，由协议层处理组帧。
  size_t Process();
  // 提交发送并复制进LibXR队列；输入字节，返回HAL兼容状态。
  uint8_t Send(const uint8_t* data, uint16_t size);

 private:
  LibXR::UART* uart_ = nullptr;
  LibXR::ReadOperation read_;
  LibXR::WriteOperation write_;
};

// 发送参考设备数据；输入管理对象及字节，输出提交状态。
uint8_t UART_Transmit_Data(Struct_UART_Manage_Object* driver,
                           const uint8_t* data, uint16_t size);

#endif
