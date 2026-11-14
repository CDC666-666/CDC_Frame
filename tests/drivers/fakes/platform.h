#ifndef FRAME_CDC_TEST_DRIVER_PLATFORM_H_
#define FRAME_CDC_TEST_DRIVER_PLATFORM_H_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <thread>
#include <vector>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
struct GPIO_TypeDef {
  GPIO_PinState state = GPIO_PIN_SET;
};
inline void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t,
                              GPIO_PinState state) {
  port->state = state;
}
inline constexpr uint32_t FDCAN_STANDARD_ID = 0;
inline constexpr uint32_t FDCAN_DATA_FRAME = 0;
struct FDCAN_RxHeaderTypeDef {
  uint32_t Identifier = 0, DataLength = 0, IdType = 0, RxFrameType = 0;
};
inline constexpr uint32_t HAL_OSPI_OPTYPE_COMMON_CFG = 0;
inline constexpr uint32_t HAL_OSPI_FLASH_ID_1 = 0;
inline constexpr uint32_t HAL_OSPI_INSTRUCTION_1_LINE = 1;
inline constexpr uint32_t HAL_OSPI_INSTRUCTION_8_BITS = 8;
inline constexpr uint32_t HAL_OSPI_ADDRESS_1_LINE = 1;
inline constexpr uint32_t HAL_OSPI_ADDRESS_NONE = 0;
inline constexpr uint32_t HAL_OSPI_ADDRESS_24_BITS = 24;
inline constexpr uint32_t HAL_OSPI_DATA_NONE = 0;
inline constexpr uint32_t HAL_OSPI_DATA_1_LINE = 1;
inline constexpr uint32_t HAL_OSPI_DATA_4_LINES = 4;
inline constexpr uint32_t HAL_OSPI_MATCH_MODE_AND = 0;
inline constexpr uint32_t HAL_OSPI_AUTOMATIC_STOP_ENABLE = 1;
inline constexpr uint32_t HAL_OSPI_TIMEOUT_COUNTER_ENABLE = 1;
inline constexpr uint32_t HAL_OSPI_STATE_READY = 2;
inline constexpr uint32_t HAL_OSPI_STATE_ERROR = 0x200;
inline constexpr uint32_t HAL_OSPI_ERROR_NONE = 0;
struct OSPI_RegularCmdTypeDef {
  uint32_t OperationType = 0, FlashId = 0, Instruction = 0, InstructionMode = 0;
  uint32_t InstructionSize = 0, Address = 0, AddressMode = 0, AddressSize = 0;
  uint32_t DataMode = 0, NbData = 0, DummyCycles = 0;
};
struct OSPI_AutoPollingTypeDef {
  uint32_t Match = 0, Mask = 0, MatchMode = 0, AutomaticStop = 0, Interval = 0;
};
struct OSPI_MemoryMappedTypeDef {
  uint32_t TimeOutActivation = 0, TimeOutPeriod = 0;
};
struct OSPI_HandleTypeDef {
  enum class Pending { NONE, RECEIVE, TRANSMIT, POLL };
  Pending pending = Pending::NONE;
  uint8_t* pending_buffer = nullptr;
  OSPI_AutoPollingTypeDef pending_poll{};
  uint32_t state_polls = 0, poll_steps = 0;
  uint32_t dma_receives = 0, dma_transmits = 0, it_polls = 0, aborts = 0;
  bool stall = false;
  OSPI_RegularCmdTypeDef command;
  std::vector<OSPI_RegularCmdTypeDef> commands;
  std::vector<uint8_t> memory = std::vector<uint8_t>(8 * 1024 * 1024, 0xff);
  HAL_StatusTypeDef result = HAL_OK;
  bool wel = false, mapped = false;
  uint8_t sr2 = 0;
  uint32_t receives = 0, transmits = 0;
};
inline HAL_StatusTypeDef HAL_OSPI_Command(OSPI_HandleTypeDef* handle,
                                          OSPI_RegularCmdTypeDef* command,
                                          uint32_t) {
  if (handle->result != HAL_OK) {
    return handle->result;
  }
  handle->command = *command;
  handle->commands.push_back(*command);
  if (command->Instruction == 0x06) {
    handle->wel = true;
  }
  if (command->Instruction == 0x20 && handle->wel) {
    std::fill_n(handle->memory.begin() + command->Address, 4096, 0xff);
    handle->wel = false;
  }
  return HAL_OK;
}
inline HAL_StatusTypeDef HAL_OSPI_Receive(OSPI_HandleTypeDef* handle,
                                          uint8_t* output, uint32_t) {
  ++handle->receives;
  if (handle->command.Instruction == 0x9f) {
    output[0] = 0xef;
    output[1] = 0x40;
    output[2] = 0x17;
  } else if (handle->command.Instruction == 0x35) {
    output[0] = handle->sr2;
  } else {
    std::copy_n(handle->memory.begin() + handle->command.Address,
                handle->command.NbData, output);
  }
  return handle->result;
}
inline HAL_StatusTypeDef HAL_OSPI_Transmit(OSPI_HandleTypeDef* handle,
                                           uint8_t* input, uint32_t) {
  ++handle->transmits;
  if (handle->command.Instruction == 0x31) {
    handle->sr2 = input[0];
  } else {
    if (!handle->wel) {
      return HAL_ERROR;
    }
    for (uint32_t index = 0; index < handle->command.NbData; ++index) {
      handle->memory[handle->command.Address + index] &= input[index];
    }
    handle->wel = false;
  }
  return handle->result;
}
inline HAL_StatusTypeDef HAL_OSPI_AutoPolling(OSPI_HandleTypeDef* handle,
                                              OSPI_AutoPollingTypeDef* config,
                                              uint32_t) {
  return (((handle->wel ? 2U : 0U) & config->Mask) == config->Match)
             ? handle->result
             : HAL_TIMEOUT;
}
inline HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef* handle,
                                               OSPI_MemoryMappedTypeDef*) {
  handle->mapped = true;
  return handle->result;
}
inline HAL_StatusTypeDef HAL_OSPI_Abort(OSPI_HandleTypeDef* handle) {
  ++handle->aborts;
  handle->pending = OSPI_HandleTypeDef::Pending::NONE;
  handle->mapped = false;
  return handle->result;
}

// MDMA替身延迟到状态观察后完成；输入句柄/缓冲，返回接受状态。
inline HAL_StatusTypeDef HAL_OSPI_Receive_DMA(OSPI_HandleTypeDef* handle,
                                              uint8_t* data) {
  ++handle->dma_receives;
  handle->pending = OSPI_HandleTypeDef::Pending::RECEIVE;
  handle->pending_buffer = data;
  handle->poll_steps = 0;
  return handle->result;
}
// 启动发送模型；输入句柄/缓冲，不立刻修改Flash。
inline HAL_StatusTypeDef HAL_OSPI_Transmit_DMA(OSPI_HandleTypeDef* handle,
                                               uint8_t* data) {
  ++handle->dma_transmits;
  handle->pending = OSPI_HandleTypeDef::Pending::TRANSMIT;
  handle->pending_buffer = data;
  handle->poll_steps = 0;
  return handle->result;
}
// 启动匹配模型；输入配置，延迟到状态观察后完成。
inline HAL_StatusTypeDef HAL_OSPI_AutoPolling_IT(
    OSPI_HandleTypeDef* handle, OSPI_AutoPollingTypeDef* config) {
  ++handle->it_polls;
  handle->pending = OSPI_HandleTypeDef::Pending::POLL;
  handle->pending_poll = *config;
  handle->poll_steps = 0;
  return handle->result;
}
// 模拟HAL IRQ更新状态；输入句柄，返回是否完成，stall时保持在途。
inline uint32_t HAL_OSPI_GetState(OSPI_HandleTypeDef* handle) {
  ++handle->state_polls;
  if (handle->pending == OSPI_HandleTypeDef::Pending::NONE) {
    return HAL_OSPI_STATE_READY;
  }
  if (handle->stall || ++handle->poll_steps < 2) {
    return 0x28;
  }
  using Pending = OSPI_HandleTypeDef::Pending;
  const auto phase = handle->pending;
  handle->pending = Pending::NONE;
  if (phase == Pending::RECEIVE) {
    HAL_OSPI_Receive(handle, handle->pending_buffer, 0);
  } else if (phase == Pending::TRANSMIT) {
    HAL_OSPI_Transmit(handle, handle->pending_buffer, 0);
  } else if (HAL_OSPI_AutoPolling(handle, &handle->pending_poll, 0) != HAL_OK) {
    handle->pending = phase;
    return 0x48;
  }
  return HAL_OSPI_STATE_READY;
}
// 读取模型错误；输入句柄，返回HAL兼容错误位。
inline uint32_t HAL_OSPI_GetError(OSPI_HandleTypeDef*) {
  return HAL_OSPI_ERROR_NONE;
}

namespace LibXR {
enum class ErrorCode { OK, FULL, EMPTY, BUSY, TIMEOUT, SIZE_ERR };
struct RawData {
  void* addr_;
  size_t size_;
};
struct ConstRawData {
  const void* addr_;
  size_t size_;
};
class Timebase {
 public:
  static uint64_t GetMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
};
template <typename... Args>
class Callback {
 public:
  template <typename Function, typename Object>
  static Callback Create(Function function, Object object) {
    Callback result;
    result.function_ = [=](bool in_isr, Args... args) {
      function(in_isr, object, args...);
    };
    return result;
  }
  void Run(bool in_isr, Args... args) const {
    if (function_) {
      function_(in_isr, args...);
    }
  }

 private:
  std::function<void(bool, Args...)> function_;
};
template <typename T>
class SPSCQueue {
 public:
  explicit SPSCQueue(size_t capacity) : capacity_(capacity) {}
  ErrorCode Push(const T& value) {
    if (values_.size() == capacity_) {
      return ErrorCode::FULL;
    }
    values_.push_back(value);
    return ErrorCode::OK;
  }
  ErrorCode Pop(T& value) {
    if (values_.empty()) {
      return ErrorCode::EMPTY;
    }
    value = values_.front();
    values_.pop_front();
    return ErrorCode::OK;
  }

 private:
  size_t capacity_;
  std::deque<T> values_;
};
class Semaphore {};
class Thread {
 public:
  static void Sleep(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }
};
class ReadOperation {};
class WriteOperation {
 public:
  using Callback = LibXR::Callback<ErrorCode>;
  Callback* callback = nullptr;
  WriteOperation() = default;
  WriteOperation(Semaphore&, uint32_t) {}
  explicit WriteOperation(Callback& value) : callback(&value) {}
};
class ReadPort {
 public:
  std::vector<uint8_t> bytes;
  size_t Size() const { return bytes.size(); }
  ErrorCode operator()(RawData data, ReadOperation&) {
    if (data.size_ > bytes.size()) {
      return ErrorCode::EMPTY;
    }
    std::copy_n(bytes.begin(), data.size_, static_cast<uint8_t*>(data.addr_));
    bytes.erase(bytes.begin(), bytes.begin() + data.size_);
    return ErrorCode::OK;
  }
};
class WritePort {
 public:
  std::vector<uint8_t> bytes;
  ErrorCode result = ErrorCode::OK;
  size_t Size() const { return bytes.size(); }
  size_t Capacity() const { return 128; }
  ErrorCode operator()(ConstRawData data, WriteOperation&, bool = false) {
    if (result != ErrorCode::OK) {
      return result;
    }
    const auto* source = static_cast<const uint8_t*>(data.addr_);
    bytes.assign(source, source + data.size_);
    return ErrorCode::OK;
  }
};
struct UART {
  ReadPort* read_port_ = nullptr;
  WritePort* write_port_ = nullptr;
};
class CAN {
 public:
  enum class Type { STANDARD };
  struct ClassicPack {
    uint32_t id;
    Type type;
    uint8_t dlc;
    uint8_t data[8];
  };
  using Callback = LibXR::Callback<const ClassicPack&>;
  Callback receiver;
  ClassicPack last{};
  ErrorCode result = ErrorCode::OK;
  void Register(Callback callback, Type) { receiver = callback; }
  ErrorCode AddMessage(const ClassicPack& pack) {
    last = pack;
    return result;
  }
};
class SPI {
 public:
  using OperationRW = WriteOperation;
  ErrorCode result = ErrorCode::OK;
  std::vector<uint8_t> sent;
  GPIO_TypeDef* cs = nullptr;
  bool cs_was_active = false;
  OperationRW::Callback* pending = nullptr;
  RawData pending_read{};
  ErrorCode Write(ConstRawData data, OperationRW& operation,
                  bool in_isr = false) {
    return ReadAndWrite({nullptr, 0}, data, operation, in_isr);
  }
  ErrorCode ReadAndWrite(RawData read, ConstRawData write, OperationRW& op,
                         bool = false) {
    if (cs != nullptr) {
      cs_was_active = cs->state == GPIO_PIN_RESET;
    }
    const auto* bytes = static_cast<const uint8_t*>(write.addr_);
    sent.assign(bytes, bytes + write.size_);
    if (op.callback != nullptr && result == ErrorCode::OK) {
      pending = op.callback;
      pending_read = read;
      return result;
    }
    for (size_t index = 0; index < read.size_; ++index) {
      static_cast<uint8_t*>(read.addr_)[index] =
          static_cast<uint8_t>(0x10 + index);
    }
    return result;
  }
  // 延迟DMA完成；输入是否ISR，先写接收区再执行借用回调。
  void Complete(bool in_isr = true) {
    auto* callback = pending;
    pending = nullptr;
    for (size_t index = 0; index < pending_read.size_; ++index) {
      static_cast<uint8_t*>(pending_read.addr_)[index] =
          static_cast<uint8_t>(0x10 + index);
    }
    if (callback != nullptr) {
      callback->Run(in_isr, ErrorCode::OK);
    }
  }
};
struct ADC {
  float voltage = 1.5f;
  float Read() { return voltage; }
};
struct PWM {};
struct Watchdog {
  uint32_t feeds = 0;
  void Feed() { ++feeds; }
};
}  // namespace LibXR
#endif
