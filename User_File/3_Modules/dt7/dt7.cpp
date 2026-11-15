// 固定XRobot要求主头文件与模块同名；对应头文件置于首位。
// clang-format off
#include "Dt7.hpp"
// clang-format on

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include "timebase.hpp"

// 绑定UART并检查配置；输入设备/超时/栈，成功后创建唯一接收线程。
Dt7::Dt7(LibXR::UART& uart, std::uint32_t timeout_ms,
         std::uint32_t task_stack_depth)
    : uart_(uart),
      timeout_ms_(timeout_ms),
      receiver_(std::chrono::milliseconds(timeout_ms)) {
  if (timeout_ms == 0 ||
      timeout_ms > static_cast<std::uint32_t>(
                       std::numeric_limits<std::int32_t>::max()) ||
      task_stack_depth < 1024) {
    initialization_status_ = LibXR::ErrorCode::ARG_ERR;
    return;
  }
  if (uart_.read_port_ == nullptr || !uart_.read_port_->Readable()) {
    initialization_status_ = LibXR::ErrorCode::NOT_SUPPORT;
    return;
  }
  // LibXR的8个有效位+EVEN映射为STM32的9位字长；P6已有硬件反相。
  initialization_status_ =
      uart_.SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 2});
  if (initialization_status_ != LibXR::ErrorCode::OK) {
    return;
  }
  last_poll_ms_ = LibXR::Timebase::GetMilliseconds();
  thread_.Create(this, Run, "dt7_rx", task_stack_depth,
                 LibXR::Thread::Priority::HIGH);
}

// 任务侧读完整副本；输入无，锁内校验过期，禁止控制任务直接访问调试成员。
frame_cdc::dt7::Snapshot Dt7::GetSnapshot() {
  LibXR::Mutex::LockGuard guard(mutex_);
  receiver_.Update(LibXR::Timebase::GetMilliseconds());
  snapshot_ = receiver_.snapshot();
  return snapshot_;
}

// 保留官方监视接口；输入输出无，不承担实时接收。
void Dt7::OnMonitor() {}

// 循环读取后发布；输入自身指针，线程保持到整机复位。
void Dt7::Run(Dt7* self) {
  auto wake_time = LibXR::Timebase::GetMilliseconds();
  while (true) {
    self->Poll();
    // 在此行设断点，可通过self->snapshot_查看一次完整发布后的值。
    LibXR::Thread::SleepUntil(wake_time, 1);
  }
}

// 每周期最多读128字节；输入无，阻塞式完成契约保证缓冲在返回后可复用。
void Dt7::Poll() {
  const std::uint32_t now_ms = LibXR::Timebase::GetMilliseconds();
  const bool stalled =
      static_cast<std::uint32_t>(now_ms - last_poll_ms_) >= timeout_ms_;
  last_poll_ms_ = now_ms;
  // 长时间停调度/调试后清退旧积压，避免将旧队列数据冒充新鲜输入。
  if (stalled) {
    const auto result = uart_.read_port_->ClearQueuedData();
    LibXR::Mutex::LockGuard guard(mutex_);
    receiver_.Invalidate();
    if (result != LibXR::ErrorCode::OK) {
      ++read_errors_;
    }
    snapshot_ = receiver_.snapshot();
    return;
  }
  const auto size = std::min(uart_.read_port_->Size(), rx_buffer_.size());
  const auto result = size == 0 ? LibXR::ErrorCode::OK
                                : uart_.Read({rx_buffer_.data(), size}, read_);
  LibXR::Mutex::LockGuard guard(mutex_);
  receiver_.Update(now_ms);
  if (result != LibXR::ErrorCode::OK) {
    ++read_errors_;
    receiver_.Invalidate();
  } else if (size != 0) {
    receiver_.Feed(std::span<const std::uint8_t>(rx_buffer_.data(), size),
                   now_ms);
  }
  snapshot_ = receiver_.snapshot();
}
