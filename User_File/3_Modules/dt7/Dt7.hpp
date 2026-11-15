#ifndef FRAME_CDC_3_MODULES_DT7_DT7_H_
#define FRAME_CDC_3_MODULES_DT7_DT7_H_

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: DT7 DR16 DBUS input with bounded framing and safe snapshots
=== END MANIFEST === */
// clang-format on

#include <array>
#include <cstdint>

#include "dt7_protocol.h"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "uart.hpp"

// 同名全局类/主头文件是XRobot模块接口；生成入口持有实例直到复位。
// UART仅由本模块消费，DMA/中断属于LibXR；所有业务状态均属于实例。
class Dt7 {
 public:
  // 构造后启动HIGH接收线程；输入借用UART、超时毫秒和栈字节数。
  // timeout_ms须为1..INT32_MAX，栈至少1024 B；失败保持离线且不建线程。
  explicit Dt7(LibXR::UART& uart, std::uint32_t timeout_ms = 100,
               std::uint32_t task_stack_depth = 2048);

  Dt7(const Dt7&) = delete;
  Dt7& operator=(const Dt7&) = delete;

  // 获取完整快照并复核超时；输入无，返回副本。仅任务调用，短暂等待互斥锁。
  frame_cdc::dt7::Snapshot GetSnapshot();

  // 获取构造接入结果；输入无，返回UART配置请求的接纳结果或参数错误。
  LibXR::ErrorCode initialization_status() const {
    return initialization_status_;
  }

  // 官方低频监视入口；输入输出均无，接收不依赖监视周期。
  void OnMonitor();

 private:
  // 永久接收线程；输入生命周期覆盖线程的自身指针，按1ms周期读取。
  static void Run(Dt7* self);

  // 单次读取及发布；输入无，更新解析器、诊断和调试视图。
  void Poll();

  LibXR::UART& uart_;
  const std::uint32_t timeout_ms_;
  LibXR::Mutex mutex_;
  frame_cdc::dt7::Receiver receiver_;
  // 调试器直接观察此成员；运行中多字段采样不保证来自同一帧。
  frame_cdc::dt7::Snapshot snapshot_{};
  LibXR::ErrorCode initialization_status_ = LibXR::ErrorCode::INIT_ERR;
  std::uint32_t read_errors_ = 0;
  std::uint32_t last_poll_ms_ = 0;
  std::array<std::uint8_t, 128> rx_buffer_{};
  LibXR::Semaphore read_done_{0};
  LibXR::ReadOperation read_{read_done_, 1};
  LibXR::Thread thread_;
};

#endif  // FRAME_CDC_3_MODULES_DT7_DT7_H_
