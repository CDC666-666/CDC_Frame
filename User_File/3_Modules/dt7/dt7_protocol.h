#ifndef FRAME_CDC_3_MODULES_DT7_DT7_PROTOCOL_H_
#define FRAME_CDC_3_MODULES_DT7_DT7_PROTOCOL_H_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace frame_cdc::dt7 {

enum class Switch : std::uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 };

// 通道为无量纲协议方向：右横、右纵、左横、左纵；尚未映射机器人坐标。
// 协议原值/诊断计数保留设备边界位宽；计数和毫秒时间戳按uint32_t回绕。
struct Snapshot {
  std::array<double, 4> channel{};
  std::array<std::uint16_t, 4> channel_raw{1024, 1024, 1024, 1024};
  Switch switch_left = Switch::UNKNOWN;
  Switch switch_right = Switch::UNKNOWN;
  bool online = false;
  std::uint32_t valid_frames = 0;
  std::uint32_t discarded_bytes = 0;
  // 首次同步不计数；每次失步或超时后成功重新锁定加一。
  std::uint32_t resync_count = 0;
  // 本地处理有效帧的HAL毫秒时刻，非线上的精确接收时刻。
  std::uint32_t last_valid_frame_ms = 0;
};

// 固定36字节滑窗；单线程拥有，线程间同步由Dt7适配层负责。
class Receiver {
 public:
  // 创建离线接收器；输入超时持续时间，须为1..INT32_MAX毫秒。
  explicit Receiver(std::chrono::milliseconds timeout);

  // 输入任意字节块及当前HAL毫秒tick；更新完整有效帧，不保存输入引用。
  void Feed(std::span<const std::uint8_t> bytes, std::uint32_t now_ms);

  // 检查帧和残包超时；输入HAL毫秒tick，失效时清零并要求重新同步。
  void Update(std::uint32_t now_ms);

  // 丢弃残包并进入离线；输入无，保留诊断及最后有效帧时间。
  void Invalidate();

  // 返回当前只读状态；输入无，引用仅有效至下一次修改，不跨线程共享。
  const Snapshot& snapshot() const { return snapshot_; }

 private:
  struct Frame {
    std::array<std::uint16_t, 4> channel_raw{};
    Switch switch_left = Switch::UNKNOWN;
    Switch switch_right = Switch::UNKNOWN;
  };

  // 显式解码18字节；输入定长帧，成功写入输出，失败返回false。
  static bool Decode(std::span<const std::uint8_t, 18> bytes, Frame& output);

  // 消费滑窗内可判定数据；输入当前tick，更新同步状态和输出。
  void Process(std::uint32_t now_ms);

  // 发布有效帧；输入帧和本地tick，更新归一化值、在线和计数。
  void Accept(const Frame& frame, std::uint32_t now_ms);

  // 移除前缀；输入长度及是否计为丢弃，保留未消费尾部。
  void RemovePrefix(std::size_t size, bool discarded);

  // 清除操纵输出；输入无，诊断计数和时间不变。
  void SetOffline();

  std::chrono::milliseconds timeout_;
  std::array<std::uint8_t, 36> buffer_{};
  std::size_t used_ = 0;
  bool locked_ = false;
  bool ever_locked_ = false;
  std::uint32_t last_byte_ms_ = 0;
  Snapshot snapshot_{};
};

}  // namespace frame_cdc::dt7

#endif  // FRAME_CDC_3_MODULES_DT7_DT7_PROTOCOL_H_
