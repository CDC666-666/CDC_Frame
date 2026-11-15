#include "dt7_protocol.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace frame_cdc::dt7 {

// 保存超时参数；输入持续时间，初始状态保持离线。
Receiver::Receiver(std::chrono::milliseconds timeout) : timeout_(timeout) {}

// 解码通道和拨杆并检查候选帧；输入18字节，输出合法性及解析结果。
bool Receiver::Decode(std::span<const std::uint8_t, 18> bytes, Frame& output) {
  output.channel_raw[0] = (bytes[0] | (bytes[1] << 8)) & 0x07ff;
  output.channel_raw[1] = ((bytes[1] >> 3) | (bytes[2] << 5)) & 0x07ff;
  output.channel_raw[2] =
      ((bytes[2] >> 6) | (bytes[3] << 2) | (bytes[4] << 10)) & 0x07ff;
  output.channel_raw[3] = ((bytes[4] >> 1) | (bytes[5] << 7)) & 0x07ff;
  output.switch_right = static_cast<Switch>((bytes[5] >> 4) & 3);
  output.switch_left = static_cast<Switch>((bytes[5] >> 6) & 3);
  for (const auto value : output.channel_raw) {
    if (value < 364 || value > 1684) {
      return false;
    }
  }
  // 不依赖鼠标移动/键盘/末尾保留字段全零；仅检查按钮的协议取值。
  return output.switch_left != Switch::UNKNOWN &&
         output.switch_right != Switch::UNKNOWN && bytes[12] <= 1 &&
         bytes[13] <= 1;
}

// 使用固定滑窗吸收任意分块；输入字节和tick，过期残包不与新数据拼接。
void Receiver::Feed(std::span<const std::uint8_t> bytes, std::uint32_t now_ms) {
  Update(now_ms);
  for (const auto byte : bytes) {
    // Process每个字节后执行，最多等待两个18字节候选帧。
    buffer_[used_++] = byte;
    last_byte_ms_ = now_ms;
    Process(now_ms);
  }
}

// 检查无符号tick差；输入当前tick，支持HAL_GetTick的32位回绕。
void Receiver::Update(std::uint32_t now_ms) {
  const auto frame_age = std::chrono::milliseconds(
      static_cast<std::uint32_t>(now_ms - snapshot_.last_valid_frame_ms));
  const auto partial_age = std::chrono::milliseconds(
      static_cast<std::uint32_t>(now_ms - last_byte_ms_));
  if ((snapshot_.online && frame_age >= timeout_) ||
      (used_ != 0 && partial_age >= timeout_)) {
    Invalidate();
  }
}

// 失去可靠输入时清空滑窗；输入无，旧控制量立即撤销。
void Receiver::Invalidate() {
  RemovePrefix(used_, true);
  locked_ = false;
  SetOffline();
}

// 连续两帧确认同步；输入当前tick，锁定后每18字节发布一次。
void Receiver::Process(std::uint32_t now_ms) {
  while (used_ >= 18) {
    Frame first;
    if (!Decode(std::span<const std::uint8_t, 18>(buffer_.data(), 18), first)) {
      locked_ = false;
      SetOffline();
      RemovePrefix(1, true);
      continue;
    }
    if (locked_) {
      Accept(first, now_ms);
      RemovePrefix(18, false);
      continue;
    }
    if (used_ < 36) {
      return;
    }
    Frame second;
    if (!Decode(std::span<const std::uint8_t, 18>(buffer_.data() + 18, 18),
                second)) {
      RemovePrefix(1, true);
      continue;
    }
    if (ever_locked_) {
      ++snapshot_.resync_count;
    }
    ever_locked_ = true;
    locked_ = true;
    Accept(first, now_ms);
    Accept(second, now_ms);
    RemovePrefix(36, false);
  }
}

// 转换通道；输入通过校验的帧和tick，输出保持DBUS协议方向。
void Receiver::Accept(const Frame& frame, std::uint32_t now_ms) {
  snapshot_.channel_raw = frame.channel_raw;
  for (std::size_t index = 0; index < frame.channel_raw.size(); ++index) {
    snapshot_.channel[index] =
        (static_cast<int>(frame.channel_raw[index]) - 1024) / 660.0;
  }
  snapshot_.switch_left = frame.switch_left;
  snapshot_.switch_right = frame.switch_right;
  snapshot_.online = true;
  snapshot_.last_valid_frame_ms = now_ms;
  ++snapshot_.valid_frames;
}

// 前移残包；输入有效前缀长度及计数开关，输出剩余滑窗。
void Receiver::RemovePrefix(std::size_t size, bool discarded) {
  if (discarded) {
    snapshot_.discarded_bytes += static_cast<std::uint32_t>(size);
  }
  std::move(buffer_.begin() + size, buffer_.begin() + used_, buffer_.begin());
  used_ -= size;
}

// 设置安全输出；输入无，诊断记录保留用于断联分析。
void Receiver::SetOffline() {
  snapshot_.channel.fill(0.0);
  snapshot_.channel_raw.fill(1024);
  snapshot_.switch_left = Switch::UNKNOWN;
  snapshot_.switch_right = Switch::UNKNOWN;
  snapshot_.online = false;
}

}  // namespace frame_cdc::dt7
