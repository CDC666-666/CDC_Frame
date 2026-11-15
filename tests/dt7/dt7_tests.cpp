#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>
#include <vector>

#include "dt7_protocol.h"

namespace {

using frame_cdc::dt7::Receiver;
using frame_cdc::dt7::Snapshot;
using frame_cdc::dt7::Switch;
using Frame = std::array<std::uint8_t, 18>;

// 用位索引编码测试输入，独立于解码公式；输入通道/开关，返回DBUS帧。
Frame Encode(std::array<std::uint16_t, 4> channels = {1024, 1024, 1024, 1024},
             Switch left = Switch::UP, Switch right = Switch::MIDDLE) {
  Frame frame{};
  for (std::size_t channel = 0; channel < channels.size(); ++channel) {
    for (std::size_t bit = 0; bit < 11; ++bit) {
      const auto position = channel * 11 + bit;
      frame[position / 8] |= ((channels[channel] >> bit) & 1U)
                             << (position % 8);
    }
  }
  frame[5] |= (static_cast<std::uint8_t>(right) << 4) |
              (static_cast<std::uint8_t>(left) << 6);
  // 非零鼠标/键盘/末尾字段须允许，不能把它们全零当作帧头。
  frame[6] = 0xa5;
  frame[7] = 0x83;
  frame[8] = 0x42;
  frame[9] = 0xf1;
  frame[10] = 0xb6;
  frame[11] = 0x74;
  frame[12] = 1;
  frame[14] = 0x5a;
  frame[15] = 0xa5;
  frame[17] = 4;
  return frame;
}

// 拼接两个帧；输入两帧，返回字节流供任意拆分测试。
std::vector<std::uint8_t> Pair(const Frame& first, const Frame& second) {
  std::vector<std::uint8_t> stream(first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());
  return stream;
}

// 校验离线安全输出；输入快照，返回所有操纵量已撤销的结果。
bool IsSafe(const Snapshot& state) {
  return !state.online && state.channel == std::array<double, 4>{} &&
         state.channel_raw ==
             std::array<std::uint16_t, 4>{1024, 1024, 1024, 1024} &&
         state.switch_left == Switch::UNKNOWN &&
         state.switch_right == Switch::UNKNOWN;
}

// 验证独立字节样本和各通道位域；输入无，返回归一化及拨杆检查结果。
bool TestChannels() {
  Receiver receiver(std::chrono::milliseconds(100));
  // 人工按协议打包：四通道均1024，右拨中(3)，左拨上(1)。
  constexpr Frame neutral{0x00, 0x04, 0x20, 0x00, 0x01, 0x78};
  receiver.Feed(Pair(neutral, neutral), 5);
  if (!receiver.snapshot().online ||
      receiver.snapshot().channel != std::array<double, 4>{} ||
      receiver.snapshot().switch_right != Switch::MIDDLE ||
      receiver.snapshot().switch_left != Switch::UP) {
    return false;
  }
  for (std::size_t channel = 0; channel < 4; ++channel) {
    for (const auto value : {364, 1024, 1684, 777, 1234}) {
      std::array<std::uint16_t, 4> raw{1024, 1024, 1024, 1024};
      raw[channel] = static_cast<std::uint16_t>(value);
      receiver.Feed(Encode(raw), 10);
      if (receiver.snapshot().channel_raw != raw) {
        return false;
      }
      for (std::size_t index = 0; index < 4; ++index) {
        const double expected = (static_cast<int>(raw[index]) - 1024) / 660.0;
        if (std::abs(receiver.snapshot().channel[index] - expected) > 1e-12) {
          return false;
        }
      }
    }
  }
  return receiver.snapshot().valid_frames == 22;
}

// 覆盖双拨杆全部9种组合；输入无，返回每个位域的准确映射。
bool TestSwitches() {
  Receiver receiver(std::chrono::milliseconds(100));
  for (const auto left : {Switch::UP, Switch::DOWN, Switch::MIDDLE}) {
    for (const auto right : {Switch::UP, Switch::DOWN, Switch::MIDDLE}) {
      const auto frame = Encode({1024, 1024, 1024, 1024}, left, right);
      receiver.Feed(Pair(frame, frame), 1);
      if (!receiver.snapshot().online ||
          receiver.snapshot().switch_left != left ||
          receiver.snapshot().switch_right != right) {
        return false;
      }
    }
  }
  return true;
}

// 覆盖所有切分位置和逐字节输入；输入无，返回组帧及两帧锁定结果。
bool TestSplits() {
  const auto first = Encode({364, 1684, 1024, 777});
  const auto last = Encode({1234, 1024, 1684, 364});
  const auto stream = Pair(first, last);
  for (std::size_t split = 1; split < stream.size(); ++split) {
    Receiver receiver(std::chrono::milliseconds(100));
    receiver.Feed(std::span(stream).first(split), 1);
    if (!IsSafe(receiver.snapshot())) {
      return false;
    }
    receiver.Feed(std::span(stream).subspan(split), 2);
    if (!receiver.snapshot().online || receiver.snapshot().valid_frames != 2 ||
        receiver.snapshot().channel_raw !=
            std::array<std::uint16_t, 4>{1234, 1024, 1684, 364}) {
      return false;
    }
  }
  Receiver receiver(std::chrono::milliseconds(100));
  for (const auto byte : stream) {
    receiver.Feed(std::span(&byte, 1), 3);
  }
  receiver.Feed(Pair(last, first), 4);
  receiver.Feed({}, 5);
  return receiver.snapshot().valid_frames == 4 &&
         receiver.snapshot().channel_raw ==
             std::array<std::uint16_t, 4>{364, 1684, 1024, 777} &&
         receiver.snapshot().discarded_bytes == 0 &&
         receiver.snapshot().resync_count == 0;
}

// 检查每通道越界、非法拨杆及按钮；输入无，返回拒绝坏帧且立即离线。
bool TestInvalid() {
  std::vector<Frame> invalid;
  for (std::size_t channel = 0; channel < 4; ++channel) {
    for (const auto value : {0, 363, 1685, 2047}) {
      std::array<std::uint16_t, 4> raw{1024, 1024, 1024, 1024};
      raw[channel] = static_cast<std::uint16_t>(value);
      invalid.push_back(Encode(raw));
    }
  }
  invalid.push_back(Encode({1024, 1024, 1024, 1024}, Switch::UNKNOWN));
  invalid.push_back(
      Encode({1024, 1024, 1024, 1024}, Switch::UP, Switch::UNKNOWN));
  for (const auto offset : {12, 13}) {
    auto frame = Encode();
    frame[offset] = 2;
    invalid.push_back(frame);
  }
  for (const auto& bad : invalid) {
    Receiver receiver(std::chrono::milliseconds(100));
    receiver.Feed(Pair(Encode(), Encode()), 1);
    receiver.Feed(bad, 2);
    if (!IsSafe(receiver.snapshot()) || receiver.snapshot().valid_frames != 2) {
      return false;
    }
  }
  Receiver receiver(std::chrono::milliseconds(100));
  receiver.Feed(Pair(Encode(), invalid.front()), 1);
  return IsSafe(receiver.snapshot()) && receiver.snapshot().valid_frames == 0;
}

// 注入/丢失各位置单字节后恢复；输入无，返回重新搜索后的最新通道。
bool TestResync() {
  const auto good = Encode({364, 1024, 1684, 777});
  for (std::size_t position = 0; position < 18; ++position) {
    for (const bool insert : {false, true}) {
      Receiver receiver(std::chrono::milliseconds(100));
      receiver.Feed(Pair(good, good), 1);
      std::vector<std::uint8_t> damaged(good.begin(), good.end());
      if (insert) {
        damaged.insert(damaged.begin() + position, 0xff);
      } else {
        damaged.erase(damaged.begin() + position);
      }
      receiver.Feed(damaged, 2);
      // 无CRC协议可能漏检部分损坏帧；后续完整帧应恢复真实边界。
      receiver.Feed(Pair(good, good), 3);
      receiver.Feed(Pair(good, good), 4);
      if (!receiver.snapshot().online ||
          receiver.snapshot().channel_raw !=
              std::array<std::uint16_t, 4>{364, 1024, 1684, 777}) {
        return false;
      }
    }
  }
  Receiver noise(std::chrono::milliseconds(100));
  const std::vector<std::uint8_t> junk(4096, 0xff);
  noise.Feed(junk, 1);
  noise.Feed(Pair(good, good), 2);
  return noise.snapshot().online && noise.snapshot().discarded_bytes == 4096;
}

// 验证精确超时和两帧恢复；输入无，返回清零、计数保留及重新锁定。
bool TestTimeout() {
  Receiver receiver(std::chrono::milliseconds(100));
  if (!IsSafe(receiver.snapshot())) {
    return false;
  }
  receiver.Feed(Pair(Encode(), Encode()), 50);
  receiver.Update(149);
  if (!receiver.snapshot().online) {
    return false;
  }
  receiver.Update(150);
  if (!IsSafe(receiver.snapshot()) || receiver.snapshot().valid_frames != 2 ||
      receiver.snapshot().last_valid_frame_ms != 50) {
    return false;
  }
  receiver.Feed(Encode(), 151);
  if (!IsSafe(receiver.snapshot())) {
    return false;
  }
  receiver.Feed(Encode(), 158);
  return receiver.snapshot().online && receiver.snapshot().valid_frames == 4 &&
         receiver.snapshot().last_valid_frame_ms == 158 &&
         receiver.snapshot().resync_count == 1;
}

// 检查旧候选/残包不能跨超时拼接；输入无，返回恢复及丢弃计数结果。
bool TestPartialTimeout() {
  const auto frame = Encode();
  for (std::size_t partial = 1; partial < 36; ++partial) {
    Receiver receiver(std::chrono::milliseconds(100));
    const auto stream = Pair(frame, frame);
    receiver.Feed(std::span(stream).first(partial), 1);
    receiver.Update(101);
    if (!IsSafe(receiver.snapshot()) ||
        receiver.snapshot().discarded_bytes != partial) {
      return false;
    }
    receiver.Feed(frame, 102);
    if (!IsSafe(receiver.snapshot())) {
      return false;
    }
    receiver.Feed(frame, 110);
    if (!receiver.snapshot().online || receiver.snapshot().valid_frames != 2) {
      return false;
    }
  }
  Receiver receiver(std::chrono::milliseconds(100));
  receiver.Feed(Pair(frame, frame), 1);
  receiver.Feed(frame, 101);
  return IsSafe(receiver.snapshot()) && receiver.snapshot().valid_frames == 2;
}

// 检查HAL tick回绕及外部失效；输入无，返回超时边界和残包清退结果。
bool TestWrapAndReset() {
  Receiver receiver(std::chrono::milliseconds(100));
  receiver.Feed(Pair(Encode(), Encode()), UINT32_MAX - 50);
  receiver.Update(48);
  if (!receiver.snapshot().online) {
    return false;
  }
  receiver.Update(49);
  if (!IsSafe(receiver.snapshot())) {
    return false;
  }
  receiver.Feed(Pair(Encode(), Encode()), 50);
  const auto frame = Encode();
  receiver.Feed(std::span(frame).first(5), 51);
  receiver.Invalidate();
  return IsSafe(receiver.snapshot()) && receiver.snapshot().valid_frames == 4 &&
         receiver.snapshot().discarded_bytes == 5 &&
         receiver.snapshot().resync_count == 1;
}

}  // namespace

// 按CTest用例执行；输入用例名，输出非零退出码报告失败，Release同样有效。
int main(int argc, char** argv) {
  struct Test {
    std::string_view name;
    bool (*run)();
  };
  const std::array tests{Test{"channels", TestChannels},
                         Test{"switches", TestSwitches},
                         Test{"splits", TestSplits},
                         Test{"invalid", TestInvalid},
                         Test{"resync", TestResync},
                         Test{"timeout", TestTimeout},
                         Test{"partial_timeout", TestPartialTimeout},
                         Test{"wrap_reset", TestWrapAndReset}};
  for (const auto& test : tests) {
    if (argc == 2 && test.name == argv[1]) {
      const bool passed = test.run();
      std::printf("%.*s: %s\n", static_cast<int>(test.name.size()),
                  test.name.data(), passed ? "PASS" : "FAIL");
      return passed ? 0 : 1;
    }
  }
  return 2;
}
