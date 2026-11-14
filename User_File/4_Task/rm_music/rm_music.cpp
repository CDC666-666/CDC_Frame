// XRobot固定主头文件名，任务源文件按项目小写命名；保留对应头文件优先。
// clang-format off
#include "RmMusic.hpp"
// clang-format on

#include <chrono>
#include <cstdint>

#include "rm_music_score.h"
#include "timebase.hpp"

// 初始化静音并检查开关/乐谱；只有有效谱才创建播放线程。
// 输入PWM、布尔开关和栈字节数，输出通过status()观察。
RmMusic::RmMusic(LibXR::PWM& pwm, bool enabled,
                 std::uint32_t task_stack_depth) {
  buzzer_.Init(&pwm, 4000.0f, 0.0f);
  const auto state =
      frame_cdc::rm_music::CheckScore(enabled, frame_cdc::rm_music::score);
  status_.store(state);
  if (state == frame_cdc::rm_music::PlaybackStatus::PLAYING) {
    thread_.Create(this, Run, "rm_music", task_stack_depth,
                   LibXR::Thread::Priority::LOW);
  }
}

// 获取跨线程状态快照；输入无，输出当前播放状态。
frame_cdc::rm_music::PlaybackStatus RmMusic::status() const {
  return status_.load();
}

// 保留官方监视入口；输入输出均无，不在监视回调内等待音符。
void RmMusic::OnMonitor() {}

// 在LibXR毫秒时钟上保持累计节拍；输入自身，输出最终状态并静音。
void RmMusic::Run(RmMusic* self) {
  auto wake_time = LibXR::Timebase::GetMilliseconds();
  const auto result = frame_cdc::rm_music::PlayScore(
      true, frame_cdc::rm_music::score, self->buzzer_,
      [&wake_time](std::chrono::milliseconds duration) {
        // 乐谱校验保证1..60000ms；在RTOS边界显式转换位宽。
        LibXR::Thread::SleepUntil(wake_time,
                                  static_cast<std::uint32_t>(duration.count()));
      });
  self->status_.store(result);
  // 固定LibXR的线程包装不支持返回后删除任务；永久阻塞保留有效实例。
  while (true) {
    self->finished_.Wait(UINT32_MAX);
  }
}
