#ifndef FRAME_CDC_4_TASK_RM_MUSIC_MUSIC_PLAYER_H_
#define FRAME_CDC_4_TASK_RM_MUSIC_MUSIC_PLAYER_H_

#include <chrono>
#include <cstdint>
#include <span>

namespace frame_cdc::rm_music {

// 乐谱存储格式：频率0表示休止；位宽覆盖20kHz和单音60s，减少Flash占用。
struct Note {
  std::uint16_t frequency_hz;
  std::uint32_t duration_ms;
};

enum class PlaybackStatus {
  DISABLED,
  MISSING_SCORE,
  INVALID_SCORE,
  PLAYING,
  DONE
};

// 检查整份乐谱再发声；输入只读音符和开关，输出可播放或拒绝原因。
inline PlaybackStatus CheckScore(bool enabled, std::span<const Note> score) {
  if (!enabled) {
    return PlaybackStatus::DISABLED;
  }
  if (score.empty()) {
    return PlaybackStatus::MISSING_SCORE;
  }
  for (const auto& note : score) {
    if (note.frequency_hz > 20000 || note.duration_ms == 0 ||
        note.duration_ms > 60000) {
      return PlaybackStatus::INVALID_SCORE;
    }
  }
  return PlaybackStatus::PLAYING;
}

// 在线程中播放一次；借用蜂鸣器和等待函数，等待期间乐谱必须保持有效。
// wait接收chrono毫秒并阻塞当前线程；返回状态，所有结束路径都静音。
template <typename Buzzer, typename Wait>
PlaybackStatus PlayScore(bool enabled, std::span<const Note> score,
                         Buzzer& buzzer, Wait wait) {
  buzzer.Set_Sound(0.0f, 0.0f);
  const auto status = CheckScore(enabled, score);
  if (status != PlaybackStatus::PLAYING) {
    return status;
  }
  for (const auto& note : score) {
    // float仅用于既有蜂鸣器接口边界；整Hz/响度0.01可满足该接口精度。
    buzzer.Set_Sound(static_cast<float>(note.frequency_hz), 0.01f);
    wait(std::chrono::milliseconds(note.duration_ms));
  }
  buzzer.Set_Sound(0.0f, 0.0f);
  return PlaybackStatus::DONE;
}

}  // namespace frame_cdc::rm_music

#endif  // FRAME_CDC_4_TASK_RM_MUSIC_MUSIC_PLAYER_H_
