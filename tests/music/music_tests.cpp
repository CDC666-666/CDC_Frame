#include <array>
#include <chrono>
#include <cstdio>
#include <vector>

#include "music_player.h"
#include "rm_music_score.h"

namespace {

// 记录播放器可观察的声音输出；输入Hz和响度，输出事件序列。
class RecordingBuzzer {
 public:
  void Set_Sound(float frequency_hz, float loudness) {
    outputs.push_back(loudness == 0.0f ? 0 : frequency_hz);
  }

  std::vector<float> outputs;
};

// 校验播放契约；输入无，输出进程退出码，避免Release禁用assert影响验证。
bool CheckPlayback() {
  using frame_cdc::rm_music::Note;
  using frame_cdc::rm_music::PlaybackStatus;
  using frame_cdc::rm_music::PlayScore;
  // 自制测试音阶，仅用于行为验证，不作为目标曲目。
  constexpr std::array<Note, 3> notes{{{440, 100}, {0, 50}, {523, 200}}};
  RecordingBuzzer buzzer;
  std::vector<std::chrono::milliseconds> waits;
  auto wait = [&waits](std::chrono::milliseconds duration) {
    waits.push_back(duration);
  };
  if (PlayScore(false, notes, buzzer, wait) != PlaybackStatus::DISABLED ||
      !waits.empty() || buzzer.outputs != std::vector<float>{0}) {
    return false;
  }
  buzzer.outputs.clear();
  if (PlayScore(true, notes, buzzer, wait) != PlaybackStatus::DONE ||
      buzzer.outputs != std::vector<float>{0, 440, 0, 523, 0} ||
      waits != std::vector<std::chrono::milliseconds>{
                   std::chrono::milliseconds(100),
                   std::chrono::milliseconds(50),
                   std::chrono::milliseconds(200)}) {
    return false;
  }
  for (const Note invalid :
       {Note{440, 0}, Note{20001, 100}, Note{440, 60001}}) {
    // 即使前一个音符有效，无效谱也必须整体拒绝，不能提前发出第一个音。
    const std::array<Note, 2> score{{{440, 100}, invalid}};
    buzzer.outputs.clear();
    waits.clear();
    if (PlayScore(true, score, buzzer, wait) != PlaybackStatus::INVALID_SCORE ||
        !waits.empty() || buzzer.outputs != std::vector<float>{0}) {
      return false;
    }
  }
  buzzer.outputs.clear();
  return PlayScore(true, {}, buzzer, wait) == PlaybackStatus::MISSING_SCORE &&
         waits.empty() && buzzer.outputs == std::vector<float>{0};
}

// 用实际生成的曲谱验证合法性、总等待时长及末次静音；不等待真实时间。
// 输入无，输出完整谱能否通过相同播放器，并保留关闭开关的零等待行为。
bool CheckImportedScore() {
  RecordingBuzzer buzzer;
  std::chrono::milliseconds duration{0};
  auto wait = [&duration](std::chrono::milliseconds value) {
    duration += value;
  };
  const auto& score = frame_cdc::rm_music::score;
  const auto result = frame_cdc::rm_music::PlayScore(true, score, buzzer, wait);
  if (result != frame_cdc::rm_music::PlaybackStatus::DONE ||
      duration.count() <= 0 || buzzer.outputs.back() != 0) {
    return false;
  }
  std::chrono::milliseconds expected{0};
  for (const auto& note : score) {
    expected += std::chrono::milliseconds(note.duration_ms);
  }
  if (duration != expected || buzzer.outputs.size() != score.size() + 2) {
    return false;
  }
  buzzer.outputs.clear();
  duration = std::chrono::milliseconds(0);
  return frame_cdc::rm_music::PlayScore(false, score, buzzer, wait) ==
             frame_cdc::rm_music::PlaybackStatus::DISABLED &&
         duration.count() == 0 && buzzer.outputs == std::vector<float>{0};
}

}  // namespace

// 运行独立播放契约；输入无，成功返回0，失败报告并返回1。
int main() {
  if (!CheckPlayback() || !CheckImportedScore()) {
    std::fputs("music playback contract failed\n", stderr);
    return 1;
  }
  std::puts("music playback contract passed");
  return 0;
}
