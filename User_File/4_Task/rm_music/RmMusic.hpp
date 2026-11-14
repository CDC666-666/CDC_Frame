#ifndef FRAME_CDC_4_TASK_RM_MUSIC_RM_MUSIC_H_
#define FRAME_CDC_4_TASK_RM_MUSIC_RM_MUSIC_H_

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: One-shot startup music using the MC02 buzzer component
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "music_player.h"
#include "pwm.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

// 全局类名/同名主头文件是固定XRobot模块解析接口，任务源码位于rm_music。
class RmMusic {
 public:
  // 由官方入口构造一次；借用PWM直到复位，enabled决定是否上电播放。
  // 栈大小以字节计；构造只创建低优先级线程，不阻塞后续模块装配。
  explicit RmMusic(LibXR::PWM& pwm, bool enabled = true,
                   std::uint32_t task_stack_depth = 2048);

  RmMusic(const RmMusic&) = delete;
  RmMusic& operator=(const RmMusic&) = delete;

  // 返回线程状态快照；输入无，输出含禁用、缺谱、无效谱及完成状态。
  frame_cdc::rm_music::PlaybackStatus status() const;

  // 官方低频监视接口；无输入输出，播放由独立线程执行。
  void OnMonitor();

 private:
  // 线程入口；输入生命周期覆盖线程的自身指针，播放一次后永久挂起。
  static void Run(RmMusic* self);

  Class_Buzzer buzzer_;
  LibXR::Semaphore finished_{0};
  LibXR::Thread thread_;
  std::atomic<frame_cdc::rm_music::PlaybackStatus> status_{
      frame_cdc::rm_music::PlaybackStatus::DISABLED};
};

#endif  // FRAME_CDC_4_TASK_RM_MUSIC_RM_MUSIC_H_
