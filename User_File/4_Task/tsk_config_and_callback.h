#ifndef FRAME_CDC_TASK_CONFIG_AND_CALLBACK_H_
#define FRAME_CDC_TASK_CONFIG_AND_CALLBACK_H_

#include <atomic>
#include <cstdint>

#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Driver/ADC/drv_adc.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/SPI/drv_spi.h"
#include "1_Middleware/Driver/USB/drv_usb.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Plotter/Serialplot/dvc_serialplot.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"
#include "2_Device/Powermeter/dvc_powermeter.h"
#include "gpio.hpp"
#include "ramfs.hpp"
#include "terminal.hpp"
#include "thread.hpp"

namespace frame_cdc {

struct ReferenceHardware {
  LibXR::CAN& can;
  LibXR::UART& uart;
  LibXR::UART& usb;
  LibXR::SPI& imu_spi;
  LibXR::SPI& led_spi;
  LibXR::ADC& adc;
  LibXR::PWM& heater;
  LibXR::PWM& buzzer;
  LibXR::GPIO& accel_interrupt;
  LibXR::GPIO& gyro_interrupt;
  OSPI_HandleTypeDef& ospi;
  LibXR::RamFS& ramfs;
  LibXR::Terminal<64, 96, 8, 5>& terminal;
};

// 参考演示的实例所有者；命令只排队，设备工作在线程执行。
class ReferenceApplication {
 public:
  // 保存硬件引用并构造长期状态；输入硬件，无外设启动动作。
  explicit ReferenceApplication(ReferenceHardware hardware);
  ReferenceApplication(const ReferenceApplication&) = delete;
  ReferenceApplication& operator=(const ReferenceApplication&) = delete;
  // 绑定接口及注册ref命令；无输入，输出启动是否成功。
  bool Start();

 private:
  enum class Action : uint8_t {
    POWER,
    LED,
    BUZZER,
    MOTOR,
    MOTOR_ANGLE,
    MOTOR_OMEGA,
    MOTOR_STOP,
    FLASH_ID,
    FLASH_READ,
    FLASH_WRITE,
    FLASH_ERASE,
    FLASH_MAP,
    FLASH_QUAD,
    PLOT
  };
  struct Request {
    Action action;
    uint32_t args[3]{};
  };
  struct ImuEvent {
    uint16_t pin;
    uint64_t timestamp_us;
  };
  struct Attitude {
    float quaternion[4]{};
    float euler[3]{};                // 参考传感器坐标，yaw/pitch/roll，rad。
    float reference_channels[23]{};  // 仅遥测边界沿用参考字段及deg/us单位。
  };
  // 解析终端请求；输入参数，返回命令状态，硬件动作由队列串行执行。
  static int Command(ReferenceApplication* self, int argc, char** argv);
  // 服务设备与周期任务；输入本对象，无输出，永久运行。
  static void IoThread(ReferenceApplication* self);
  // 服务IMU初始化和事件驱动解算；输入本对象，无输出。
  static void ImuThread(ReferenceApplication* self);
  // 保存EXTI时间；输入中断标识及上下文，输出有界事件交接。
  void Interrupt(uint16_t pin, bool in_isr);
  // 执行排队请求；输入命令，输出设备状态到终端。
  void Execute(const Request& request);
  // 复制姿态快照；无输入，返回线程一致的样本。
  Attitude GetAttitude();
  // 修改参考Kalman噪声参数；输入索引及有限正值，输出矩阵更新。
  void Tune(int32_t index, float value);

  ReferenceHardware hardware_;
  Struct_CAN_Manage_Object can_;
  Struct_UART_Manage_Object uart_;
  Struct_USB_Manage_Object usb_;
  Struct_SPI_Manage_Object imu_spi_;
  Struct_SPI_Manage_Object led_spi_;
  Struct_ADC_Manage_Object adc_;
  Struct_OSPI_Manage_Object ospi_;
  Class_BMI088 imu_{};
  Class_Buzzer buzzer_{};
  Class_Key key_{};
  Class_Power power_{};
  Class_WS2812 led_{};
  Class_W25Q64JV flash_{};
  Class_Motor_DJI_GM6020 motor_{};
  Class_Vofa_USB vofa_usb_{};
  Class_Vofa_UART vofa_uart_{};
  Class_Serialplot_UART serialplot_{};
  Class_Powermeter_UART powermeter_{};
  StreamFramer uart_frames_;
  UsbInputRouter usb_router_;
  LibXR::ReadPort console_input_{512};
  LibXR::Thread console_thread_;
  const char* tune_names_[4]{"q00", "q11", "r00", "r11"};
  // 原参考矩阵算法为float，保持精度及原API；状态仅由IO线程访问。
  Class_Filter_Kalman<2, 1, 2> kalman_;
  std::atomic<uint32_t> console_dropped_{0};
  LibXR::SPSCQueue<Request> requests_{8};
  LibXR::SPSCQueue<ImuEvent> imu_events_{32};
  LibXR::Semaphore imu_ready_;
  LibXR::Mutex attitude_mutex_;
  Attitude attitude_{};
  LibXR::Thread io_thread_;
  LibXR::Thread imu_thread_;
  LibXR::RamFS::File command_;
  std::atomic<bool> imu_requested_{false};
  std::atomic<bool> heater_requested_{false};
  std::atomic<uint32_t> imu_state_{0};
  std::atomic<uint32_t> imu_dropped_{0};
  std::atomic<float> voltage_{0};
  std::atomic<uint32_t> key_state_{0};
  std::atomic<uint32_t> meter_state_{0};
  std::atomic<float> meter_voltage_{0};
  std::atomic<float> meter_current_{0};
  std::atomic<float> meter_power_{0};
  bool motor_enabled_ = false;
  bool flash_ready_ = false;
  bool uart_power_mode_ = false;
  uint32_t io_ticks_ = 0;
  uint32_t plot_remaining_ = 0;
  uint32_t plot_mode_ = 0;
  uint64_t buzzer_available_us_ = 0;
};

}  // namespace frame_cdc
#endif
