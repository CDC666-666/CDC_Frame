#include "tsk_config_and_callback.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "FreeRTOS.h"
#include "main.h"

namespace frame_cdc {

// 保存硬件借用及命令节点；输入硬件，输出未启动的应用实例。
ReferenceApplication::ReferenceApplication(ReferenceHardware hardware)
    : hardware_(hardware),
      command_(LibXR::RamFS::CreateCommand("ref", Command, this)) {}

// 装配接口、参数及线程；无输入，返回绑定/中断初始化结果。
bool ReferenceApplication::Start() {
  // 原有BuzzerAlarm随后播放约800ms开机音；延后自有蜂鸣器命令避免竞争。
  buzzer_available_us_ = Class_Timestamp::Get_Now_Microsecond() + 2000000;
  if (!can_.Bind(hardware_.can) || !uart_.Bind(hardware_.uart) ||
      !usb_.Bind(hardware_.usb)) {
    return false;
  }
  imu_spi_.Bind(hardware_.imu_spi);
  imu_spi_.power_adc = &hardware_.adc;
  imu_spi_.heater_pwm = &hardware_.heater;
  led_spi_.Bind(hardware_.led_spi);
  adc_.channel = &hardware_.adc;
  ospi_.handle = &hardware_.ospi;
  key_.Init();
  power_.Init(&adc_, false, false, false);
  led_.Init(&led_spi_);
  vofa_usb_.Init(&usb_, 4, tune_names_);
  vofa_uart_.Init(&uart_, 4, tune_names_);
  serialplot_.Init(&uart_);
  powermeter_.Init(&uart_);
  motor_.Init(&can_, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_TORQUE, 0,
              PI / 6);
  motor_.PID_Angle.Init(12.0f, 0, 0, 0, 10, 10);
  motor_.PID_Omega.Init(0.03f, 5, 0, 0, 0.2f, 0.2f);
  auto a = Namespace_ALG_Matrix::Identity<2, 2>();
  auto b = Namespace_ALG_Matrix::Zero<2, 1>();
  const auto h = Namespace_ALG_Matrix::Identity<2, 2>();
  auto q = Namespace_ALG_Matrix::Zero<2, 2>();
  auto r = Namespace_ALG_Matrix::Zero<2, 2>();
  a[0][1] = 0.001f;
  q[0][0] = r[0][0] = 0.001f;
  q[1][1] = 0.1f;
  r[1][1] = 1.0f;
  kalman_.Init(a, b, h, q, r);
  can_.on_receive = LibXR::Callback<>::Create(
      [](bool, ReferenceApplication* self) {
        if (self->can_.Rx_Header.Identifier == 0x206) {
          self->motor_.CAN_RxCpltCallback();
        }
      },
      this);
  imu_spi_.on_receive = LibXR::Callback<>::Create(
      [](bool, ReferenceApplication* self) { self->imu_.SPI_RxCpltCallback(); },
      this);
  uart_.on_receive = LibXR::Callback<const uint8_t*, uint16_t>::Create(
      [](bool, ReferenceApplication* self, const uint8_t* data, uint16_t size) {
        self->uart_frames_.Feed(
            data, size, [self](const uint8_t* frame, uint16_t length) {
              if (self->uart_power_mode_) {
                self->powermeter_.UART_RxCpltCallback(frame, length);
              } else {
                self->vofa_uart_.UART_RxCpltCallback(frame, length);
                self->Tune(self->vofa_uart_.Get_Variable_Index(),
                           self->vofa_uart_.Get_Variable_Value());
              }
            });
      },
      this);
  usb_.on_receive = LibXR::Callback<const uint8_t*, uint16_t>::Create(
      [](bool, ReferenceApplication* self, const uint8_t* data, uint16_t size) {
        self->usb_router_.Feed(
            data, size,
            [self](const uint8_t* bytes, size_t count) {
              auto queue = self->console_input_.GetReadQueue();
              if (queue.PushBatch(bytes, count) != LibXR::ErrorCode::OK) {
                self->console_dropped_.fetch_add(count);
              }
              queue.Publish();
            },
            [self](const uint8_t* bytes, size_t count) {
              self->vofa_usb_.USB_RxCallback(bytes, count);
              self->Tune(self->vofa_usb_.Get_Variable_Index(),
                         self->vofa_usb_.Get_Variable_Value());
            });
      },
      this);
  const auto accel_result =
      hardware_.accel_interrupt.RegisterCallback(LibXR::GPIO::Callback::Create(
          [](bool in_isr, ReferenceApplication* self) {
            self->Interrupt(BMI088_ACCEL__INTERRUPT_Pin, in_isr);
          },
          this));
  const auto gyro_result =
      hardware_.gyro_interrupt.RegisterCallback(LibXR::GPIO::Callback::Create(
          [](bool in_isr, ReferenceApplication* self) {
            self->Interrupt(BMI088_GYRO__INTERRUPT_Pin, in_isr);
          },
          this));
  if (accel_result != LibXR::ErrorCode::OK ||
      gyro_result != LibXR::ErrorCode::OK ||
      hardware_.accel_interrupt.EnableInterrupt() != LibXR::ErrorCode::OK ||
      hardware_.gyro_interrupt.EnableInterrupt() != LibXR::ErrorCode::OK) {
    hardware_.accel_interrupt.DisableInterrupt();
    hardware_.gyro_interrupt.DisableInterrupt();
    return false;
  }
  hardware_.ramfs.Add(command_);
  // 生成配置禁止自动终端线程；首次读取前改接逻辑输入，CDC只由IO读取。
  hardware_.terminal.read_port_ = &console_input_;
  LibXR::STDIO::read_ = &console_input_;
  console_thread_.Create(&hardware_.terminal, hardware_.terminal.ThreadFun,
                         "reference_console", 2048,
                         LibXR::Thread::Priority::MEDIUM);
  io_thread_.Create(this, IoThread, "reference_io", 4096,
                    LibXR::Thread::Priority::MEDIUM);
  imu_thread_.Create(this, ImuThread, "reference_imu", 8192,
                     LibXR::Thread::Priority::REALTIME);
  return true;
}

// 更新四个参考噪声参数；输入索引和值，非法值保持原矩阵。
void ReferenceApplication::Tune(int32_t index, float value) {
  if (!std::isfinite(value) || value <= 0 || value > 1e6f) {
    return;
  }
  if (index == 0) {
    kalman_.Matrix_Q[0][0] = value;
  } else if (index == 1) {
    kalman_.Matrix_Q[1][1] = value;
  } else if (index == 2) {
    kalman_.Matrix_R[0][0] = value;
  } else if (index == 3) {
    kalman_.Matrix_R[1][1] = value;
  }
}

// ISR捕获时间并交接事件；输入引脚/上下文，满队列只计数。
void ReferenceApplication::Interrupt(uint16_t pin, bool in_isr) {
  if (imu_state_.load(std::memory_order_relaxed) != 2) {
    return;
  }
  const ImuEvent event{pin, LibXR::Timebase::GetMicroseconds()};
  if (imu_events_.Push(event) != LibXR::ErrorCode::OK) {
    imu_dropped_.fetch_add(1, std::memory_order_relaxed);
  }
  imu_ready_.PostFromCallback(in_isr);
}

// 获取一致姿态副本；无输入，返回传感器参考坐标下的原精度值。
ReferenceApplication::Attitude ReferenceApplication::GetAttitude() {
  attitude_mutex_.Lock();
  const auto sample = attitude_;
  attitude_mutex_.Unlock();
  return sample;
}

// 解析参数并排队；输入终端参数，返回接受状态，不直接访问设备状态。
int ReferenceApplication::Command(ReferenceApplication* self, int argc,
                                  char** argv) {
  if (argc < 2 || std::strcmp(argv[1], "status") == 0) {
    std::printf(
        "ref: imu=%lu dropped=%lu can_drop=%lu voltage=%f V console_drop=%lu\n",
        static_cast<unsigned long>(self->imu_state_.load()),
        static_cast<unsigned long>(self->imu_dropped_.load()),
        static_cast<unsigned long>(self->can_.dropped.load()),
        static_cast<double>(self->voltage_.load()),
        static_cast<unsigned long>(self->console_dropped_.load()));
    std::printf(
        "ref imu|heater on/off; led r g b; power mask; buzzer Hz percent;\n"
        "ref motor signed_torque_mNm; angle signed_mrad; omega signed_mrad_s; "
        "stop;\n"
        "ref flash-id; read addr len; write addr byte;\n"
        "ref erase sector_addr; map 0/1; quad; plot usb|vofa|serial|kalman|imu "
        "count; uart-power 0/1\n"
        "USB/UART tuning: q00=0.001# q11=0.1# r00=0.001# r11=1# (each without "
        "leading spaces)\n");
    std::printf("tx rejected CAN=%lu UART=%lu USB=%lu; heap_free=%lu B\n",
                static_cast<unsigned long>(self->can_.transmit_failed.load()),
                static_cast<unsigned long>(self->uart_.transmit_failed.load()),
                static_cast<unsigned long>(self->usb_.transmit_failed.load()),
                static_cast<unsigned long>(xPortGetFreeHeapSize()));
    std::printf("key=%lu meter=%lu: %f V %f A %f W\n",
                static_cast<unsigned long>(self->key_state_.load()),
                static_cast<unsigned long>(self->meter_state_.load()),
                static_cast<double>(self->meter_voltage_.load()),
                static_cast<double>(self->meter_current_.load()),
                static_cast<double>(self->meter_power_.load()));
    return 0;
  }
  if (argc == 3 && (std::strcmp(argv[1], "imu") == 0 ||
                    std::strcmp(argv[1], "heater") == 0)) {
    if (std::strcmp(argv[2], "on") != 0 && std::strcmp(argv[2], "off") != 0) {
      return -1;
    }
    const bool enabled = std::strcmp(argv[2], "on") == 0;
    if (std::strcmp(argv[1], "imu") == 0) {
      self->imu_requested_.store(enabled);
    } else {
      self->heater_requested_.store(enabled);
    }
    self->imu_ready_.Post();
    return 0;
  }
  if (argc == 3 && std::strcmp(argv[1], "uart-power") == 0) {
    // 与工作线程共享的模式也通过队列切换，复用POWER动作的专用标志。
    Request request{Action::POWER, {0, 1, 0}};
    if (std::strcmp(argv[2], "0") != 0 && std::strcmp(argv[2], "1") != 0) {
      return -1;
    }
    request.args[0] = argv[2][0] - '0';
    return self->requests_.Push(request) == LibXR::ErrorCode::OK ? 0 : -1;
  }
  Request request{};
  const char* name = argv[1];
  int first_number = 2;
  if (std::strcmp(name, "led") == 0 && argc == 5) {
    request.action = Action::LED;
  } else if (std::strcmp(name, "power") == 0 && argc == 3) {
    request.action = Action::POWER;
  } else if (std::strcmp(name, "buzzer") == 0 && argc == 4) {
    request.action = Action::BUZZER;
  } else if (std::strcmp(name, "motor") == 0 && argc == 3) {
    request.action = Action::MOTOR;
  } else if (std::strcmp(name, "angle") == 0 && argc == 3) {
    request.action = Action::MOTOR_ANGLE;
  } else if (std::strcmp(name, "omega") == 0 && argc == 3) {
    request.action = Action::MOTOR_OMEGA;
  } else if (std::strcmp(name, "stop") == 0 && argc == 2) {
    request.action = Action::MOTOR_STOP;
  } else if (std::strcmp(name, "flash-id") == 0 && argc == 2) {
    request.action = Action::FLASH_ID;
  } else if (std::strcmp(name, "read") == 0 && argc == 4) {
    request.action = Action::FLASH_READ;
  } else if (std::strcmp(name, "write") == 0 && argc == 4) {
    request.action = Action::FLASH_WRITE;
  } else if (std::strcmp(name, "erase") == 0 && argc == 3) {
    request.action = Action::FLASH_ERASE;
  } else if (std::strcmp(name, "map") == 0 && argc == 3) {
    request.action = Action::FLASH_MAP;
  } else if (std::strcmp(name, "quad") == 0 && argc == 2) {
    request.action = Action::FLASH_QUAD;
  } else if (std::strcmp(name, "plot") == 0 && argc == 4) {
    request.action = Action::PLOT;
    if (std::strcmp(argv[2], "usb") == 0) {
      request.args[1] = 0;
    } else if (std::strcmp(argv[2], "vofa") == 0) {
      request.args[1] = 1;
    } else if (std::strcmp(argv[2], "serial") == 0) {
      request.args[1] = 2;
    } else if (std::strcmp(argv[2], "kalman") == 0) {
      request.args[1] = 3;
    } else if (std::strcmp(argv[2], "imu") == 0) {
      request.args[1] = 4;
    } else {
      return -1;
    }
    first_number = 3;
  } else {
    return -1;
  }
  for (int index = first_number; index < argc; ++index) {
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(argv[index], &end, 0);
    if (errno == ERANGE || end == argv[index] || *end != 0) {
      return -1;
    }
    if (request.action == Action::MOTOR ||
        request.action == Action::MOTOR_ANGLE ||
        request.action == Action::MOTOR_OMEGA) {
      const long limit = request.action == Action::MOTOR_ANGLE ? 12566 : 1000;
      if (value < -limit || value > limit) {
        return -1;
      }
      request.args[0] = static_cast<uint32_t>(value < 0 ? -value : value);
      request.args[1] = value < 0;
      break;
    }
    if (value < 0) {
      return -1;
    }
    request.args[index - first_number] = static_cast<uint32_t>(value);
  }
  return self->requests_.Push(request) == LibXR::ErrorCode::OK ? 0 : -1;
}

// 在IO线程串行执行请求；输入有界命令，输出操作结果及设备状态。
void ReferenceApplication::Execute(const Request& request) {
  const auto* args = request.args;
  if (motor_enabled_ && request.action >= Action::FLASH_ID &&
      request.action <= Action::FLASH_QUAD) {
    std::printf("Stop motor before Flash maintenance.\n");
    return;
  }
  switch (request.action) {
    case Action::POWER:
      if (args[1] == 1) {
        uart_power_mode_ = args[0] != 0;
        uart_frames_.Reset(uart_power_mode_ ? StreamFramer::Mode::POWERMETER
                                            : StreamFramer::Mode::TEXT);
      } else if (args[0] < 8) {
        power_.Set_DC24_0(args[0] & 1);
        power_.Set_DC24_1(args[0] & 2);
        power_.Set_DC5(args[0] & 4);
      }
      break;
    case Action::LED:
      if (args[0] < 256 && args[1] < 256 && args[2] < 256) {
        led_.Set_RGB(args[0], args[1], args[2]);
        led_.TIM_10ms_Write_PeriodElapsedCallback();
      }
      break;
    case Action::BUZZER:
      if (Class_Timestamp::Get_Now_Microsecond() >= buzzer_available_us_ &&
          args[0] <= 20000 && args[1] <= 100) {
        buzzer_.Init(&hardware_.buzzer, args[0], args[1] / 100.0f);
      }
      break;
    case Action::MOTOR:
    case Action::MOTOR_ANGLE:
    case Action::MOTOR_OMEGA: {
      const float value = (args[1] ? -1.0f : 1.0f) * args[0] / 1000.0f;
      if (request.action == Action::MOTOR) {
        motor_.Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
        motor_.Set_Target_Torque(value);
      } else if (request.action == Action::MOTOR_ANGLE) {
        motor_.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        motor_.Set_Target_Angle(value);
      } else {
        motor_.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        motor_.Set_Target_Omega(value);
      }
      motor_enabled_ = true;
      break;
    }
    case Action::MOTOR_STOP:
      motor_.Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
      motor_.Set_Target_Torque(0);
      motor_.TIM_Calculate_PeriodElapsedCallback();
      can_.Send(0x1fe, can_.Group(0x1fe), 8);
      motor_enabled_ = false;
      break;
    case Action::FLASH_ID:
      if (ospi_.memory_mapped) {
        std::printf("Unmap Flash before identification.\n");
        break;
      }
      flash_ready_ = flash_.Init(&ospi_);
      std::printf("flash JEDEC=%06lx ready=%d\n",
                  static_cast<unsigned long>(flash_.jedec_id()), flash_ready_);
      break;
    case Action::FLASH_READ:
      if (flash_ready_ && args[1] > 0 && args[1] <= 255 &&
          flash_.Get_Buffer(args[0], args[1])) {
        for (uint32_t index = 0; index < args[1]; ++index) {
          std::printf("%02x", ospi_.Rx_Buffer[index]);
        }
        std::printf("\n");
      }
      break;
    case Action::FLASH_WRITE:
      if (flash_ready_ && args[1] < 256) {
        const uint8_t byte = args[1];
        std::printf("flash write=%d\n", flash_.Set_Buffer(&byte, args[0], 1));
      }
      break;
    case Action::FLASH_ERASE:
      if (flash_ready_) {
        std::printf("flash erase=%d\n", flash_.Set_Sector_Erased(args[0]));
      }
      break;
    case Action::FLASH_MAP:
      if (flash_ready_) {
        std::printf("flash map=%d\n", flash_.SetMemoryMapped(args[0] != 0));
      }
      break;
    case Action::FLASH_QUAD:
      if (flash_ready_) {
        std::printf("flash quad=%d\n", flash_.EnableQuad());
      }
      break;
    case Action::PLOT:
      plot_remaining_ = std::min<uint32_t>(args[0], 10000);
      plot_mode_ = args[1];
      break;
  }
}

// 消费通信及演示服务；输入应用对象，输出状态快照，永久循环。
void ReferenceApplication::IoThread(ReferenceApplication* self) {
  uint64_t previous_us = LibXR::Timebase::GetMicroseconds();
  for (;;) {
    self->can_.Process();
    self->uart_.Process();
    self->usb_.Process();
    const uint64_t now_us = LibXR::Timebase::GetMicroseconds();
    const float dt_s = static_cast<float>(now_us - previous_us) * 1e-6f;
    previous_us = now_us;
    self->kalman_.Matrix_A[0][1] = dt_s;
    self->kalman_.Vector_Z[0][0] = self->motor_.Get_Now_Angle();
    self->kalman_.Vector_Z[1][0] = self->motor_.Get_Now_Omega();
    self->kalman_.TIM_Predict_PeriodElapsedCallback();
    self->kalman_.TIM_Update_PeriodElapsedCallback();
    self->voltage_.store(self->power_.Get_Power_Voltage());
    self->key_.TIM_1ms_Process_PeriodElapsedCallback();
    self->key_state_.store(self->key_.Get_Key_Status());
    self->meter_state_.store(self->powermeter_.Get_Status());
    self->meter_voltage_.store(self->powermeter_.Get_Voltage());
    self->meter_current_.store(self->powermeter_.Get_Current());
    self->meter_power_.store(self->powermeter_.Get_Power());
    if (++self->io_ticks_ % 50 == 0) {
      self->key_.TIM_50ms_Read_PeriodElapsedCallback();
    }
    if (self->io_ticks_ % 100 == 0) {
      self->motor_.TIM_100ms_Alive_PeriodElapsedCallback();
      self->powermeter_.TIM_100ms_Alive_PeriodElapsedCallback();
    }
    if (self->motor_enabled_) {
      if (self->motor_.Get_Status() != Motor_DJI_Status_ENABLE) {
        self->motor_.Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
        self->motor_.Set_Target_Torque(0);
        self->motor_enabled_ = false;
      }
      self->motor_.TIM_Calculate_PeriodElapsedCallback();
      self->can_.Send(0x1fe, self->can_.Group(0x1fe), 8);
    }
    if (self->io_ticks_ % 10 == 0 && self->plot_remaining_ != 0) {
      const auto sample = self->GetAttitude();
      if (self->plot_mode_ == 0) {
        self->vofa_usb_.Set_Data(4, &sample.quaternion[0],
                                 &sample.quaternion[1], &sample.quaternion[2],
                                 &sample.quaternion[3]);
        self->vofa_usb_.TIM_1ms_Write_PeriodElapsedCallback();
      } else if (self->plot_mode_ == 1) {
        self->vofa_uart_.Set_Data(4, &sample.quaternion[0],
                                  &sample.quaternion[1], &sample.quaternion[2],
                                  &sample.quaternion[3]);
        self->vofa_uart_.TIM_1ms_Write_PeriodElapsedCallback();
      } else if (self->plot_mode_ == 2) {
        self->serialplot_.Set_Data(3, &sample.euler[0], &sample.euler[1],
                                   &sample.euler[2]);
        self->serialplot_.TIM_1ms_Write_PeriodElapsedCallback();
      } else if (self->plot_mode_ == 3) {
        self->vofa_usb_.Set_Data(2, &self->kalman_.Vector_X[0][0],
                                 &self->kalman_.Vector_X[1][0]);
        self->vofa_usb_.TIM_1ms_Write_PeriodElapsedCallback();
      } else {
        const auto* p = sample.reference_channels;
        self->vofa_usb_.Set_Data(23, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5],
                                 &p[6], &p[7], &p[8], &p[9], &p[10], &p[11],
                                 &p[12], &p[13], &p[14], &p[15], &p[16], &p[17],
                                 &p[18], &p[19], &p[20], &p[21], &p[22]);
        self->vofa_usb_.TIM_1ms_Write_PeriodElapsedCallback();
      }
      --self->plot_remaining_;
    }
    Request request{};
    if (self->requests_.Pop(request) == LibXR::ErrorCode::OK) {
      self->Execute(request);
    }
    LibXR::Thread::Sleep(1);
  }
}

// 消费原始时间戳事件并采样解算；输入应用对象，输出姿态及IMU状态。
void ReferenceApplication::ImuThread(ReferenceApplication* self) {
  uint64_t last_temperature_us = 0;
  uint64_t last_sample_us = 0;
  for (;;) {
    self->imu_ready_.Wait(5);
    if (!self->imu_requested_.load()) {
      self->heater_requested_.store(false);
      self->hardware_.heater.SetDutyCycle(0);
      self->imu_state_.store(0);
      last_sample_us = 0;
      continue;
    }
    if (self->imu_state_.load() != 2) {
      self->imu_state_.store(1);
      if (!self->imu_.Init(&self->imu_spi_)) {
        self->imu_state_.store(3);
        self->imu_requested_.store(false);
        continue;
      }
      ImuEvent old{};
      while (self->imu_events_.Pop(old) == LibXR::ErrorCode::OK) {
      }
      self->imu_state_.store(2);
    }
    ImuEvent event{};
    for (size_t index = 0;
         index < 32 && self->imu_events_.Pop(event) == LibXR::ErrorCode::OK;
         ++index) {
      // 延迟事件不能代表当前样本，避免旧时间配新寄存器导致异常dt。
      const uint64_t received_us = LibXR::Timebase::GetMicroseconds();
      if (received_us - event.timestamp_us > 2000) {
        self->imu_dropped_.fetch_add(1);
        continue;
      }
      last_sample_us = event.timestamp_us;
      self->imu_.EXTI_Flag_Callback(event.pin, event.timestamp_us);
      self->imu_.TIM_10us_Calculate_PeriodElapsedCallback();
      self->imu_.TIM_125us_Calculate_PeriodElapsedCallback();
    }
    const uint64_t now_us = LibXR::Timebase::GetMicroseconds();
    if (self->imu_spi_.faulted()) {
      self->hardware_.heater.SetDutyCycle(0);
      self->heater_requested_.store(false);
      self->imu_state_.store(3);
      self->imu_requested_.store(false);
      continue;
    }
    if (now_us - last_temperature_us >= 128000) {
      self->imu_.BMI088_Accel.Set_Heater_Enable(
          self->heater_requested_.load() && now_us - last_sample_us < 100000);
      self->imu_.TIM_128ms_Calculate_PeriodElapsedCallback();
      self->imu_.TIM_10us_Calculate_PeriodElapsedCallback();
      last_temperature_us = now_us;
    }
    auto quaternion = self->imu_.Get_Quaternion();
    const auto euler = self->imu_.Get_Euler_Angle();
    const auto original_accel = self->imu_.Get_Original_Accel();
    const auto original_gyro = self->imu_.Get_Original_Gyro();
    const auto accel_odom = self->imu_.Get_Accel_Odom();
    const auto gyro_odom = self->imu_.Get_Gyro_Odom();
    self->attitude_mutex_.Lock();
    for (size_t index = 0; index < 4; ++index) {
      self->attitude_.quaternion[index] = quaternion[index];
    }
    for (size_t index = 0; index < 3; ++index) {
      self->attitude_.euler[index] = euler[index][0];
    }
    auto* p = self->attitude_.reference_channels;
    for (size_t index = 0; index < 3; ++index) {
      p[index] = original_accel[index][0];
      p[index + 3] = original_gyro[index][0];
      p[index + 10] = euler[index][0] / BASIC_MATH_DEG_TO_RAD;
      p[index + 14] = accel_odom[index][0];
      p[index + 17] = gyro_odom[index][0];
    }
    for (size_t index = 0; index < 4; ++index) {
      p[index + 6] = quaternion[index];
    }
    p[13] = self->imu_.BMI088_Accel.Get_Now_Temperature();
    p[20] = self->imu_.Get_Accel_Chi_Square_Loss();
    p[21] = static_cast<float>(self->imu_.Get_Calculating_Time());
    p[22] = static_cast<float>(now_us) * 1e-6f;
    self->attitude_mutex_.Unlock();
  }
}

}  // namespace frame_cdc
