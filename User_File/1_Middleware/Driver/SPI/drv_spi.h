#ifndef FRAME_CDC_DRIVER_SPI_DRV_SPI_H_
#define FRAME_CDC_DRIVER_SPI_DRV_SPI_H_

#include <cstdint>

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "adc.hpp"
#include "pwm.hpp"
#include "spi.hpp"
#include "stm32h7xx_hal.h"

inline constexpr size_t SPI_BUFFER_SIZE = 512;

// 参考SPI事务视图；仅由一个设备工作线程调用，LibXR管理实际DMA。
struct Struct_SPI_Manage_Object {
  GPIO_TypeDef* Activate_GPIOx = nullptr;
  uint16_t Activate_GPIO_Pin = 0;
  GPIO_PinState Activate_Level = GPIO_PIN_RESET;
  uint8_t Tx_Buffer[SPI_BUFFER_SIZE]{};
  uint8_t Rx_Buffer[SPI_BUFFER_SIZE]{};
  uint16_t Tx_Buffer_Length = 0;
  uint16_t Rx_Buffer_Length = 0;
  uint64_t Rx_Timestamp = 0;
  LibXR::Callback<> on_receive;
  LibXR::ADC* power_adc = nullptr;
  LibXR::PWM* heater_pwm = nullptr;

  // 绑定已初始化SPI；输入接口引用，无输出，不配置HAL。
  void Bind(LibXR::SPI& spi);
  // 查询后端故障锁存；无输入，true表示需重启后才能继续事务。
  bool faulted() const { return faulted_; }
  // 执行完整事务；输入片选及命令/接收长度，返回最终状态，线程调用。
  uint8_t Transfer(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState active,
                   uint16_t tx_size, uint16_t rx_size);

 private:
  LibXR::SPI* spi_ = nullptr;
  LibXR::Semaphore completed_;
  bool busy_ = false;
  bool faulted_ = false;
};

// 发送事务；输入管理对象、片选与长度，输出完成状态。
uint8_t SPI_Transmit_Data(Struct_SPI_Manage_Object* driver, GPIO_TypeDef* port,
                          uint16_t pin, GPIO_PinState active, uint16_t size);
// 命令后接收事务；输入管理对象及两阶段长度，输出完成状态。
uint8_t SPI_Transmit_Receive_Data(Struct_SPI_Manage_Object* driver,
                                  GPIO_TypeDef* port, uint16_t pin,
                                  GPIO_PinState active, uint16_t tx_size,
                                  uint16_t rx_size);

#endif
