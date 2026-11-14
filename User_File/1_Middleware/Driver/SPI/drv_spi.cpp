#include "drv_spi.h"

#include <cstring>

#include "timebase.hpp"

// 保存SPI借用；输入接口，无返回，不启动新事务。
void Struct_SPI_Manage_Object::Bind(LibXR::SPI& spi) {
  spi_ = &spi;
  completion_ = LibXR::SPI::OperationRW::Callback::Create(
      [](bool in_isr, Struct_SPI_Manage_Object* self, LibXR::ErrorCode result) {
        self->Complete(in_isr, result);
      },
      this);
}

// DMA或阻塞事务完成收尾；输入上下文/结果，输出设备回调并释放占用。
void Struct_SPI_Manage_Object::Complete(bool in_isr, LibXR::ErrorCode result) {
  if (Activate_GPIOx != nullptr) {
    HAL_GPIO_WritePin(
        Activate_GPIOx, Activate_GPIO_Pin,
        Activate_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  if (result != LibXR::ErrorCode::OK) {
    faulted_.store(true);
  }
  Rx_Timestamp = LibXR::Timebase::GetMicroseconds();
  if (result == LibXR::ErrorCode::OK && Rx_Buffer_Length != 0) {
    on_receive.Run(in_isr);
  }
  Activate_GPIOx = nullptr;
  busy_.store(false);
}

// 输入片选/阶段长度/上下文；线程等待DMA，ISR只提交，CS在完成后释放。
uint8_t Struct_SPI_Manage_Object::Transfer(GPIO_TypeDef* port, uint16_t pin,
                                           GPIO_PinState active,
                                           uint16_t tx_size, uint16_t rx_size,
                                           bool in_isr) {
  const size_t total = static_cast<size_t>(tx_size) + rx_size;
  if (spi_ == nullptr || faulted_.load() || total == 0 ||
      total > SPI_BUFFER_SIZE) {
    return HAL_ERROR;
  }
  if (busy_.exchange(true)) {
    return HAL_BUSY;
  }
  Activate_GPIOx = port;
  Activate_GPIO_Pin = pin;
  Activate_Level = active;
  Tx_Buffer_Length = tx_size;
  Rx_Buffer_Length = rx_size;
  std::memset(Tx_Buffer + tx_size, 0, rx_size);
  if (port != nullptr) {
    HAL_GPIO_WritePin(port, pin, active);
  }
  LibXR::SPI::OperationRW operation =
      in_isr ? LibXR::SPI::OperationRW(completion_)
             : LibXR::SPI::OperationRW(completed_, 20);
  const auto result =
      rx_size == 0 ? spi_->Write({Tx_Buffer, total}, operation, in_isr)
                   : spi_->ReadAndWrite({Rx_Buffer, total}, {Tx_Buffer, total},
                                        operation, in_isr);
  if (!in_isr || result != LibXR::ErrorCode::OK) {
    Complete(in_isr, result);
  }
  return result == LibXR::ErrorCode::OK ? HAL_OK : HAL_ERROR;
}

// 发送SPI事务；输入对象、片选及长度，线程返回完成状态，ISR返回接受状态。
uint8_t SPI_Transmit_Data(Struct_SPI_Manage_Object* driver, GPIO_TypeDef* port,
                          uint16_t pin, GPIO_PinState active, uint16_t size) {
  if (driver == nullptr) {
    return HAL_ERROR;
  }
  return driver->Transfer(port, pin, active, size, 0,
                          frame_cdc::IsInterruptContext());
}

// 执行命令加接收事务；输入对象、片选及长度，线程返回完成、ISR返回接受状态。
uint8_t SPI_Transmit_Receive_Data(Struct_SPI_Manage_Object* driver,
                                  GPIO_TypeDef* port, uint16_t pin,
                                  GPIO_PinState active, uint16_t tx_size,
                                  uint16_t rx_size) {
  if (driver == nullptr) {
    return HAL_ERROR;
  }
  return driver->Transfer(port, pin, active, tx_size, rx_size,
                          frame_cdc::IsInterruptContext());
}
