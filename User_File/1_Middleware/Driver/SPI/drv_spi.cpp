#include "drv_spi.h"

#include <cstring>

#include "timebase.hpp"

// 保存SPI借用；输入接口，无返回，不启动新事务。
void Struct_SPI_Manage_Object::Bind(LibXR::SPI& spi) { spi_ = &spi; }

// 在线程执行完整SPI事务；输入片选和阶段长度，返回完成状态。
uint8_t Struct_SPI_Manage_Object::Transfer(GPIO_TypeDef* port, uint16_t pin,
                                           GPIO_PinState active,
                                           uint16_t tx_size, uint16_t rx_size) {
  const size_t total = static_cast<size_t>(tx_size) + rx_size;
  if (spi_ == nullptr || faulted_ || total == 0 || total > SPI_BUFFER_SIZE ||
      frame_cdc::IsInterruptContext()) {
    return HAL_ERROR;
  }
  if (busy_) {
    return HAL_BUSY;
  }
  busy_ = true;
  Activate_GPIOx = port;
  Activate_GPIO_Pin = pin;
  Activate_Level = active;
  Tx_Buffer_Length = tx_size;
  Rx_Buffer_Length = rx_size;
  std::memset(Tx_Buffer + tx_size, 0, rx_size);
  if (port != nullptr) {
    HAL_GPIO_WritePin(port, pin, active);
  }
  LibXR::SPI::OperationRW operation(completed_, 20);
  const auto result = rx_size == 0
                          ? spi_->Write({Tx_Buffer, total}, operation)
                          : spi_->ReadAndWrite({Rx_Buffer, total},
                                               {Tx_Buffer, total}, operation);
  // LibXR BLOCK超时不承诺取消底层DMA，锁存后禁止再次提交/复用后端缓冲。
  faulted_ = result != LibXR::ErrorCode::OK;
  if (port != nullptr) {
    HAL_GPIO_WritePin(port, pin,
                      active == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  Rx_Timestamp = LibXR::Timebase::GetMicroseconds();
  // 参考设备回调在清除片选元数据前解析整帧；回调在本工作线程执行。
  if (result == LibXR::ErrorCode::OK && rx_size != 0) {
    on_receive.Run(false);
  }
  Activate_GPIOx = nullptr;
  busy_ = false;
  return result == LibXR::ErrorCode::OK ? HAL_OK : HAL_ERROR;
}

// 发送SPI事务；输入对象、片选及长度，返回最终状态。
uint8_t SPI_Transmit_Data(Struct_SPI_Manage_Object* driver, GPIO_TypeDef* port,
                          uint16_t pin, GPIO_PinState active, uint16_t size) {
  if (driver == nullptr) {
    return HAL_ERROR;
  }
  return driver->Transfer(port, pin, active, size, 0);
}

// 执行命令加接收事务；输入对象、片选及长度，返回最终状态。
uint8_t SPI_Transmit_Receive_Data(Struct_SPI_Manage_Object* driver,
                                  GPIO_TypeDef* port, uint16_t pin,
                                  GPIO_PinState active, uint16_t tx_size,
                                  uint16_t rx_size) {
  if (driver == nullptr) {
    return HAL_ERROR;
  }
  return driver->Transfer(port, pin, active, tx_size, rx_size);
}
