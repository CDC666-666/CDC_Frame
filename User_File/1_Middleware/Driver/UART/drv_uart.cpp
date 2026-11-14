#include "drv_uart.h"

#include <algorithm>

#include "timebase.hpp"

// 绑定现有串口；输入接口，返回单次绑定结果。
bool Struct_UART_Manage_Object::Bind(LibXR::UART& uart) {
  if (uart_ != nullptr) {
    return false;
  }
  uart_ = &uart;
  return true;
}

// 在线程读取并同步分发字节块；无输入，返回实际读取长度。
size_t Struct_UART_Manage_Object::Process() {
  if (uart_ == nullptr || uart_->read_port_ == nullptr) {
    return 0;
  }
  const auto size = std::min(uart_->read_port_->Size(), UART_BUFFER_SIZE);
  if (size == 0 || (*uart_->read_port_)({Rx_Buffer_0, size}, read_) !=
                       LibXR::ErrorCode::OK) {
    return 0;
  }
  Rx_Time_Stamp = LibXR::Timebase::GetMicroseconds();
  on_receive.Run(false, Rx_Buffer_0, static_cast<uint16_t>(size));
  return size;
}

// 复制字节到发送队列；输入数据及长度，返回提交状态。
uint8_t Struct_UART_Manage_Object::Send(const uint8_t* data, uint16_t size) {
  if (uart_ == nullptr || uart_->write_port_ == nullptr ||
      (size != 0 && data == nullptr)) {
    return HAL_ERROR;
  }
  if (size > uart_->write_port_->Capacity()) {
    return HAL_ERROR;
  }
  const auto status = (*uart_->write_port_)({data, size}, write_);
  if (status != LibXR::ErrorCode::OK) {
    transmit_failed.fetch_add(1);
  }
  return status == LibXR::ErrorCode::OK ? HAL_OK : HAL_BUSY;
}

// 转发串口发送；输入对象及数据，返回提交状态。
uint8_t UART_Transmit_Data(Struct_UART_Manage_Object* driver,
                           const uint8_t* data, uint16_t size) {
  if (driver == nullptr) {
    return HAL_ERROR;
  }
  return driver->Send(data, size);
}
