#include "drv_usb.h"

// 复用CDCUart提交数据；输入对象及字节，返回提交状态。
uint8_t USB_Transmit_Data(Struct_USB_Manage_Object* driver, const uint8_t* data,
                          uint16_t size) {
  return UART_Transmit_Data(driver, data, size);
}
