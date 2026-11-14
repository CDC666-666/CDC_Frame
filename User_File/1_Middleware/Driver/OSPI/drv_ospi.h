#ifndef FRAME_CDC_DRIVER_OSPI_DRV_OSPI_H_
#define FRAME_CDC_DRIVER_OSPI_DRV_OSPI_H_

#include <cstdint>

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "libxr_cb.hpp"
#include "octospi.h"

inline constexpr size_t OSPI_BUFFER_SIZE = 512;

// OSPI2/MDMA由后台线程独占；数据DMA期间让出CPU，禁止ISR调用。
struct Struct_OSPI_Manage_Object {
  OSPI_HandleTypeDef* handle = nullptr;
  uint8_t Tx_Buffer[OSPI_BUFFER_SIZE]{};
  alignas(4) uint8_t Rx_Buffer[OSPI_BUFFER_SIZE]{};
  uint64_t Auto_Polling_Timestamp = 0;
  uint64_t Rx_Timestamp = 0;
  LibXR::Callback<> on_poll;
  LibXR::Callback<> on_receive;
  LibXR::Callback<> on_transmit;
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t transfer_timeout_ms = 100;
  bool memory_mapped = false;
  uint32_t poll_timeout_ms = 5000;

  // 线程提交MDMA并休眠等完成；输入命令/方向，返回最终状态。
  HAL_StatusTypeDef Execute(OSPI_RegularCmdTypeDef& command, bool receive,
                            bool transmit);
  // 自动轮询就绪位；输入匹配配置，输出有界等待状态。
  HAL_StatusTypeDef Poll(OSPI_AutoPollingTypeDef& config);
  // 切换内存映射；输入启用标志，输出HAL状态。
  HAL_StatusTypeDef Map(bool enable);

 private:
  // 等待HAL中断发布完成；输入超时毫秒，返回最终状态，休眠让出CPU。
  HAL_StatusTypeDef WaitForReady(uint32_t timeout_ms);
  bool faulted_ = false;
};

// 提交OSPI命令；输入显式管理对象及命令，输出状态。
void OSPI_Command(Struct_OSPI_Manage_Object* driver,
                  OSPI_RegularCmdTypeDef* cmd);
// 命令后发送；输入管理对象及命令，输出记录于status。
void OSPI_Command_Transmit_Data(Struct_OSPI_Manage_Object* driver,
                                OSPI_RegularCmdTypeDef* cmd);
// 命令后接收；输入管理对象及命令，输出记录于status。
void OSPI_Command_Receive_Data(Struct_OSPI_Manage_Object* driver,
                               OSPI_RegularCmdTypeDef* cmd);
// 有界自动轮询；输入对象及匹配条件，输出记录于status。
void OSPI_Auto_Polling(Struct_OSPI_Manage_Object* driver,
                       OSPI_AutoPollingTypeDef* cfg);

#endif
