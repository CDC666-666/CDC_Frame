#include "drv_ospi.h"

#include "timebase.hpp"

// 在线程执行单向OSPI阶段；输入命令/方向，返回HAL最终状态。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::Execute(
    OSPI_RegularCmdTypeDef& command, bool receive, bool transmit) {
  if (handle == nullptr || memory_mapped || frame_cdc::IsInterruptContext() ||
      command.NbData > OSPI_BUFFER_SIZE || (receive && transmit)) {
    return status = HAL_ERROR;
  }
  status = HAL_OSPI_Command(handle, &command, 100);
  if (status != HAL_OK) {
    HAL_OSPI_Abort(handle);
    return status;
  }
  if (receive) {
    status = HAL_OSPI_Receive(handle, Rx_Buffer, 100);
    Rx_Timestamp = LibXR::Timebase::GetMicroseconds();
    if (status == HAL_OK) {
      on_receive.Run(false);
    }
  } else if (transmit) {
    status = HAL_OSPI_Transmit(handle, Tx_Buffer, 100);
    if (status == HAL_OK) {
      on_transmit.Run(false);
    }
  }
  if (status != HAL_OK) {
    HAL_OSPI_Abort(handle);
  }
  return status;
}

// 有界轮询状态；输入掩码等配置，返回HAL匹配状态。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::Poll(
    OSPI_AutoPollingTypeDef& config) {
  if (handle == nullptr || memory_mapped || frame_cdc::IsInterruptContext()) {
    return status = HAL_ERROR;
  }
  status = HAL_OSPI_AutoPolling(handle, &config, poll_timeout_ms);
  if (status != HAL_OK) {
    HAL_OSPI_Abort(handle);
  }
  Auto_Polling_Timestamp = LibXR::Timebase::GetMicroseconds();
  if (status == HAL_OK) {
    on_poll.Run(false);
  }
  return status;
}

// 切换内存映射；输入使能标志，返回HAL状态并更新模式。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::Map(bool enable) {
  if (handle == nullptr || frame_cdc::IsInterruptContext()) {
    return status = HAL_ERROR;
  }
  OSPI_MemoryMappedTypeDef config{};
  config.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_ENABLE;
  config.TimeOutPeriod = 0x20;
  status =
      enable ? HAL_OSPI_MemoryMapped(handle, &config) : HAL_OSPI_Abort(handle);
  if (status == HAL_OK) {
    memory_mapped = enable;
  }
  return status;
}

// 提交无数据命令；输入对象及命令，结果保存到status。
void OSPI_Command(Struct_OSPI_Manage_Object* driver,
                  OSPI_RegularCmdTypeDef* cmd) {
  if (driver != nullptr && cmd != nullptr) {
    driver->Execute(*cmd, false, false);
  }
}
// 提交发送阶段；输入对象及命令，结果保存到status。
void OSPI_Command_Transmit_Data(Struct_OSPI_Manage_Object* driver,
                                OSPI_RegularCmdTypeDef* cmd) {
  if (driver != nullptr && cmd != nullptr) {
    driver->Execute(*cmd, false, true);
  }
}
// 提交接收阶段；输入对象及命令，结果保存到status。
void OSPI_Command_Receive_Data(Struct_OSPI_Manage_Object* driver,
                               OSPI_RegularCmdTypeDef* cmd) {
  if (driver != nullptr && cmd != nullptr) {
    driver->Execute(*cmd, true, false);
  }
}
// 提交轮询阶段；输入对象及配置，结果保存到status。
void OSPI_Auto_Polling(Struct_OSPI_Manage_Object* driver,
                       OSPI_AutoPollingTypeDef* cfg) {
  if (driver != nullptr && cfg != nullptr) {
    driver->Poll(*cfg);
  }
}
