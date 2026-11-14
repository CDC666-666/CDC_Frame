#include "drv_ospi.h"

#include "thread.hpp"
#include "timebase.hpp"

// 在线程执行单向OSPI阶段；输入命令/方向，返回HAL最终状态。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::Execute(
    OSPI_RegularCmdTypeDef& command, bool receive, bool transmit) {
  if (handle == nullptr || memory_mapped || faulted_ ||
      frame_cdc::IsInterruptContext() || command.NbData > OSPI_BUFFER_SIZE ||
      (receive && transmit)) {
    return status = HAL_ERROR;
  }
  status = HAL_OSPI_Command(handle, &command, 100);
  if (status != HAL_OK) {
    if (HAL_OSPI_Abort(handle) != HAL_OK) {
      faulted_ = true;
    }
    return status;
  }
  if (receive) {
    status = HAL_OSPI_Receive_DMA(handle, Rx_Buffer);
    if (status == HAL_OK) {
      status = WaitForReady(transfer_timeout_ms);
    }
    Rx_Timestamp = LibXR::Timebase::GetMicroseconds();
    if (status == HAL_OK) {
      on_receive.Run(false);
    }
  } else if (transmit) {
    status = HAL_OSPI_Transmit_DMA(handle, Tx_Buffer);
    if (status == HAL_OK) {
      status = WaitForReady(transfer_timeout_ms);
    }
    if (status == HAL_OK) {
      on_transmit.Run(false);
    }
  }
  if (status != HAL_OK) {
    if (HAL_OSPI_Abort(handle) != HAL_OK) {
      faulted_ = true;
    }
  }
  return status;
}

// 有界轮询状态；输入掩码等配置，返回HAL匹配状态。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::Poll(
    OSPI_AutoPollingTypeDef& config) {
  if (handle == nullptr || memory_mapped || faulted_ ||
      frame_cdc::IsInterruptContext()) {
    return status = HAL_ERROR;
  }
  status = HAL_OSPI_AutoPolling_IT(handle, &config);
  if (status == HAL_OK) {
    status = WaitForReady(poll_timeout_ms);
  }
  if (status != HAL_OK) {
    if (HAL_OSPI_Abort(handle) != HAL_OK) {
      faulted_ = true;
    }
  }
  Auto_Polling_Timestamp = LibXR::Timebase::GetMicroseconds();
  if (status == HAL_OK) {
    on_poll.Run(false);
  }
  return status;
}

// 切换内存映射；输入使能标志，返回HAL状态并更新模式。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::Map(bool enable) {
  if (handle == nullptr || (enable && faulted_) ||
      frame_cdc::IsInterruptContext()) {
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

// HAL IRQ更新状态，线程休眠等待；输入毫秒超时，返回结果，不在ISR等待。
HAL_StatusTypeDef Struct_OSPI_Manage_Object::WaitForReady(uint32_t timeout_ms) {
  const uint64_t start_us = LibXR::Timebase::GetMicroseconds();
  for (;;) {
    const auto state = HAL_OSPI_GetState(handle);
    if (HAL_OSPI_GetError(handle) != HAL_OSPI_ERROR_NONE ||
        state == HAL_OSPI_STATE_ERROR) {
      return HAL_ERROR;
    }
    if (state == HAL_OSPI_STATE_READY) {
      return HAL_OK;
    }
    const uint64_t now_us = LibXR::Timebase::GetMicroseconds();
    if (now_us - start_us >= static_cast<uint64_t>(timeout_ms) * 1000) {
      return HAL_TIMEOUT;
    }
    LibXR::Thread::Sleep(1);
  }
}
