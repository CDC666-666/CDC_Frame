#include "bsp_w25q64jv.h"

#include <algorithm>
#include <cstring>

// 构造单笔SPI Flash命令；输入操作码、地址及长度，返回配置。
OSPI_RegularCmdTypeDef Class_W25Q64JV::Command(uint8_t opcode, uint32_t address,
                                               size_t length, bool addressed) {
  OSPI_RegularCmdTypeDef command{};
  command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  command.FlashId = HAL_OSPI_FLASH_ID_1;
  command.Instruction = opcode;
  command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  command.Address = address;
  command.AddressMode =
      addressed ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_NONE;
  command.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  command.DataMode = length == 0 ? HAL_OSPI_DATA_NONE : HAL_OSPI_DATA_1_LINE;
  command.NbData = length;
  return command;
}
// 绑定Flash并检查JEDEC；输入后端及模式，返回初始化结果。
bool Class_W25Q64JV::Init(Struct_OSPI_Manage_Object* driver,
                          const Enum_W25Q64JV_Mode& mode) {
  driver_ = driver;
  if (driver_ == nullptr) {
    return false;
  }
  auto command = Command(0x9f, 0, 3, false);
  if (driver_->Execute(command, true, false) != HAL_OK) {
    return false;
  }
  jedec_id_ = (driver_->Rx_Buffer[0] << 16U) | (driver_->Rx_Buffer[1] << 8U) |
              driver_->Rx_Buffer[2];
  if (jedec_id_ != 0xef4017U) {
    return false;
  }
  command = Command(0x35, 0, 1, false);
  if (driver_->Execute(command, true, false) != HAL_OK) {
    return false;
  }
  quad_ = (driver_->Rx_Buffer[0] & 2U) != 0;
  return mode != W25Q64JV_Mode_MemoryMapped || SetMemoryMapped(true);
}
// 有界等待状态位；输入掩码、期望值及超时，返回匹配结果。
bool Class_W25Q64JV::Wait(uint8_t mask, uint8_t value, uint32_t timeout_ms) {
  auto command = Command(0x05, 0, 1, false);
  if (driver_ == nullptr || driver_->Execute(command, false, false) != HAL_OK) {
    return false;
  }
  OSPI_AutoPollingTypeDef config{};
  config.Match = value;
  config.Mask = mask;
  config.MatchMode = HAL_OSPI_MATCH_MODE_AND;
  config.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
  config.Interval = 16;
  driver_->poll_timeout_ms = timeout_ms;
  return driver_->Poll(config) == HAL_OK;
}
// 等待空闲并设置WEL；无输入，返回硬件写使能检查结果。
bool Class_W25Q64JV::Set_Write_Enable() {
  if (driver_ == nullptr || !Wait(1, 0, 5000)) {
    return false;
  }
  auto command = Command(0x06, 0, 0, false);
  return driver_->Execute(command, false, false) == HAL_OK && Wait(2, 2, 100);
}
// 读取Flash数据；输入地址及长度，返回状态，数据保存在Rx_Buffer。
bool Class_W25Q64JV::Get_Buffer(const uint32_t& address,
                                const uint8_t& length) {
  if (driver_ == nullptr || length == 0 || address >= capacity_ ||
      length > capacity_ - address) {
    return false;
  }
  auto command = Command(quad_ ? 0x6b : 0x03, address, length, true);
  if (quad_) {
    command.DataMode = HAL_OSPI_DATA_4_LINES;
    command.DummyCycles = 8;
  }
  return driver_->Execute(command, true, false) == HAL_OK;
}
// 分割跨页编程；输入数据、地址及长度，返回整笔完成状态。
bool Class_W25Q64JV::Set_Buffer(const uint8_t* data, const uint32_t& address,
                                const uint8_t& length) {
  if (driver_ == nullptr || data == nullptr || length == 0 ||
      address >= capacity_ || length > capacity_ - address) {
    return false;
  }
  size_t done = 0;
  while (done < length) {
    const size_t count =
        std::min<size_t>(length - done, 256U - ((address + done) & 255U));
    if (!Set_Write_Enable()) {
      return false;
    }
    std::memcpy(driver_->Tx_Buffer, data + done, count);
    auto command = Command(quad_ ? 0x32 : 0x02, address + done, count, true);
    if (quad_) {
      command.DataMode = HAL_OSPI_DATA_4_LINES;
    }
    if (driver_->Execute(command, false, true) != HAL_OK || !Wait(1, 0, 5000)) {
      return false;
    }
    done += count;
  }
  return true;
}
// 擦除对齐区域；输入操作码、地址及粒度，返回完成状态。
bool Class_W25Q64JV::Erase(uint8_t opcode, uint32_t address,
                           uint32_t granularity) {
  if (address >= capacity_ || address % granularity != 0 ||
      !Set_Write_Enable()) {
    return false;
  }
  auto command = Command(opcode, address, 0, true);
  return driver_->Execute(command, false, false) == HAL_OK && Wait(1, 0, 10000);
}
// 擦除4KiB扇区；输入对齐地址，返回完成状态。
bool Class_W25Q64JV::Set_Sector_Erased(const uint32_t& address) {
  return Erase(0x20, address, 4096);
}
// 擦除32KiB块；输入对齐地址，返回完成状态。
bool Class_W25Q64JV::Set_Bolck_Erased_32K(const uint32_t& address) {
  return Erase(0x52, address, 32768);
}
// 擦除64KiB块；输入对齐地址，返回完成状态。
bool Class_W25Q64JV::Set_Bolck_Erased_64K(const uint32_t& address) {
  return Erase(0xd8, address, 65536);
}
// 擦除整片；无输入，返回有界等待后的完成状态。
bool Class_W25Q64JV::Set_Chip_Erased() {
  if (!Set_Write_Enable()) {
    return false;
  }
  auto command = Command(0xc7, 0, 0, false);
  return driver_->Execute(command, false, false) == HAL_OK &&
         Wait(1, 0, 120000);
}
// 保留其他状态位并设置易失QE；无输入，返回读回验证结果。
bool Class_W25Q64JV::EnableQuad() {
  if (driver_ == nullptr) {
    return false;
  }
  auto command = Command(0x35, 0, 1, false);
  if (driver_->Execute(command, true, false) != HAL_OK) {
    return false;
  }
  const uint8_t sr2 = driver_->Rx_Buffer[0] | 2U;
  command = Command(0x50, 0, 0, false);
  if (driver_->Execute(command, false, false) != HAL_OK) {
    return false;
  }
  driver_->Tx_Buffer[0] = sr2;
  command = Command(0x31, 0, 1, false);
  if (driver_->Execute(command, false, true) != HAL_OK) {
    return false;
  }
  command = Command(0x35, 0, 1, false);
  if (driver_->Execute(command, true, false) != HAL_OK) {
    return false;
  }
  return quad_ = (driver_->Rx_Buffer[0] & 2U) != 0;
}
// 切换映射模式；输入使能标志，返回执行状态。
bool Class_W25Q64JV::SetMemoryMapped(bool enable) {
  if (driver_ == nullptr) {
    return false;
  }
  if (!enable) {
    return driver_->Map(false) == HAL_OK;
  }
  auto command = Command(quad_ ? 0x6b : 0x03, 0, 1, true);
  command.DataMode = quad_ ? HAL_OSPI_DATA_4_LINES : HAL_OSPI_DATA_1_LINE;
  command.DummyCycles = quad_ ? 8 : 0;
  return driver_->Execute(command, false, false) == HAL_OK &&
         driver_->Map(true) == HAL_OK;
}
