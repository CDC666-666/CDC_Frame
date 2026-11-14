// Adapted from USTC-RoboWalker W25Q64JV (2025); reference API names retained.
#ifndef FRAME_CDC_BSP_W25Q64JV_H_
#define FRAME_CDC_BSP_W25Q64JV_H_
#include <cstddef>
#include <cstdint>

#include "1_Middleware/Driver/OSPI/drv_ospi.h"

enum Enum_W25Q64JV_Mode { W25Q64JV_Mode_Normal, W25Q64JV_Mode_MemoryMapped };
class Class_W25Q64JV {
 public:
  // 读取JEDEC并绑定；输入后端与模式，返回初始化状态。
  bool Init(Struct_OSPI_Manage_Object* driver,
            const Enum_W25Q64JV_Mode& mode = W25Q64JV_Mode_Normal);
  // 有界读取；输入地址和长度，结果位于driver的Rx_Buffer。
  bool Get_Buffer(const uint32_t& address, const uint8_t& length);
  // 跨页写入；输入源、地址和长度，返回整笔完成状态。
  bool Set_Buffer(const uint8_t* data, const uint32_t& address,
                  const uint8_t& length);
  // 设置并检查WEL；无输入，返回芯片写使能状态。
  bool Set_Write_Enable();
  // 擦除4KB扇区；输入对齐地址，返回完成状态。
  bool Set_Sector_Erased(const uint32_t& address);
  // 擦除32KB块；输入对齐地址，返回完成状态。
  bool Set_Bolck_Erased_32K(const uint32_t& address);
  // 擦除64KB块；输入对齐地址，返回完成状态。
  bool Set_Bolck_Erased_64K(const uint32_t& address);
  // 擦除整片；无输入，返回有界等待后的状态。
  bool Set_Chip_Erased();
  // 只修改易失QE并核对；无输入，返回Quad可用状态。
  bool EnableQuad();
  // 切换映射模式；输入启用标志，返回状态。
  bool SetMemoryMapped(bool enable);
  // 读取JEDEC缓存；无输入，返回24位识别码。
  uint32_t jedec_id() const { return jedec_id_; }
  // 兼容旧异步通知名；同步路径已经完成，无输入输出，不改变状态。
  void OSPI_StatusMatchCallback() {}
  void OSPI_RxCallback() {}
  void OSPI_TxCallback() {}
  void TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback() {}

 private:
  // 构造单笔命令；输入操作码、地址和长度，返回命令描述。
  OSPI_RegularCmdTypeDef Command(uint8_t opcode, uint32_t address,
                                 size_t length, bool address_enabled);
  // 等待状态位；输入掩码、期望值和超时，返回匹配状态。
  bool Wait(uint8_t mask, uint8_t value, uint32_t timeout_ms);
  // 擦除对齐区域；输入操作码、地址和粒度，返回状态。
  bool Erase(uint8_t opcode, uint32_t address, uint32_t granularity);
  Struct_OSPI_Manage_Object* driver_ = nullptr;
  uint32_t jedec_id_ = 0;
  bool quad_ = false;
  static constexpr uint32_t capacity_ = 8U * 1024U * 1024U;
};
#endif
