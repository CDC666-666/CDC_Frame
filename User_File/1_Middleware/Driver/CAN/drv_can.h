#ifndef FRAME_CDC_DRIVER_CAN_DRV_CAN_H_
#define FRAME_CDC_DRIVER_CAN_DRV_CAN_H_

#include <array>
#include <atomic>
#include <cstdint>

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "can.hpp"
#include "fdcan.h"
#include "spsc_queue.hpp"

// 参考设备的兼容边界；每条总线由装配对象持有，不创建全局管理对象。
struct Struct_CAN_Manage_Object {
  FDCAN_RxHeaderTypeDef Rx_Header{};
  uint8_t Rx_Buffer[64]{};
  uint64_t Rx_Timestamp = 0;
  std::array<std::array<uint8_t, 8>, 7> tx_groups{};

  // 绑定已初始化总线；输入LibXR对象，注册接收交接，返回绑定状态。
  bool Bind(LibXR::CAN& can);
  // 在线程中分发有界批次；输入最大帧数，输出本批处理数。
  size_t Process(size_t budget = 32);
  // 提交标准CAN帧；输入ID和字节，返回HAL兼容状态，不代表设备确认。
  uint8_t Send(uint16_t id, const uint8_t* data, uint16_t size);
  // 获取设备组帧区；输入发送ID，输出8字节区或空指针。
  uint8_t* Group(uint16_t id);
  LibXR::Callback<> on_receive;
  std::atomic<uint32_t> dropped{0};
  std::atomic<uint32_t> transmit_failed{0};

 private:
  struct Received {
    LibXR::CAN::ClassicPack pack;
    uint64_t timestamp;
  };
  LibXR::CAN* can_ = nullptr;
  LibXR::SPSCQueue<Received> received_{32};
};

// 发送参考设备帧；输入显式管理对象及载荷，输出提交状态。
uint8_t CAN_Transmit_Data(Struct_CAN_Manage_Object* driver, uint16_t id,
                          const uint8_t* data, uint16_t size);

#endif
