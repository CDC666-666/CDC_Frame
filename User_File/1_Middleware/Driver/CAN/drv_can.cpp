#include "drv_can.h"

#include <algorithm>
#include <cstring>

#include "timebase.hpp"

// 绑定已初始化CAN并注册复制回调；输入接口，返回单次绑定结果。
bool Struct_CAN_Manage_Object::Bind(LibXR::CAN& can) {
  if (can_ != nullptr) {
    return false;
  }
  can_ = &can;
  can.Register(LibXR::CAN::Callback::Create(
                   [](bool, Struct_CAN_Manage_Object* self,
                      const LibXR::CAN::ClassicPack& pack) {
                     const Received frame{pack,
                                          LibXR::Timebase::GetMicroseconds()};
                     if (self->received_.Push(frame) != LibXR::ErrorCode::OK) {
                       self->dropped.fetch_add(1, std::memory_order_relaxed);
                     }
                   },
                   this),
               LibXR::CAN::Type::STANDARD);
  return true;
}

// 分发排队接收帧；输入处理预算，返回实际处理数。
size_t Struct_CAN_Manage_Object::Process(size_t budget) {
  Received frame{};
  size_t count = 0;
  while (count < budget && received_.Pop(frame) == LibXR::ErrorCode::OK) {
    Rx_Header = {};
    Rx_Header.Identifier = frame.pack.id;
    Rx_Header.DataLength = frame.pack.dlc;
    Rx_Header.IdType = FDCAN_STANDARD_ID;
    Rx_Header.RxFrameType = FDCAN_DATA_FRAME;
    std::memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
    std::memcpy(Rx_Buffer, frame.pack.data,
                std::min<size_t>(frame.pack.dlc, 8));
    Rx_Timestamp = frame.timestamp;
    on_receive.Run(false);
    ++count;
  }
  return count;
}

// 提交经典CAN标准帧；输入ID/数据/长度，返回HAL兼容提交状态。
uint8_t Struct_CAN_Manage_Object::Send(uint16_t id, const uint8_t* data,
                                       uint16_t size) {
  if (can_ == nullptr || id > 0x7ff || size > 8 || (size && data == nullptr)) {
    return HAL_ERROR;
  }
  LibXR::CAN::ClassicPack pack{};
  pack.id = id;
  pack.type = LibXR::CAN::Type::STANDARD;
  pack.dlc = static_cast<uint8_t>(size);
  if (size) {
    std::memcpy(pack.data, data, size);
  }
  const auto result = can_->AddMessage(pack);
  if (result != LibXR::ErrorCode::OK) {
    transmit_failed.fetch_add(1);
  }
  return result == LibXR::ErrorCode::OK ? HAL_OK : HAL_BUSY;
}

// 查找本总线组帧区；输入组ID，返回8字节缓冲或空指针。
uint8_t* Struct_CAN_Manage_Object::Group(uint16_t id) {
  constexpr uint16_t ids[] = {0x1fe, 0x1ff, 0x200, 0x2fe, 0x2ff, 0x3fe, 0x4fe};
  for (size_t index = 0; index < std::size(ids); ++index) {
    if (ids[index] == id) {
      return tx_groups[index].data();
    }
  }
  return nullptr;
}

// 转发设备发送；输入对象、ID及数据，返回提交状态。
uint8_t CAN_Transmit_Data(Struct_CAN_Manage_Object* driver, uint16_t id,
                          const uint8_t* data, uint16_t size) {
  if (driver == nullptr) {
    return HAL_ERROR;
  }
  return driver->Send(id, data, size);
}
