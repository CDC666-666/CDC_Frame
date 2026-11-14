#ifndef FRAME_CDC_DRIVER_ADC_DRV_ADC_H_
#define FRAME_CDC_DRIVER_ADC_DRV_ADC_H_

// 先经兼容边界加载LibXR，隔离原算法CMSIS的PI/INDEX_MASK宏。
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "adc.hpp"

struct Struct_ADC_Manage_Object {
  LibXR::ADC* channel = nullptr;
  // 读取引脚电压；无输入，输出V，分压换算由Power完成。
  float ReadVoltage() const;
};

#endif
