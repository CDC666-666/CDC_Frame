#include "drv_adc.h"

#include <limits>

// 读取ADC电压；无输入，返回V，未绑定时为NaN。
float Struct_ADC_Manage_Object::ReadVoltage() const {
  return channel == nullptr ? std::numeric_limits<float>::quiet_NaN()
                            : channel->Read();
}
