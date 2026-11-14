#ifndef FRAME_CDC_DRIVER_WDG_DRV_WDG_H_
#define FRAME_CDC_DRIVER_WDG_DRV_WDG_H_

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "watchdog.hpp"

// 显式刷新已有看门狗；输入已构造接口，输出底层状态。
void IWDG_Independent_Feed(LibXR::Watchdog& watchdog);

#endif
