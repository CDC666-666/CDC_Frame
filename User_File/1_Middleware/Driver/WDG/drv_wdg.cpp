#include "drv_wdg.h"

// 复用现有看门狗接口；输入对象，输出一次喂狗，无新线程。
void IWDG_Independent_Feed(LibXR::Watchdog& watchdog) { watchdog.Feed(); }
