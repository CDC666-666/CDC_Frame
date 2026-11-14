#include "sys_timestamp.h"

namespace frame_cdc {
// 查询Cortex-M上下文；无输入，输出是否在异常/中断中。
bool IsInterruptContext() { return __get_IPSR() != 0; }
}  // namespace frame_cdc

// 读取TIM17支持的LibXR时间；无输入，返回微秒时间戳。
uint64_t Class_Timestamp::Get_Current_Timestamp() {
  return LibXR::Timebase::GetMicroseconds();
}
// 读取当前时间；无输入，返回微秒时间戳。
uint64_t Class_Timestamp::Get_Now_Microsecond() {
  return Get_Current_Timestamp();
}
// 换算参考时间接口；无输入，返回float秒。
float Class_Timestamp::Get_Now_Second() {
  return Get_Current_Timestamp() / 1e6f;
}
// 换算参考时间接口；无输入，返回float毫秒。
float Class_Timestamp::Get_Now_Millisecond() {
  return Get_Current_Timestamp() / 1e3f;
}
namespace Namespace_SYS_Timestamp {
// 在线程中延时；输入秒数，无输出，不在中断阻塞。
void Delay_Second(const uint32_t& seconds) {
  LibXR::Thread::Sleep(seconds * 1000U);
}
// 在线程中延时；输入毫秒数，无输出。
void Delay_Millisecond(const uint32_t& ms) { LibXR::Thread::Sleep(ms); }
// 有界微秒等待；输入微秒数，无输出。
void Delay_Microsecond(const uint32_t& us) {
  LibXR::Timebase::DelayMicroseconds(us);
}
}  // namespace Namespace_SYS_Timestamp
