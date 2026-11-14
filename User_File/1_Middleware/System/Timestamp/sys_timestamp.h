/**
 * @file sys_timestamp.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 时间计算相关支持库
 * @version 0.1
 * @date 2025-08-16 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/**
 * Frame_CDC适配: 复用现有LibXR/TIM17时基，不另占TIM5或小时中断。
 * 保留参考接口单位，float秒/毫秒用于兼容，长期时间计算应使用整数微秒。
 */

#ifndef SYS_TIME_H
#define SYS_TIME_H

/* Includes ------------------------------------------------------------------*/

#pragma push_macro("PI")
#pragma push_macro("INDEX_MASK")
#undef PI
#undef INDEX_MASK
#include "libxr.hpp"
#pragma pop_macro("INDEX_MASK")
#pragma pop_macro("PI")
#include "tim.h"
#include "timebase.hpp"
#include "thread.hpp"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 系统时间戳
 *
 */
namespace frame_cdc {
// 读取当前调用上下文；无输入，中断返回true，便于隔离平台与主机替身。
bool IsInterruptContext();
}

class Class_Timestamp {
 public:
  // 读取已有LibXR时钟；无输入，输出微秒时间戳及单位换算。
  static uint64_t Get_Current_Timestamp();
  static uint64_t Get_Now_Microsecond();
  static float Get_Now_Second();
  static float Get_Now_Millisecond();
};

class Class_Time
{
public:
    // 构造函数
    Class_Time()
    {
        Microsecond = 0;
    }

    explicit Class_Time(const int64_t &__Microsecond)
    {
        Microsecond = __Microsecond;
    }

    Class_Time(const int32_t &__Second, const int16_t &__Millisecond, const int16_t &__Microsecond)
    {
        Microsecond = (int64_t)(__Second) * 1000000LL + (int64_t)(__Millisecond) * 1000LL + (int64_t)(__Microsecond);
    }

    // 复制构造函数
    Class_Time(const Class_Time &Time)
    {
        Microsecond = Time.Microsecond;
    }

    // 移动构造函数
    Class_Time(Class_Time &&Time) noexcept
    {
        Microsecond = Time.Microsecond;
    }

    // 析构函数
    ~Class_Time() = default;

    // 复制赋值函数
    Class_Time &operator=(const Class_Time &Time)
    {
        if (this != &Time)
        {
            Microsecond = Time.Microsecond;
        }
        return (*this);
    }

    // 移动赋值函数
    Class_Time &operator=(Class_Time &&Time) noexcept
    {
        Microsecond = Time.Microsecond;
        return (*this);
    }

    inline Class_Time operator+(const Class_Time &Time) const
    {
        return (Class_Time(Microsecond + Time.Get_Microsecond()));
    }

    inline Class_Time operator-(const Class_Time &Time) const
    {
        return (Class_Time(Microsecond - Time.Get_Microsecond()));
    }

    inline Class_Time operator*(const float &value) const
    {
        return (Class_Time((int64_t)((float) (Microsecond) * value)));
    }

    inline friend Class_Time operator*(const float &value, const Class_Time &Time)
    {
        return (Class_Time((int64_t)((float) (Time.Get_Microsecond()) * value)));
    }

    inline Class_Time operator/(const float &value) const
    {
        return (Class_Time((int64_t)((float) (Microsecond) / value)));
    }

    inline Class_Time &operator+=(const Class_Time &Time)
    {
        Microsecond += Time.Get_Microsecond();
        return (*this);
    }

    inline Class_Time &operator-=(const Class_Time &Time)
    {
        Microsecond -= Time.Get_Microsecond();
        return (*this);
    }

    inline Class_Time &operator*=(const float &value)
    {
        Microsecond = (int64_t)((float) (Microsecond) * value);
        return (*this);
    }

    inline Class_Time &operator/=(const float &value)
    {
        Microsecond = (int64_t)((float) (Microsecond) / value);
        return (*this);
    }

    inline bool operator>(const Class_Time &Time) const
    {
        return (Microsecond > Time.Get_Microsecond());
    }

    inline bool operator<(const Class_Time &Time) const
    {
        return (Microsecond < Time.Get_Microsecond());
    }

    inline bool operator>=(const Class_Time &Time) const
    {
        return (Microsecond >= Time.Get_Microsecond());
    }

    inline bool operator<=(const Class_Time &Time) const
    {
        return (Microsecond <= Time.Get_Microsecond());
    }

    inline bool operator==(const Class_Time &Time) const
    {
        return (Microsecond == Time.Get_Microsecond());
    }

    inline bool operator!=(const Class_Time &Time) const
    {
        return (Microsecond != Time.Get_Microsecond());
    }

    inline float Get_Second() const;

    inline float Get_Millisecond() const;

    inline int64_t Get_Microsecond() const;

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 微秒
    int64_t Microsecond = 0;

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/



namespace Namespace_SYS_Timestamp
{
    void Delay_Second(const uint32_t &Second);

    void Delay_Millisecond(const uint32_t &Millisecond);

    void Delay_Microsecond(const uint32_t &Microsecond);
}

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前时间
 *
 * @return uint64_t 当前时间
 */
inline float Class_Time::Get_Second() const
{
    return ((float) Microsecond / 1000000.0f);
}

/**
 * @brief 获取时间, 单位毫秒
 *
 * @return float 时间, 单位毫秒
 */
inline float Class_Time::Get_Millisecond() const
{
    return ((float) Microsecond / 1000.0f);
}

/**
 * @brief 获取时间, 单位微秒
 *
 * @return int64_t 时间, 单位微秒
 */
inline int64_t Class_Time::Get_Microsecond() const
{
    return (Microsecond);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
