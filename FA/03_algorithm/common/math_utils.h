#ifndef FRAME_CDC_ALGORITHM_COMMON_MATH_UTILS_H
#define FRAME_CDC_ALGORITHM_COMMON_MATH_UTILS_H

#include <math.h>

/** 作用将数值限制在给定的闭区间。
 * 输入value 及有限有序的 minimum、maximum，输出限幅后的数值。 */
static inline double math_utils_clamp(double value, double minimum, double maximum)
{
    return fmin(fmax(value, minimum), maximum);
}

#endif /* FRAME_CDC_ALGORITHM_COMMON_MATH_UTILS_H */
