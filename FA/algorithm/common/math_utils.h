#ifndef FRAME_CDC_COMPONENTS_ALGORITHMS_COMMON_MATH_UTILS_H
#define FRAME_CDC_COMPONENTS_ALGORITHMS_COMMON_MATH_UTILS_H

#include <math.h>

/** @brief 将数值限制在 [minimum, maximum]。上下界必须有序且有限。 */
static inline double math_utils_clamp(double value, double minimum, double maximum)
{
    return fmin(fmax(value, minimum), maximum);
}

#endif /* FRAME_CDC_COMPONENTS_ALGORITHMS_COMMON_MATH_UTILS_H */
