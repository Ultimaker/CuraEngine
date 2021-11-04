//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include <cmath>


//c++11 no longer defines M_PI, so add our own constant.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cura
{

static constexpr float sqrt2 = 1.41421356237;

template<typename T> inline T square(const T& a) { return a * a; }

inline unsigned int round_divide_signed(int dividend, int divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    const int abs_div = std::abs(divisor);
    return (dividend * divisor > 0 ? 1 : -1) * ((std::abs(dividend) + abs_div / 2) / abs_div);
}
inline unsigned int ceil_divide_signed(int dividend, int divisor) //!< Return dividend divided by divisor rounded up towards positive infinity.
{
    return (dividend / divisor) + (dividend * divisor > 0 ? 1 : 0);
}
inline unsigned int floor_divide_signed(int dividend, int divisor) //!< Return dividend divided by divisor rounded down towards negative infinity.
{
    return (dividend / divisor) + (dividend * divisor > 0 ? 0 : -1);
}
inline unsigned int round_divide(unsigned int dividend, unsigned int divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    return (dividend + divisor / 2) / divisor;
}
inline unsigned int round_up_divide(unsigned int dividend, unsigned int divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    return (dividend + divisor - 1) / divisor;
}

}//namespace cura
#endif // UTILS_MATH_H

