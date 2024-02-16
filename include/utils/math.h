// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include <cmath>
#include <cstdint>
#include <numbers>


namespace cura
{

template<typename T>
inline T square(const T& a)
{
    return a * a;
}

inline int64_t round_divide_signed(const int64_t dividend, const int64_t divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    if ((dividend < 0) ^ (divisor < 0)) // Either the numerator or the denominator is negative, so the result must be negative.
    {
        return (dividend - divisor / 2) / divisor; // Flip the .5 offset to do proper rounding in the negatives too.
    }
    else
    {
        return (dividend + divisor / 2) / divisor;
    }
}
inline uint64_t ceil_divide_signed(const int64_t dividend, const int64_t divisor) //!< Return dividend divided by divisor rounded up towards positive infinity.
{
    return static_cast<uint64_t>((dividend / divisor) + (dividend * divisor > 0 ? 1 : 0));
}
inline uint64_t floor_divide_signed(const int64_t dividend, const int64_t divisor) //!< Return dividend divided by divisor rounded down towards negative infinity.
{
    return static_cast<uint64_t>((dividend / divisor) + (dividend * divisor > 0 ? 0 : -1));
}
inline uint64_t round_divide(const uint64_t dividend, const uint64_t divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    return (dividend + divisor / 2) / divisor;
}
inline uint64_t round_up_divide(const uint64_t dividend, const uint64_t divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    return (dividend + divisor - 1) / divisor;
}

} // namespace cura
#endif // UTILS_MATH_H
