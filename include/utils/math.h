// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include <cmath>
#include <cstdint>

#include "utils/types/generic.h"


namespace cura
{

/**
 * @brief Returns the square of a value.
 *
 * @tparam T A multipliable type (arithmetic types such as int, float, double, etc.)
 * @param a The value to be squared.
 * @return T The square of the input value.
 */
template<utils::multipliable T>
[[nodiscard]] T square(const T& a)
{
    return a * a;
}

/**
 * @brief Returns the quotient of the division of two signed integers, rounded to the nearest integer.
 *
 * @param dividend The numerator.
 * @param divisor The denominator (must not be zero).
 * @return int64_t The result of the division rounded to the nearest integer.
 * @throws std::invalid_argument If the divisor is zero.
 */
[[nodiscard]] inline int64_t round_divide_signed(const int64_t dividend, const int64_t divisor)
{
    if (divisor == 0)
    {
        throw std::invalid_argument("Divisor cannot be zero");
    }

    if ((dividend < 0) ^ (divisor < 0))
    {
        return (dividend - divisor / 2) / divisor;
    }
    return (dividend + divisor / 2) / divisor;
}

/**
 * @brief Returns the quotient of the division of two signed integers, rounded up towards positive infinity.
 *
 * @param dividend The numerator.
 * @param divisor The denominator (must not be zero).
 * @return int64_t The result of the division rounded up.
 * @throws std::invalid_argument If the divisor is zero.
 */
[[nodiscard]] inline int64_t ceil_divide_signed(const int64_t dividend, const int64_t divisor)
{
    if (divisor == 0)
    {
        throw std::invalid_argument("Divisor cannot be zero");
    }

    int64_t quotient = dividend / divisor;
    int64_t remainder = dividend % divisor;

    // Round up if there's a remainder and the signs of dividend and divisor are the same
    if (remainder != 0 && ((dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0)))
    {
        quotient += 1;
    }

    return quotient;
}

/**
 * @brief Returns the quotient of the division of two signed integers, rounded down towards negative infinity.
 *
 * @param dividend The numerator.
 * @param divisor The denominator (must not be zero).
 * @return int64_t The result of the division rounded down.
 * @throws std::invalid_argument If the divisor is zero.
 */
[[nodiscard]] inline int64_t floor_divide_signed(const int64_t dividend, const int64_t divisor)
{
    if (divisor == 0)
    {
        throw std::invalid_argument("Divisor cannot be zero");
    }

    int64_t quotient = dividend / divisor;
    int64_t remainder = dividend % divisor;

    if (remainder != 0 && ((dividend > 0 && divisor < 0) || (dividend < 0 && divisor > 0)))
    {
        quotient -= 1;
    }

    return quotient;
}

/**
 * @brief Returns the quotient of the division of two unsigned integers, rounded to the nearest integer.
 *
 * @param dividend The numerator.
 * @param divisor The denominator (must not be zero).
 * @return uint64_t The result of the division rounded to the nearest integer.
 */
[[nodiscard]] inline uint64_t round_divide(const uint64_t dividend, const uint64_t divisor)
{
    if (divisor == 0)
    {
        throw std::invalid_argument("Divisor cannot be zero");
    }

    return (dividend + divisor / 2) / divisor;
}

/**
 * @brief Returns the quotient of the division of two unsigned integers, rounded up towards positive infinity.
 *
 * @param dividend The numerator.
 * @param divisor The denominator (must not be zero).
 * @return uint64_t The result of the division rounded up.
 */
[[nodiscard]] inline uint64_t round_up_divide(const uint64_t dividend, const uint64_t divisor)
{
    if (divisor == 0)
    {
        throw std::invalid_argument("Divisor cannot be zero");
    }

    return (dividend + divisor - 1) / divisor;
}

} // namespace cura
#endif // UTILS_MATH_H
