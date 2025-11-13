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
    const int64_t quotient = dividend / divisor;
    const int64_t remainder = dividend % divisor;
    if (remainder != 0 && ((dividend > 0 && divisor < 0) || (dividend < 0 && divisor > 0)))
    {
        return quotient - 1;
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
    return (dividend + divisor - 1) / divisor;
}

/*!
 * \brief Calculates the "inverse linear interpolation" of a value over a range, i.e. given a range [min, max] the
 *        value "min" would give a result of 0.0 and the value "max" would give a result of 1.0, values in between will
 *        be interpolated linearly.
 * \note The returned value may be out of the [0.0, 1.0] range if the given value is outside the [min, max] range, it is
 *       up to the caller to clamp the result if required
 * \note The range_min value may be greater than the range_max, inverting the interpolation logic
 */
template<utils::numeric T>
[[nodiscard]] inline double inverse_lerp(T range_min, T range_max, T value)
{
    if (range_min == range_max)
    {
        return 0.0;
    }

    return static_cast<double>(value - range_min) / (range_max - range_min);
}

/*!
 * \brief Get a random floating point number in the range [0.0, 1.0]
 */
template<utils::floating_point T>
[[nodiscard]] inline T randf()
{
    return static_cast<T>(std::rand()) / static_cast<T>(RAND_MAX);
}

/*! \brief Check if a value is very close to 0 */
template<utils::floating_point T>
[[nodiscard]] bool is_null(T value)
{
    return std::abs(value) < (std::numeric_limits<T>::epsilon() * 100.0);
}

} // namespace cura
#endif // UTILS_MATH_H
