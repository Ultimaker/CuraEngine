// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_COORD_T_H
#define UTILS_COORD_T_H


// Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <cmath>
#include <polyclipping/clipper.hpp>

#include "utils/types/generic.h"

namespace cura
{

using coord_t = ClipperLib::cInt;

constexpr coord_t EPSILON = 5;

static inline coord_t operator""_mu(unsigned long long i)
{
    return static_cast<coord_t>(i);
}

#define INT2MM(n) (static_cast<double>(n) / 1000.0)
#define INT2MM2(n) (static_cast<double>(n) / 1000000.0)
#define MM2INT(n) (static_cast<coord_t>((n) * 1000 + 0.5 * (((n) > 0) - ((n) < 0))))
#define MM2_2INT(n) (static_cast<coord_t>((n) * 1000000 + 0.5 * (((n) > 0) - ((n) < 0))))
#define MM3_2INT(n) (static_cast<coord_t>((n) * 1000000000 + 0.5 * (((n) > 0) - ((n) < 0))))

#define INT2MICRON(n) ((n) / 1)
#define MICRON2INT(n) ((n) * 1)

template<utils::floating_point FactorType>
[[nodiscard]] inline coord_t lerp(coord_t a, coord_t b, FactorType t)
{
    return std::llrint(std::lerp(static_cast<double>(a), static_cast<double>(b), t));
}

/*! Returns true if the given value is null or small enough to be considered null */
[[nodiscard]] inline bool fuzzy_is_zero(const coord_t value)
{
    return std::abs(value) <= EPSILON;
}

/*! Returns true if the given values are equal or close enough to be considered equal */
[[nodiscard]] inline bool fuzzy_equal(const coord_t a, const coord_t b)
{
    return fuzzy_is_zero(b - a);
}

/*! Returns true if the given values are not equal and different enough to be considered not equal */
[[nodiscard]] inline bool fuzzy_not_equal(const coord_t a, const coord_t b)
{
    return ! fuzzy_equal(a, b);
}

/*! Returns true if the given \a value is greater enough to \b to be considered greater */
[[nodiscard]] inline bool fuzzy_is_greater(const coord_t a, const coord_t b)
{
    return a - b > EPSILON;
}

/*! Returns true if the given \a value is greater or close enough to \b to be considered greater or equal */
[[nodiscard]] inline bool fuzzy_is_greater_or_equal(const coord_t a, const coord_t b)
{
    return a > b - EPSILON;
}

/*! Returns true if the given \a value is lesser enough to \b to be considered lesser */
[[nodiscard]] inline bool fuzzy_is_lesser(const coord_t a, const coord_t b)
{
    return b - a > EPSILON;
}

/*! Returns true if the given \a value is lesser or close enough to \b to be considered lesser or equal */
[[nodiscard]] inline bool fuzzy_is_lesser_or_equal(const coord_t a, const coord_t b)
{
    return b > a - EPSILON;
}

} // namespace cura


#endif // UTILS_COORD_T_H
