// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once


// Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <cmath>
#include <polyclipping/clipper.hpp>

#include "types/generic.h"
#include "utils/math.h"

namespace curaengine::types
{

using coord_t = ClipperLib::cInt;

constexpr coord_t EPSILON = 5;
constexpr coord_t EPSILON_SQUARED = utils::square(EPSILON);

static constexpr coord_t operator""_mu(unsigned long long i)
{
    return static_cast<coord_t>(i);
}

[[nodiscard]] constexpr double INT2MM(coord_t n)
{
    return static_cast<double>(n) / 1000.0;
}

[[nodiscard]] constexpr double INT2MM2(coord_t n)
{
    return static_cast<double>(n) / 1000000.0;
}

[[nodiscard]] constexpr coord_t MM2INT(double n)
{
    return static_cast<coord_t>(n * 1000 + 0.5 * ((n > 0) - (n < 0)));
}

[[nodiscard]] constexpr coord_t MM2_2INT(double n)
{
    return static_cast<coord_t>(n * 1000000 + 0.5 * ((n > 0) - (n < 0)));
}

[[nodiscard]] constexpr coord_t MM3_2INT(double n)
{
    return static_cast<coord_t>(n * 1000000000 + 0.5 * ((n > 0) - (n < 0)));
}

[[nodiscard]] constexpr coord_t INT2MICRON(coord_t n)
{
    return n;
}

[[nodiscard]] constexpr coord_t MICRON2INT(coord_t n)
{
    return n;
}

template<types::floating_point FactorType>
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

} // namespace curaengine::types
