//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_COORD_T_H
#define UTILS_COORD_T_H


//Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <polyclipping/clipper.hpp>

namespace cura
{

using coord_t = ClipperLib::cInt;

template<typename T>
constexpr T ipow(T x, unsigned power)
{
    return power != 0 ? x * ipow(x, power - 1) : T(1);
}

static constexpr unsigned INT10POW_PER_MM = 3;
static constexpr coord_t INT_PER_MM = ipow(10, INT10POW_PER_MM);
static constexpr coord_t INT_PER_MM2 = INT_PER_MM * INT_PER_MM;
static constexpr coord_t INT_PER_MM3 = INT_PER_MM2 * INT_PER_MM;

constexpr double coord_to_mm(coord_t n)
{
    return static_cast<double>(n) / INT_PER_MM;
}
constexpr double coord_to_mm2(coord_t n)
{
    return static_cast<double>(n) / INT_PER_MM2;
}
constexpr double coord_to_mm3(coord_t n)
{
    return static_cast<double>(n) / INT_PER_MM3;
}
constexpr coord_t mm_to_coord(double n)
{
    return static_cast<coord_t>(n * INT_PER_MM + (n >= 0. ? 0.5 : -0.5));
}
constexpr coord_t mm2_to_coord(double n)
{
    return static_cast<coord_t>(n * INT_PER_MM2 + (n >= 0. ? 0.5 : -0.5));
}
constexpr coord_t mm3_to_coord(double n)
{
    return static_cast<coord_t>(n * INT_PER_MM3 + (n >= 0. ? 0.5 : -0.5));
}

constexpr coord_t operator"" _mm(long double n)
{
    return mm_to_coord(n);
}
constexpr coord_t operator"" _mm(unsigned long long int n)
{
    return mm_to_coord(n);
}
constexpr coord_t operator"" _mm2(long double n)
{
    return mm2_to_coord(n);
}
constexpr coord_t operator"" _mm2(unsigned long long int n)
{
    return mm2_to_coord(n);
}
constexpr coord_t operator"" _mm3(long double n)
{
    return mm3_to_coord(n);
}
constexpr coord_t operator"" _mm3(unsigned long long int n)
{
    return mm3_to_coord(n);
}

constexpr coord_t operator"" _mu(long double n)
{
    return mm_to_coord(n * 1e-3);
}
constexpr coord_t operator"" _mu(unsigned long long int n)
{
    return mm_to_coord(n * 1e-3);
}
constexpr coord_t operator"" _mu2(long double n)
{
    return mm2_to_coord(n * 1e-6);
}
constexpr coord_t operator"" _mu2(unsigned long long int n)
{
    return mm2_to_coord(n * 1e-6);
}
constexpr coord_t operator"" _mu3(long double n)
{
    return mm3_to_coord(n * 1e-9);
}
constexpr coord_t operator"" _mu3(unsigned long long int n)
{
    return mm3_to_coord(n * 1e-9);
}

} // namespace cura


#endif // UTILS_COORD_T_H
