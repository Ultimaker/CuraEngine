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

static inline coord_t operator"" _mu(unsigned long long i)
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

} // namespace cura


#endif // UTILS_COORD_T_H
