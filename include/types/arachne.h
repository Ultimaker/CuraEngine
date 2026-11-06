// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <concepts>

#include <range/v3/range/concepts.hpp>

#include "types/generic.h"
#include "types/geometry.h"

namespace curaengine::types
{

/*!
 * @brief A 2D point with an associated weight value
 * @details This concept is used to check if a type is a junction as used in wall toolpaths
 * @tparam T Type to check
 */
template<class T>
concept junction = requires(T val) {
    requires point2d<decltype(val.p)>;
    requires integral<decltype(val.w)>;
};

/*!
 * @brief A collection of junctions
 * @details This concept is used to check if a type is a collection of junctions
 * @tparam T Type to check
 */
template<class T>
concept junctions = requires(T val) {
    requires ranges::range<T>;
    requires junction<decltype(*ranges::begin(val))>;
};

} // namespace curaengine::types
