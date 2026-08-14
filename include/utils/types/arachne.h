// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_ARACHNE_H
#define UTILS_TYPES_ARACHNE_H

#include <concepts>

#include <range/v3/range/concepts.hpp>

#include "utils/types/generic.h"
#include "utils/types/geometry.h"

namespace cura::utils
{

/*!
 * @brief A 2D point with an associated weight value
 * @details This concept is used to check if a type is a junction as used in wall toolpaths
 * @tparam T Type to check
 */
template<class T>
concept junction = requires(T val) {
    requires point2d<decltype(val.p)>;
    requires utils::integral<decltype(val.w)>;
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

} // namespace cura::utils

#endif // UTILS_TYPES_ARACHNE_H