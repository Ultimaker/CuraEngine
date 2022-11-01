// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_ACTIONS_LEAVES_H
#define UTILS_ACTIONS_LEAVES_H

#include "utils/concepts/graph.h"

#include <range/v3/algorithm/set_algorithm.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/iterator/insert_iterators.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/unique.hpp>

namespace cura::actions
{

/* # leaves
 * Get the leaves from a graph
 *
 * \returns a vector of leaves
 */
constexpr auto leaves(isGraph auto map)
{
    std::vector<typename decltype(map)::mapped_type> leaves{};
    auto values = map | ranges::views::values | ranges::views::unique | ranges::to_vector;
    ranges::sort(values);
    auto keys = map | ranges::views::keys | ranges::views::unique | ranges::to_vector;
    ranges::sort(keys);
    ranges::set_difference(values, keys, ranges::back_inserter(leaves));
    return leaves;
};

} // namespace cura::actions

#endif // UTILS_ACTIONS_LEAVES_H
