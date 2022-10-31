// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_ACTIONS_GET_LEAVES_H
#define UTILS_ACTIONS_GET_LEAVES_H

#include <concepts>
#include <type_traits>

#include <range/v3/algorithm/set_algorithm.hpp>
#include <range/v3/iterator/insert_iterators.hpp>
#include <range/v3/view/map.hpp>

namespace cura::actions
{

/* # isGraph
 * Describing the basic requirement for a directed graph, namely that it is a map, where the `key_type` and the `mapped_type`
 * exist and are of the same type.
 */
template<class T>
concept isGraph = requires(typename T::mapped_type value, typename T::key_type key)
{
    std::is_same<std::remove_const<decltype(key)>, std::remove_const<decltype(value)>>::value;
};

/* # get_leaves
 * Get the difference between the values and the keys of a graph
 *
 * \returns a vector of leaves or end-nodes
 */
constexpr auto get_leaves(isGraph auto map)
{
    std::vector<typename decltype(map)::mapped_type> leaves{};
    ranges::set_difference(map | ranges::views::values, map | ranges::views::keys, ranges::back_inserter(leaves));
    return leaves;
};

} // namespace cura::actions

#endif // UTILS_ACTIONS_GET_LEAVES_H
