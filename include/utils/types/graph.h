// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GRAPH_H
#define UTILS_CONCEPTS_GRAPH_H

#include "utils/types/generic.h"

#include <concepts>
#include <type_traits>

namespace cura::utils
{
// clang-format off

/* # nodable
 * Describing the basic requirement for a node in a graph.
 */
template<class T>
concept nodeable = hashable<T>;

/* # graphable
 * Describing the basic requirement for a directed graph, namely that it is a map, where the `key_type` and the `mapped_type`
 * exist and are of the same type.
 */
template<class T>
concept graphable =
    nodeable<typename T::key_type> && (std::is_same<T, std::unordered_map<typename T::key_type, typename T::mapped_type>>::value || std::is_same<T, std::unordered_multimap<typename T::key_type, typename T::mapped_type>>::value);

/* # setable
 * Describing the basic requirement for a set.
 */
template<class T>
concept setable = nodeable<typename T::key_type>
               && (std::is_same<T, std::vector<typename T::value_type>>::value || std::is_same<T, std::unordered_multiset<typename T::value_type>>::value || std::is_same<T, std::unordered_set<typename T::value_type>>::value);

// clang-format off
} // namespace cura

#endif // UTILS_CONCEPTS_GRAPH_H
