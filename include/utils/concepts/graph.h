// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GRAPH_H
#define UTILS_CONCEPTS_GRAPH_H

#include <concepts>
#include <type_traits>

namespace cura
{
/* # isGraph
 * Describing the basic requirement for a directed graph, namely that it is a map, where the `key_type` and the `mapped_type`
 * exist and are of the same type.
 */
template<class T>
concept isGraph = std::is_same<T, std::unordered_map<typename T::key_type, typename T::mapped_type>>::value || std::is_same<T, std::unordered_multimap<typename T::key_type, typename T::mapped_type>>::value;
} // namespace cura

#endif // UTILS_CONCEPTS_GRAPH_H
