// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_ARACHNE_H
#define UTILS_TYPES_ARACHNE_H

#include "utils/types/generic.h"
#include "utils/types/geometry.h"

#include <range/v3/range/concepts.hpp>

#include <concepts>
#include <string>
#include <type_traits>

namespace cura::utils
{
// clang-format off
template<class T>
concept st_storable_data = requires(T val)
{
    val.data;
};

/*!
 * @brief A node in a skeleton trapezoidal graph, defined as a 2D point with additional stored data.
 * @details This concept is used to check if a type is a node in a straight skeleton graph. A node is a type that is a 2D point with additional stored data.
 * @tparam T Type to check
 */
template<class T>
concept st_node = requires(T val)
{
    requires point2d<decltype(val.p)>;
};

/*!
 * @brief A edge in a skeleton trapezoidal graph, defined as a 2D point with additional stored data.
 * @details This concept is used to check if a type is a edge in a skeleton trapezoidal graph. defined as a pair of nodes with pointers to the next, previous, and twin edges, and additional stored data.
 * @tparam T Type to check
 */
template<class T>
concept st_edge = requires(T val)
{
    requires std::is_pointer_v<decltype(val.from)>;
    requires st_node<decltype(*val.from)>;
    requires std::is_pointer_v<decltype(val.to)>;
    requires st_node<decltype(*val.to)>;
    requires std::is_pointer_v<decltype(val.twin)>;
    requires std::is_pointer_v<decltype(val.next)>;
    requires std::is_pointer_v<decltype(val.prev)>;
};

template<class T>
concept st_edges = requires(T edges)
{
    requires ranges::range<T>;
    requires st_edge<decltype(*ranges::begin(edges))>;
    requires st_storable_data<decltype(*ranges::begin(edges))>;
};

template<class T>
concept st_nodes = requires(T nodes)
{
    requires ranges::range<T>;
    requires st_node<decltype(*ranges::begin(nodes))>;
    requires st_storable_data<decltype(*ranges::begin(nodes))>;
};

/*!
 * @brief A skeleton trapezoidal graph, defined as a collection of nodes and edges.
 * @details This concept is used to check if a type is a skeleton trapezoidal graph.
 * @tparam T Type to check
 */
template<class T>
concept st_graph = requires(T graph)
{
    requires st_edges<decltype(graph.edges)>;
    requires st_nodes<decltype(graph.nodes)>;
};

/*!
 * @brief A 2D point with an associated weight value
 * @details This concept is used to check if a type is a junction as used in wall toolpaths
 * @tparam T Type to check
 */
template<class T>
concept junction = requires(T val)
{
    requires point2d<decltype(val.p)>;
    requires utils::integral<decltype(val.w)>;
};

/*!
 * @brief A collection of junctions
 * @details This concept is used to check if a type is a collection of junctions
 * @tparam T Type to check
 */
template<class T>
concept junctions = requires(T val)
{
    requires ranges::range<T>;
    requires junction<decltype(*ranges::begin(val))>;
};

/*!
 * @brief an Extrusion line in a toolpath
 * @details This concept is used to check if a type is a collection of junctions. A series of junctions defining a path for extrusion,
 * with additional flags indicating whether the path is closed and whether it corresponds to an odd or even layer.
 * @tparam T Type to check
 */
template<class T>
concept extrusion_line = requires(T val)
{
    std::is_same_v<decltype(val.inset_idx), size_t>;
    std::is_same_v<decltype(val.is_odd), bool>;
    std::is_same_v<decltype(val.is_closed), bool>;
    requires junctions<decltype(val.junctions)>;
};

/*!
 * @brief A collection of extrusion lines
 * @details This concept is used to check if a type is a collection of extrusion lines
 * @tparam T Type to check
 */
template<class T>
concept toolpath = requires(T tp)
{
    requires ranges::range<T>;
    requires extrusion_line<decltype(*ranges::begin(tp))>;
};

/*!
 * @brief A collection of toolpaths
 * @details This concept is used to check if a type is a collection of toolpaths
 * @tparam T Type to check
 */
template<class T>
concept toolpaths = requires(T tp)
{
    requires ranges::range<T>;
    requires toolpath<decltype(*ranges::begin(tp))>;
};
// clang-format on
} // namespace cura::utils

#endif // UTILS_TYPES_ARACHNE_H