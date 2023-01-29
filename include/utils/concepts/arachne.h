// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <concepts>
#include <ranges>

#include "utils/concepts/geometry.h"

#ifndef CURAENGINE_ARACHNE_H
#define CURAENGINE_ARACHNE_H

namespace cura
{

template<class T>
concept storable_data =requires(T val) { val.data; };

template<class T>
concept st_edges_viewable = std::ranges::range<T> && storable_data<typename T::value_type>;

template<class T>
concept st_nodes_viewable = std::ranges::range<T> && storable_data<typename T::value_type>;

template<class T>
concept st_graph = requires(T graph)
{
    { graph.edges } -> st_edges_viewable;
    { graph.nodes } -> st_nodes_viewable;
};

template<class T>
concept junction = requires (T val)
{
    { val.p } -> point;
    { val.w } -> std::integral;
};

template<class T>
concept extrusion_line = requires(T val)
{
    val.inset_idx;
    val.is_odd;
    val.is_closed;
    { val.junctions } -> std::ranges::range;
};

template<class T>
concept toolpath = std::ranges::range<T> && extrusion_line<typename T::value_type>;

template<class T>
concept toolpaths = std::ranges::range<T> && toolpath<typename T::value_type>;

}// namespace cura

#endif// CURAENGINE_ARACHNE_H
