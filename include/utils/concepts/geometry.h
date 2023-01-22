// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_CONCEPTS_GEOMETRY_H
#define INCLUDE_UTILS_CONCEPTS_GEOMETRY_H

#include <concepts>
#include <ranges>
#include <string>
#include <type_traits>

#include "utils/polygon.h"// FIXME: remove once proper concept is defined

namespace cura
{

template<class T>
concept point2d_named = requires( T point )
{
    point.X;
    point.Y;
};

template<class T>
concept point2d = point2d_named<T>  || (std::ranges::range<T> && std::integral<typename T::value_type> && std::tuple_size_v<T> == 2);;

template<class T>
concept point3d_named = requires( T point )
{
    point.x;
    point.y;
    point.z;
};

template<class T>
concept point3d = point3d_named<T> || (std::ranges::range<T> && std::integral<typename T::value_type> && std::tuple_size_v<T> == 3);

template<class T>
concept point_named = point2d_named<T> || point3d_named<T>;

template<class T>
concept point = point2d<T> || point3d<T>;

template<class T>
concept point_ranged = point<T> && ! point2d_named<T> && ! point3d_named<T>;

template<class T>
concept polygon = std::is_base_of_v<ConstPolygonRef, T>;// FIXME: define proper concept

template<class T>
concept polygons = std::is_base_of_v<Polygons, T>;// FIXME: define proper concept

template<class T>
concept vertex = requires( T vert )
{
    { vert.p } -> point3d;
    { vert.connected_faces } -> std::ranges::range;
};

template<class T>
concept vertices = std::ranges::range<T> && vertex<typename T::value_type>;

template<class T>
concept face = requires( T f )
{
    { f.vertex_index } -> std::ranges::range;
    { f.connected_face_index } -> std::ranges::range;
};

template<class T>
concept faces = std::ranges::range<T> && face<typename T::value_type>;

template<class T>
concept mesh =
requires( T m )
{
    m.vertices;
    m.faces;
	{ m.mesh_name } -> std::convertible_to<std::string>;
};// FIXME: define proper concept

template<class T>
concept layer =
requires( T layer ) { layer.z; };

template<class T>
concept layer_viewable = std::ranges::range<T> && layer < typename T::value_type>;

}// namespace cura

#endif// INCLUDE_UTILS_CONCEPTS_GEOMETRY_H
