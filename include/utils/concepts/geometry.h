// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GEOMETRY_H
#define UTILS_CONCEPTS_GEOMETRY_H

#include <concepts>
#include <ranges>
#include <string>
#include <type_traits>

#include "utils/polygon.h" // FIXME: remove once proper concept is defined


namespace cura
{

template<class T>
concept isPolygon = std::is_base_of_v<ConstPolygonRef, T>; // FIXME: define proper concept

template<class T>
concept isPolygons = std::is_base_of_v<Polygons, T>; // FIXME: define proper concept

template<class T>
concept isMesh = requires(T mesh)
{
    { mesh.vertices };
    { mesh.faces };
    {
        mesh.mesh_name
    } -> std::convertible_to<std::string>;

}; // FIXME: define proper concept

template<class T>
concept isLayer = requires(T layer)
{
    { layer.z };
};

template<class T>
concept isLayers = std::ranges::range<T> && isLayer<typename T::value_type>;

} // namespace cura

#endif // UTILS_CONCEPTS_GEOMETRY_H
