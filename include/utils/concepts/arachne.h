// Copyright (c) 2023 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <concepts>
#include <ranges>

#ifndef CURAENGINE_ARACHNE_H
#define CURAENGINE_ARACHNE_H

namespace cura
{

template<class T>
concept isSTGraph = requires(T graph)
{
    { graph.edges } -> std::ranges::range;
    { graph.nodes } -> std::ranges::range;
};
;

} // namespace cura

#endif // CURAENGINE_ARACHNE_H
