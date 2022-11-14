// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_TOPOLOGICAL_ORDER_H
#define CURAENGINE_TOPOLOGICAL_ORDER_H

#include <range/v3/algorithm/contains.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/subrange.hpp>
#include <range/v3/view/transform.hpp>

#include "utils/concepts/graph.h"

namespace cura::actions
{
constexpr void topological_order(const auto& node, const isGraph auto& dag, isSet auto& visited, auto&& proj)
{
    visited.push_back(node);
    const auto& [children_begin, children_end] = dag.equal_range(node);
    auto children = ranges::make_subrange(children_begin, children_end) | ranges::views::values | ranges::to_vector;
    ranges::sort(children, {}, proj);
    for (const auto& child : children)
    {
        topological_order(child, dag, visited, proj);
    }
}
} // namespace cura::actions

#endif // CURAENGINE_TOPOLOGICAL_ORDER_H
