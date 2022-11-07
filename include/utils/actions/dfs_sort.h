// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_DFS_SORT_H
#define CURAENGINE_DFS_SORT_H

#include <range/v3/algorithm/contains.hpp>
#include <range/v3/view/subrange.hpp>

#include "utils/concepts/graph.h"

namespace cura::actions
{
/* # dfs_sort
 *
 * Walks through a directed graph using depth-first-search and adds the nodes it visited to the visited set
 * \param node the current visited node, should be convertable or have the same type as the nodes in dag and visited (currently not constrained by a `require`)
 * \param dag Directed Acyclic Graph as defined by \concept isGraph
 * \param visited nodes as defined by concept \isSet; _note: visited will not be ordered if it is of type `unordered_###`
 */
constexpr void dfs_sort(const auto& node, const isGraph auto& dag, isSet auto& visited)
{
    if (ranges::contains(visited, node))
    {
        return;
    }
    visited.push_back(node);
    const auto& [children_begin, children_end] = dag.equal_range(node);
    auto children = ranges::make_subrange(children_begin, children_end);
    for (const auto& [_, child] : children)
    {
        dfs_sort(child, dag, visited);
    }
}
} // namespace cura::actions

#endif // CURAENGINE_DFS_SORT_H
