// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_DFS_SORT_H
#define CURAENGINE_DFS_SORT_H

#include <range/v3/algorithm/contains.hpp>
#include <range/v3/view/subrange.hpp>

#include "utils/concepts/graph.h"

namespace cura::actions
{
/* # dfs utility
 *
 * Walks through an arbitrary graph using depth-first-search and calls a custom callback at each visited node
 * \param node the current visited node, should be convertable or have the same type as the nodes in dag and visited
 * \param dag Directed Acyclic Graph as defined by \concept isGraph
 * \param state a state that p
 * \param handle_node Custom call back function called at each visited node. Arguments for the functions are the current node, and the state resulted from the parent node
 * \param visited nodes as defined by concept \isSet; _note: visited will not be ordered if it is of type `unordered_###`
 */

template <typename Node, typename State>
constexpr void dfs(
    const Node& current_node,
    const isGraph auto& dag,
    const State& state,
    std::function<State(const Node, const State)> handle_node,
    isSet auto& visited)
{
    if (ranges::contains(visited, current_node))
    {
        //        return DFSAction::ContinueSearch;
        return;
    }
    visited.emplace(current_node);

    //    const auto handle_node_result = handle_node(current_node);
    //    switch (handle_node_result)
    //    {
    //        case DFSAction::StopBranch: return DFSAction::ContinueSearch;
    //        case DFSAction::StopSearch: return DFSAction::StopSearch;
    //    }

    auto current_state = handle_node(current_node, state);

    const auto& [children_begin, children_end] = dag.equal_range(current_node);
    auto children = ranges::make_subrange(children_begin, children_end);
    for (const auto& [_, child_node] : children)
    {
        dfs(
            child_node,
            dag,
            current_state,
            handle_node,
            visited
        );

        //        const auto dfs_result = dfs(child_node, current_node, dag, visited);
        //        switch (dfs_result)
        //        {
        //            case DFSAction::StopSearch: return DFSAction::StopSearch;
        //        }
    }

    //    return DFSAction::ContinueSearch;
}
} // namespace cura::actions

#endif // CURAENGINE_DFS_SORT_H