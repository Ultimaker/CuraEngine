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
 * \param node the current visited node, should be convertable or have the same type as the nodes in graph and visited
 * \param graph A Graph as defined by \concept isGraph
 * \param state a state that propagate
 * \param handle_node Custom call back function called at each visited node. Arguments for the functions are the current node, and the state resulted from the parent node
 * \param visited nodes as defined by concept \isSet; _note: visited will not be ordered if it is of type `unordered_###`
 * \param parent_node the node in the graph that triggered the call to dfs on current_node.
 */

template <typename Node, typename State, isGraph Graph>
constexpr void dfs(
    const Node& current_node,
    const Graph& graph,
    std::function<State(const Node, const State)> handle_node,
    const State& state = nullptr,
    std::unordered_set<Node> visited = std::unordered_set<Node>(),
    std::function<std::vector<Node>(const Node, const Graph&)> get_neighbours = [](const Node current_node, const Graph& graph){
        const auto& [neighbour_begin, neighbour_end] = graph.equal_range(current_node);
        auto neighbours_iterator = ranges::make_subrange(neighbour_begin, neighbour_end);
        std::vector<Node> neighbours;
        for (const auto& [_, neighbour] : neighbours_iterator)
        {
            neighbours.push_back(neighbour);
        }
        return neighbours;
    }
)
{
    if (visited.contains(current_node))
    {
        return;
    }
    visited.emplace(current_node);

    auto current_state = handle_node(current_node, state);

    for (const auto& neighbour : get_neighbours(current_node, graph))
    {
        dfs(
            neighbour,
            graph,
            handle_node,
            current_state,
            visited,
            get_neighbours
        );
    }
}

template <typename Node>
constexpr void dfs_parent_view(
    const Node& current_node,
    const isGraph auto& graph,
    std::function<void(const Node, const Node)> handle_node)
{
    const std::function<Node(const Node, const Node)> parent_view =
        [handle_node](auto current_node, auto parent_node)
    {
        handle_node(current_node, parent_node);
        return current_node;
    };

    dfs(current_node, graph, parent_view);
}

template <typename Node>
constexpr void dfs_depth_view(
    const Node& current_node,
    const isGraph auto& graph,
    std::function<void(const Node, const unsigned int)> handle_node)
{
    const std::function<unsigned int(const Node, const unsigned int)> depth_view =
        [handle_node](auto current_node, auto depth)
    {
        handle_node(current_node, depth);
        return depth + 1;
    };

    dfs(current_node, graph, depth_view, 0u);
}

template <typename Node, isGraph Graph>
constexpr void dfs_conditional_neighbour_view(
    const Node& current_node,
    const Graph graph,
    std::function<void(const Node)> handle_node,
    std::unordered_set<Node> visited,
    std::function<std::vector<Node>(const Node, const Graph&)> get_neighbours)
{
    const std::function<std::nullptr_t(const Node, const std::nullptr_t)> wrapped_handle_node =
        [handle_node](auto current_node, auto)
    {
        handle_node(current_node);
        return nullptr;
    };

    dfs(current_node, graph, wrapped_handle_node, nullptr, visited, get_neighbours);
}
} // namespace cura::actions

#endif // CURAENGINE_DFS_SORT_H
