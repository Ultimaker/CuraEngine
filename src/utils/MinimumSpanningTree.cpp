//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "MinimumSpanningTree.h"

#include <iterator>
#include <algorithm>

namespace cura
{

MinimumSpanningTree::MinimumSpanningTree(std::unordered_set<Point> vertices) : adjacency_graph(prim(vertices))
{
    //Just copy over the fields.
}

auto MinimumSpanningTree::prim(std::unordered_set<Point> vertices) const -> AdjacencyGraph_t
{
    AdjacencyGraph_t result;
    if (vertices.empty())
    {
        return result; //No vertices, so we can't create edges either.
    }
    // If there's only one vertex, we can't go creating any edges so just add the point to the adjacency list with no
    // edges
    if (vertices.size() == 1)
    {
        // unordered_map::operator[]() will construct an empty vector in place for us when we try and access an element
        // that doesnt exist
        result[*vertices.begin()];
        return result;
    }
    result.reserve(vertices.size());
    std::vector<Point> vertices_list(vertices.begin(), vertices.end());

    std::unordered_map<const Point*, coord_t> smallest_distance;    //The shortest distance to the current tree.
    std::unordered_map<const Point*, const Point*> smallest_distance_to; //Which point the shortest distance goes towards.
    smallest_distance.reserve(vertices_list.size());
    smallest_distance_to.reserve(vertices_list.size());
    for (size_t vertex_index = 1; vertex_index < vertices_list.size(); vertex_index++)
    {
        const auto& vert = vertices_list[vertex_index];
        smallest_distance[&vert] = vSize2(vert - vertices_list[0]);
        smallest_distance_to[&vert] = &vertices_list[0];
    }

    while (result.size() < vertices_list.size()) //All of the vertices need to be in the tree at the end.
    {
        //Choose the closest vertex to connect to that is not yet in the tree.
        //This search is O(V) right now, which can be made down to O(log(V)). This reduces the overall time complexity from O(V*V) to O(V*log(E)).
        //However that requires an implementation of a heap that supports the decreaseKey operation, which is not in the std library.
        //TODO: Implement this?
        using MapValue = std::pair<const Point*, coord_t>;
        const auto closest = std::min_element(smallest_distance.begin(), smallest_distance.end(),
                                              [](const MapValue& a, const MapValue& b) {
                                                  return a.second < b.second;
                                              });

        //Add this point to the graph and remove it from the candidates.
        const Point* closest_point = closest->first;
        const Point other_end = *smallest_distance_to[closest_point];
        if (result.find(*closest_point) == result.end())
        {
            result[*closest_point] = std::vector<Edge>();
        }
        result[*closest_point].push_back({*closest_point, other_end});
        if (result.find(other_end) == result.end())
        {
            result[other_end] = std::vector<Edge>();
        }
        result[other_end].push_back({other_end, *closest_point});
        smallest_distance.erase(closest_point); //Remove it so we don't check for these points again.
        smallest_distance_to.erase(closest_point);

        //Update the distances of all points that are not in the graph.
        for (std::pair<const Point*, coord_t> point_and_distance : smallest_distance)
        {
            const coord_t new_distance = vSize2(*closest_point - *point_and_distance.first);
            const coord_t old_distance = point_and_distance.second;
            if (new_distance < old_distance) //New point is closer.
            {
                smallest_distance[point_and_distance.first] = new_distance;
                smallest_distance_to[point_and_distance.first] = closest_point;
            }
        }
    }

    return result;
}

std::vector<Point> MinimumSpanningTree::adjacentNodes(Point node) const
{
    std::vector<Point> result;
    AdjacencyGraph_t::const_iterator adjacency_entry = adjacency_graph.find(node);
    if (adjacency_entry != adjacency_graph.end())
    {
        const auto& edges = adjacency_entry->second;
        std::transform(edges.begin(), edges.end(), std::back_inserter(result),
                       [&node](const Edge& e) { return (e.start == node) ? e.end : e.start; });
    }
    return result;
}

std::vector<Point> MinimumSpanningTree::leaves() const
{
    std::vector<Point> result;
    for (std::pair<Point, std::vector<Edge>> node : adjacency_graph)
    {
        if (node.second.size() <= 1) //Leaves are nodes that have only one adjacent edge, or just the one node if the tree contains one node.
        {
            result.push_back(node.first);
        }
    }
    return result;
}

std::vector<Point> MinimumSpanningTree::vertices() const
{
    std::vector<Point> result;
    using MapValue = std::pair<Point, std::vector<Edge>>; 
    std::transform(adjacency_graph.begin(), adjacency_graph.end(), std::back_inserter(result),
                   [](const MapValue& node) { return node.first; });
    return result;
}

}
