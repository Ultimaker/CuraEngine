//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "MinimumSpanningTree.h"

namespace cura
{

MinimumSpanningTree::MinimumSpanningTree(std::unordered_set<Point> vertices) : adjacency_graph(prim(vertices))
{
    //Just copy over the fields.
}

MinimumSpanningTree::Edge::Edge(const Point start, const Point end) : start(start), end(end)
{
    //Just copy over the fields.
}

int MinimumSpanningTree::Edge::length() const
{
    return vSize2(start - end);
}

const std::unordered_map<Point, std::vector<MinimumSpanningTree::Edge>> MinimumSpanningTree::prim(std::unordered_set<Point> vertices) const
{
    std::unordered_map<Point, std::vector<Edge>> result;
    if (vertices.empty())
    {
        return result; //No vertices, so we can't create edges either.
    }
    result.reserve(vertices.size());
    std::vector<Point> vertices_list;
    for (Point vertex : vertices)
    {
        vertices_list.push_back(vertex);
    }

    Point first_point = vertices_list[0];
    result[first_point] = std::vector<MinimumSpanningTree::Edge>(); //Start with one vertex in the tree.

    if (vertices_list.size() == 1)
    {
        return result; //If there's only one vertex, we can't go creating any edges.
    }

    std::unordered_map<Point*, coord_t> smallest_distance; //The shortest distance to the current tree.
    smallest_distance.reserve(vertices_list.size());
    std::unordered_map<Point*, Point*> smallest_distance_to; //Which point the shortest distance goes towards.
    smallest_distance_to.reserve(vertices_list.size());
    for (size_t vertex_index = 0; vertex_index < vertices_list.size(); vertex_index++)
    {
        if (vertices_list[vertex_index] == first_point)
        {
            continue;
        }
        smallest_distance[&vertices_list[vertex_index]] = vSize2(vertices_list[vertex_index] - first_point);
        smallest_distance_to[&vertices_list[vertex_index]] = &vertices_list[0];
    }

    while(result.size() < vertices_list.size()) //All of the vertices need to be in the tree at the end.
    {
        //Choose the closest vertex to connect to that is not yet in the tree.
        //This search is O(V) right now, which can be made down to O(log(V)). This reduces the overall time complexity from O(V*V) to O(V*log(E)).
        //However that requires an implementation of a heap that supports the decreaseKey operation, which is not in the std library.
        //TODO: Implement this?
        Point* closest_point = nullptr;
        coord_t closest_distance = std::numeric_limits<coord_t>::max();
        for(std::pair<Point*, coord_t> point_and_distance : smallest_distance)
        {
            if (point_and_distance.second < closest_distance) //This one's closer!
            {
                closest_point = point_and_distance.first;
                closest_distance = point_and_distance.second;
            }
        }

        //Add this point to the graph and remove it from the candidates.
        Point closest_point_local = *closest_point;
        Point other_end = *smallest_distance_to[closest_point];
        if (result.find(closest_point_local) == result.end())
        {
            result[closest_point_local] = std::vector<Edge>();
        }
        result[closest_point_local].emplace_back(closest_point_local, other_end);
        if (result.find(other_end) == result.end())
        {
            result[other_end] = std::vector<Edge>();
        }
        result[other_end].emplace_back(other_end, closest_point_local);
        smallest_distance.erase(closest_point); //Remove it so we don't check for these points again.
        smallest_distance_to.erase(closest_point);

        //Update the distances of all points that are not in the graph.
        for (std::pair<Point*, coord_t> point_and_distance : smallest_distance)
        {
            coord_t new_distance = vSize2(*closest_point - *point_and_distance.first);
            if (new_distance < point_and_distance.second) //New point is closer.
            {
                smallest_distance[point_and_distance.first] = new_distance;
                smallest_distance_to[point_and_distance.first] = closest_point;
            }
        }
    }

    return result;
}

const std::vector<Point> MinimumSpanningTree::adjacentNodes(Point node) const
{
    std::vector<Point> result;
    std::unordered_map<Point, std::vector<Edge>>::const_iterator adjacency_entry = adjacency_graph.find(node);
    if (adjacency_entry != adjacency_graph.end())
    {
        for (const Edge edge : (*adjacency_entry).second)
        {
            //Get the opposite side.
            if (edge.start == node)
            {
                result.push_back(edge.end);
            }
            else
            {
                result.push_back(edge.start);
            }
        }
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
    for (std::pair<Point, std::vector<Edge>> node : adjacency_graph)
    {
        result.push_back(node.first);
    }
    return result;
}

}