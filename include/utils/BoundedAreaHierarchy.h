// Copyright (c) 2023 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "AABB.h"
#include "utils/IntPoint.h"
#include <sstream>

#ifndef CURAENGINE_BOUNDEDAREAHIERARCHY_H
#define CURAENGINE_BOUNDEDAREAHIERARCHY_H

namespace cura
{
template<typename edge_data>
class BoundedAreaHierarchy
{
    struct Edge
    {
        const Point start;
        const Point end;

        Edge(const Point start, const Point end) : start(start), end(end)
        {
        }

        AABB getAABB() const
        {
            AABB aabb;
            aabb.include(start);
            aabb.include(end);
            return aabb;
        }
    };

    struct Node
    {
        AABB aabb;
        // if child_a is 0, then this is a leaf node, child_b
        // is the index of the edge in the edges vector
        // this works as the first element of the bounded_area_hierarchy-tree
        // is the root; no child points to the root in a tree data structure
        int child_a;
        int child_b;
    };

    std::vector<std::pair<Edge, edge_data>> edges;
    std::vector<Node> bounded_area_hierarchy;

    /*!
     * \brief calculate_bounds
     * \param start index of the first edge to include
     * \param end index of the last edge to include (inclusive)
     * \return the bounding box of the edges
     */
    AABB calculate_bounds(const int start, const int end)
    {
        AABB aabb;
        for (int i = start; i <= end; i++)
        {
            aabb.include(edges[i].first.getAABB());
        }
        return aabb;
    }

    // Quicksort algorithm based on https://www.geeksforgeeks.org/cpp-program-for-quicksort/
    void quick_sort_primitives(const int axis, const int first, const int last) {
        if (first < last) {
            float pivot = get_axis(axis, vertices[indices[(first + last) / 2] * 3]);

            int i = first - 1;

            for (int j = first; j <= last - 1; j ++) {
                if (get_axis(axis, centroids[indices[j]]) <= pivot) {
                    i++;
                    Swap(&indices[i], &indices[j]);
                }
            }
            Swap(&indices[i + 1], &indices[last]);

            int pivotIndex = i + 1;

            QuickSortPrimitives(axis, first, pivotIndex - 1);
            QuickSortPrimitives(axis, pivotIndex + 1, last);
        }
    }

    void construct_search_structure(const int start, const int end)
    {
        if (end - start == 1)
        {
            bounded_area_hierarchy[start].child_a = 0;
            bounded_area_hierarchy[start].child_b = start;
            return;
        }

        edges.sort(start, end, [](const std::pair<Edge, edge_data>& a, const std::pair<Edge, edge_data>& b) {
            return a.first.getAABB().getCenter().x < b.first.getAABB().getCenter().x;
        });


        const int middle = (start + end) / 2;
        construct_search_structure(start, middle);
        construct_search_structure(middle, end);
        bounded_area_hierarchy[start].child_a = middle;
        bounded_area_hierarchy[start].child_b = 0;
        bounded_area_hierarchy[start].aabb = bounded_area_hierarchy[start].aabb.combine(bounded_area_hierarchy[middle].aabb);
    }

public:
    BoundedAreaHierarchy();

    /*!
     * \brief insert_edge inserts an edge into the bounded area hierarchy
     * \param start the start point of the edge
     * \param end the end point of the edge
     * \param data the data associated with the edge
     */
    void insert_edge(const Point start, const Point end, const edge_data data)
    {
        edges.emplace_back(Edge(start, end), data);
    }

    void draw_debug_svg(const std::string file_name)
    {
        if (! bounded_area_hierarchy.empty()) return;

        AABB outer_aabb = bounded_area_hierarchy[0].aabb;
        outer_aabb.expand(1000);
        SVG svg(file_name, outer_aabb);

        std::vector<int> stack;
        stack.push_back(0);
        while (!stack.empty())
        {
            const int node_index = stack.back();
            stack.pop_back();
            const Node& node = bounded_area_hierarchy[node_index];
            const AABB aabb = node.aabb;
            const Polygon polygons = aabb.toPolygon();

            svg.writePolygon(polygons);

            if (node.child_a == 0)
            {
                const int edge_index = node.child_b;
                const Edge& edge = edges[edge_index].first;
                svg.writeLine(edge.start, edge.end);
            }
            else
            {
                stack.push_back(node.child_a);
                stack.push_back(node.child_b);
            }
        }
    }

    void construct_search_structure()
    {
        bounded_area_hierarchy.clear();
        bounded_area_hierarchy.reserve(edges.size() * 2 - 1);
        for (const auto& edge : edges)
        {
            bounded_area_hierarchy.emplace_back(Node{ edge.first.getAABB(), 0, 0 });
        }
        construct_search_structure(0, edges.size());
    }

    void radius_search(const Point point, const coord_t radius, std::vector<edge_data> result) const
    {
        std::vector<int> stack;
        stack.push_back(0);
        while (! stack.empty())
        {
            const int node_index = stack.back();
            stack.pop_back();
            const Node& node = bounded_area_hierarchy[node_index];
            if (node.child_a == 0)
            {
                const int edge_index = node.child_b;
                const Edge& edge = edges[edge_index].first;
                if (edge.start.distance_to(point) < radius || edge.end.distance_to(point) < radius)
                {
                    result.push_back(edges[edge_index].second);
                }
            }
            else
            {
                const Node& child_a = bounded_area_hierarchy[node.child_a];
                const Node& child_b = bounded_area_hierarchy[node.child_b];
                if (child_a.aabb.distance_to(point) < radius)
                {
                    stack.push_back(node.child_a);
                }
                if (child_b.aabb.distance_to(point) < radius)
                {
                    stack.push_back(node.child_b);
                }
            }
        }
    }

    std::vector<edge_data> radius_search(const Point point, const coord_t radius) const
    {
        std::vector<edge_data> result;
        radius_search(point, radius, result);
        return result;
    }
};
} // namespace cura

#endif // CURAENGINE_BOUNDEDAREAHIERARCHY_H
