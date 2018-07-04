//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MINIMUMSPANNINGTREE_H
#define MINIMUMSPANNINGTREE_H

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "IntPoint.h"

namespace cura
{

/*!
 * \brief Implements Prim's algorithm to compute Minimum Spanning Trees (MST).
 *
 * The minimum spanning tree is always computed from a clique of vertices.
 */
class MinimumSpanningTree
{
    /*!
     * \brief Represents an edge of the tree.
     *
     * While edges are meant to be undirected, these do have a start and end
     * point.
     */
    struct Edge {
        /**
         * Constructs a new edge.
         * @param start One endpoint of the edge.
         * @param end The other endpoint of the edge.
         */
        Edge(const Point start, const Point end);

        /**
         * The point at which this edge starts.
         */
        const Point start;

        /**
         * The point at which this edge ends.
         */
        const Point end;

        /**
         * Computes the length of this edge.
         * @return The length of this edge.
         */
        int length() const;
    };
public:
    /*!
     * \brief Constructs a minimum spanning tree that spans all given vertices.
     */
    MinimumSpanningTree(std::unordered_set<Point> vertices);

    /*!
     * \brief Gets the nodes that are adjacent to the specified node.
     * \return A list of nodes that are adjacent.
     */
    const std::vector<Point> adjacentNodes(Point node) const;

    /*!
     * \brief Gets the leaves of the tree.
     * \return A list of nodes that are all leaves of the tree.
     */
    std::vector<Point> leaves() const;

    /*!
     * \brief Gets all vertices of the tree.
     * \return A list of vertices of the tree.
     */
    std::vector<Point> vertices() const;

private:
    const std::unordered_map<Point, std::vector<Edge>> adjacency_graph;

    /*!
     * \brief Computes the edges of a minimum spanning tree using Prim's
     * algorithm.
     *
     * \param vertices The vertices to span.
     * \return An adjacency graph with for each point one or more edges.
     */
    const std::unordered_map<Point, std::vector<Edge>> prim(std::unordered_set<Point> vertices) const;
};

}

#endif /* MINIMUMSPANNINGTREE_H */

