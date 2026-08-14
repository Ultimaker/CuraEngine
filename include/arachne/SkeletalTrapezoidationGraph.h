// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SKELETAL_TRAPEZOIDATION_GRAPH_H
#define SKELETAL_TRAPEZOIDATION_GRAPH_H

#include <list>

#include "arachne/STHalfEdge.h"
#include "arachne/STHalfEdgeNode.h"
#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

class SkeletalTrapezoidationGraph
{
    using edge_t = STHalfEdge;
    using node_t = STHalfEdgeNode;

public:
    std::list<edge_t> edges_;
    std::list<node_t> nodes_;

    /*!
     * If an edge is too small, collapse it and its twin and fix the surrounding edges to ensure a consistent graph.
     *
     * Don't collapse support edges, unless we can collapse the whole quad.
     *
     * o-,
     * |  "-o
     * |    | > Don't collapse this edge only.
     * o    o
     */
    void collapseSmallEdges(coord_t snap_dist = 5);

    void makeRib(edge_t*& prev_edge, Point2LL start_source_point, Point2LL end_source_point);

    /*!
     * Insert a node into the graph and connect it to the input polygon using ribs
     *
     * \return the last edge which replaced [edge], which points to the same [to] node
     */
    edge_t* insertNode(edge_t* edge, Point2LL mid, coord_t mide_node_bead_count);

    /*!
     * Return the first and last edge of the edges replacing \p edge pointing to the same node
     */
    std::pair<edge_t*, edge_t*> insertRib(edge_t& edge, node_t* mid_node);

private:
    std::pair<Point2LL, Point2LL> getSource(const edge_t& edge);
};

} // namespace cura
#endif
