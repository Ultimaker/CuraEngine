//Copyright (c) 2020 Ultimaker B.V.


#ifndef SKELETAL_TRAPEZOIDATION_GRAPH_H
#define SKELETAL_TRAPEZOIDATION_GRAPH_H

#include "utils/HalfEdgeGraph.h"
#include "SkeletalTrapezoidationEdge.h"
#include "SkeletalTrapezoidationJoint.h"

namespace arachne
{
    using namespace cura;

class SkeletalTrapezoidationGraph: public HalfEdgeGraph<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge>
{
    using edge_t = HalfEdge<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge>;
    using node_t = HalfEdgeNode<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge>;    
public:
    void fixNodeDuplication();
    
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
    
    
    void makeRib(edge_t*& prev_edge, Point start_source_point, Point end_source_point, bool is_next_to_start_or_end);
    
};

}
#endif 
