//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_NODE_H
#define UTILS_HALF_EDGE_NODE_H

#include <list>

#include "IntPoint.h"

namespace arachne
{
    using namespace cura;

template<typename node_data_t, typename edge_data_t>
class HalfEdge;

template<typename node_data_t, typename edge_data_t>
class HalfEdgeNode
{
    using edge_t = HalfEdge<node_data_t, edge_data_t>;
    using node_t = HalfEdgeNode<node_data_t, edge_data_t>;
public:
    node_data_t data;
    Point p;
    edge_t* some_edge = nullptr;
    HalfEdgeNode(node_data_t data, Point p)
    : data(data)
    , p(p)
    {}
    
    bool operator==(const HalfEdgeNode& other)
    {
        return this == &other;
    }
    
    bool isMultiIntersection()
    {
        int odd_path_count = 0;
        bool first = true;
        for (edge_t* outgoing = this->some_edge; first || outgoing != this->some_edge; outgoing = outgoing->twin->next)
        {
            first = false;
            if (outgoing->data.isMarked())
            {
                odd_path_count++;
            }
        }
        return odd_path_count > 2;
    }
    
    bool isMarked() const
    {
        bool first = true;
        for (edge_t* edge = some_edge; first || edge != some_edge; edge = edge->twin->next)
        {
            if (edge->data.isMarked())
            {
                return true;
            }
            first = false;
            assert(edge->twin); if (!edge->twin) return false;
        }
        return false;
    }
    
    /*!
     * Check whether this node has a locally maximal distance_to_boundary
     * 
     * \param strict Whether equidistant edges can count as a local maximum
     */
    bool isLocalMaximum(bool strict = false) const
    {
        if (data.distance_to_boundary == 0)
        {
            return false;
        }
        
        bool first = true;
        for (edge_t* edge = some_edge; first || edge != some_edge; edge = edge->twin->next)
        {
            if (edge->canGoUp(strict))
            {
                return false;
            }
            first = false;
            assert(edge->twin); if (!edge->twin) return false;
            
            if (!edge->twin->next)
            { // This point is on the boundary
                return false;
            }
        }
        return true;
    }

};




} // namespace arachne
#endif // UTILS_HALF_EDGE_NODE_H
