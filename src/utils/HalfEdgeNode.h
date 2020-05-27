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
    edge_t* incident_edge = nullptr;
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
        edge_t* outgoing = this->incident_edge;
        do
        {
            if (outgoing->data.isMarked())
            {
                odd_path_count++;
            }
        }
        while(outgoing = outgoing->twin->next, outgoing != this->incident_edge);
        return odd_path_count > 2;
    }
    
    bool isMarked() const
    {
        edge_t* edge = incident_edge;
        do
        {
            if (edge->data.isMarked())
            {
                return true;
            }
            assert(edge->twin); if (!edge->twin) return false;
        }
        while(edge = edge->twin->next, edge != incident_edge);
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
        
        edge_t* edge = incident_edge;
        do
        {
            if (edge->canGoUp(strict))
            {
                return false;
            }
            assert(edge->twin); if (!edge->twin) return false;
            
            if (!edge->twin->next)
            { // This point is on the boundary
                return false;
            }
        }
        while (edge = edge->twin->next, edge != incident_edge);
        return true;
    }

};




} // namespace arachne
#endif // UTILS_HALF_EDGE_NODE_H
