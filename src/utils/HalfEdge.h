//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_HALF_EDGE_H
#define UTILS_HALF_EDGE_H

#include <forward_list>
#include "../utils/optional.h"  // until the move to C++17
#include "../utils/IntPoint.h"
#include "Coord_t.h"

namespace arachne
{

template<typename node_data_t, typename edge_data_t>
class HalfEdgeNode;


template<typename node_data_t, typename edge_data_t>
class HalfEdge
{
    using edge_t = HalfEdge<node_data_t, edge_data_t>;
    using node_t = HalfEdgeNode<node_data_t, edge_data_t>;
public:
    edge_data_t data;
    edge_t* twin = nullptr;
    edge_t* next = nullptr;
    edge_t* prev = nullptr;
    node_t* from = nullptr;
    node_t* to = nullptr;
    HalfEdge(edge_data_t data)
    : data(data)
    {}
    bool operator==(const HalfEdge& other)
    {
        return this == &other;
    }
    
    /*!
     * Check (recursively) whether there is any upward edge from the distance_to_boundary of the from of the \param edge
     * 
     * \param strict Whether equidistant edges can count as a local maximum
     */
    bool canGoUp(bool strict = false) const
    {
        if (to->data.distance_to_boundary > from->data.distance_to_boundary)
        {
            return true;
        }
        if (to->data.distance_to_boundary < from->data.distance_to_boundary || strict)
        {
            return false;
        }
        
        // Edge is between equidistqant verts; recurse!
        for (edge_t* outgoing = next; outgoing != twin; outgoing = outgoing->twin->next)
        {
            if (outgoing->canGoUp())
            {
                return true;
            }
            assert(outgoing->twin); if (!outgoing->twin) return false;
            assert(outgoing->twin->next); if (!outgoing->twin->next) return true; // This point is on the boundary?! Should never occur
        }
        return false;
    }
    
    /*!
     * Check whether the edge goes from a lower to a higher distance_to_boundary.
     * Effectively deals with equidistant edges by looking beyond this edge.
     */
    bool isUpward() const
    {
        if (to->data.distance_to_boundary > from->data.distance_to_boundary)
        {
            return true;
        }
        if (to->data.distance_to_boundary < from->data.distance_to_boundary)
        {
            return false;
        }
        
        // Equidistant edge case:
        std::optional<cura::coord_t> forward_up_dist = this->distToGoUp();
        std::optional<cura::coord_t> backward_up_dist = twin->distToGoUp();
        if (forward_up_dist && backward_up_dist)
        {
            return forward_up_dist < backward_up_dist;
        }
        
        if (forward_up_dist) 
        {
            return true;
        }
        
        if (backward_up_dist)
        {
            return false;
        }
        return to->p < from->p; // Arbitrary ordering, which returns the opposite for the twin edge
    }
    
    
    /*!
     * Calculate the traversed distance until we meet an upward edge.
     * Useful for calling on edges between equidistant points.
     * 
     * If we can go up then the distance includes the length of the \param edge
     */
    std::optional<cura::coord_t> distToGoUp() const
    {
        if (to->data.distance_to_boundary > from->data.distance_to_boundary)
        {
            return 0;
        }
        if (to->data.distance_to_boundary < from->data.distance_to_boundary)
        {
            return std::optional<cura::coord_t>();
        }
        
        // Edge is between equidistqant verts; recurse!
        std::optional<cura::coord_t> ret;
        for (edge_t* outgoing = next; outgoing != twin; outgoing = outgoing->twin->next)
        {
            std::optional<cura::coord_t> dist_to_up = outgoing->distToGoUp();
            if (dist_to_up)
            {
                if (ret)
                {
                    ret = std::min(*ret, *dist_to_up);
                }
                else
                {
                    ret = dist_to_up;
                }
            }
            assert(outgoing->twin); if (!outgoing->twin) return std::optional<cura::coord_t>();
            assert(outgoing->twin->next); if (!outgoing->twin->next) return 0; // This point is on the boundary?! Should never occur
        }
        if (ret)
        {
            ret =  *ret + cura::vSize(to->p - from->p);
        }
        return ret;
    }
};




} // namespace arachne
#endif // UTILS_HALF_EDGE_H
