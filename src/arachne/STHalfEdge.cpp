// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "arachne/STHalfEdge.h"

#include <spdlog/spdlog.h>

#include "arachne/STHalfEdgeNode.h"
#include "utils/linearAlg2D.h"

namespace cura
{

STHalfEdge::STHalfEdge(const SkeletalTrapezoidationEdge& data)
    : data_(data)
{
}

bool STHalfEdge::canGoUp(bool strict) const
{
    if (to_->data_.distance_to_boundary_ > from_->data_.distance_to_boundary_)
    {
        return true;
    }
    if (to_->data_.distance_to_boundary_ < from_->data_.distance_to_boundary_ || strict)
    {
        return false;
    }

    // Edge is between equidistqant verts; recurse!
    for (edge_t* outgoing = next_; outgoing != twin_; outgoing = outgoing->twin_->next_)
    {
        if (outgoing->canGoUp())
        {
            return true;
        }
        assert(outgoing->twin_);
        if (! outgoing->twin_)
            return false;
        assert(outgoing->twin_->next_);
        if (! outgoing->twin_->next_)
            return true; // This point is on the boundary?! Should never occur
    }
    return false;
}

bool STHalfEdge::isUpward(const bool strict) const
{
    if (to_->data_.distance_to_boundary_ > from_->data_.distance_to_boundary_)
    {
        return true;
    }
    if (to_->data_.distance_to_boundary_ < from_->data_.distance_to_boundary_ || strict)
    {
        return false;
    }

    // Equidistant edge case:
    std::optional<coord_t> forward_up_dist = this->distToGoUp();
    std::optional<coord_t> backward_up_dist = twin_->distToGoUp();
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
    return to_->p_ < from_->p_; // Arbitrary ordering, which returns the opposite for the twin edge
}

std::optional<coord_t> STHalfEdge::distToGoUp() const
{
    if (to_->data_.distance_to_boundary_ > from_->data_.distance_to_boundary_)
    {
        return 0;
    }
    if (to_->data_.distance_to_boundary_ < from_->data_.distance_to_boundary_)
    {
        return std::optional<coord_t>();
    }

    // Edge is between equidistqant verts; recurse!
    std::optional<coord_t> ret;
    for (edge_t* outgoing = next_; outgoing != twin_; outgoing = outgoing->twin_->next_)
    {
        std::optional<coord_t> dist_to_up = outgoing->distToGoUp();
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
        assert(outgoing->twin_);
        if (! outgoing->twin_)
            return std::optional<coord_t>();
        assert(outgoing->twin_->next_);
        if (! outgoing->twin_->next_)
            return 0; // This point is on the boundary?! Should never occur
    }
    if (ret)
    {
        ret = *ret + vSize(to_->p_ - from_->p_);
    }
    return ret;
}

STHalfEdge* STHalfEdge::getNextUnconnected()
{
    edge_t* result = static_cast<STHalfEdge*>(this);
    while (result->next_)
    {
        result = result->next_;
        if (result == this)
        {
            return nullptr;
        }
    }
    return result->twin_;
}

} // namespace cura
