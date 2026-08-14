// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "arachne/STHalfEdgeNode.h"

#include <unordered_map>

#include <spdlog/spdlog.h>

#include "arachne/STHalfEdge.h"
#include "arachne/SkeletalTrapezoidationGraph.h"
#include "utils/linearAlg2D.h"
#include "utils/macros.h"

namespace cura
{

STHalfEdgeNode::STHalfEdgeNode(const SkeletalTrapezoidationJoint& data, const Point2LL& p)
    : data_(data)
    , p_(p)
{
}

bool STHalfEdgeNode::isMultiIntersection()
{
    int odd_path_count = 0;
    edge_t* outgoing = incident_edge_;
    do
    {
        if (! outgoing)
        { // This is a node on the outside
            return false;
        }
        if (outgoing->data_.isCentral())
        {
            odd_path_count++;
        }
    } while (outgoing = outgoing->twin_->next_, outgoing != incident_edge_);
    return odd_path_count > 2;
}

bool STHalfEdgeNode::isCentral() const
{
    edge_t* edge = incident_edge_;
    do
    {
        if (edge->data_.isCentral())
        {
            return true;
        }
        assert(edge->twin_);
        if (! edge->twin_)
            return false;
    } while (edge = edge->twin_->next_, edge != incident_edge_);
    return false;
}

bool STHalfEdgeNode::isLocalMaximum(bool strict) const
{
    if (data_.distance_to_boundary_ == 0)
    {
        return false;
    }

    edge_t* edge = incident_edge_;
    do
    {
        if (edge->canGoUp(strict))
        {
            return false;
        }
        assert(edge->twin_);
        if (! edge->twin_)
            return false;

        if (! edge->twin_->next_)
        { // This point is on the boundary
            return false;
        }
    } while (edge = edge->twin_->next_, edge != incident_edge_);
    return true;
}

} // namespace cura
