// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SKELETAL_TRAPEZOIDATION_EDGE_NODE_H
#define SKELETAL_TRAPEZOIDATION_EDGE_NODE_H

#include <optional>

#include "SkeletalTrapezoidationEdge.h"
#include "SkeletalTrapezoidationJoint.h"

namespace cura
{

class SkeletalTrapezoidationJoint;
class SkeletalTrapezoidationEdge;
class STHalfEdgeNode;
class STHalfEdge;

class STHalfEdgeNode
{
    using node_data_t = SkeletalTrapezoidationJoint;
    using edge_data_t = SkeletalTrapezoidationEdge;
    using edge_t = STHalfEdge;
    using node_t = STHalfEdgeNode;

public:
    node_data_t data_;
    Point2LL p_;
    edge_t* incident_edge_ = nullptr;

    STHalfEdgeNode(const SkeletalTrapezoidationJoint& data, const Point2LL& p);

    bool isMultiIntersection();

    bool isCentral() const;

    /*!
     * Check whether this node has a locally maximal distance_to_boundary
     *
     * \param strict Whether equidistant edges can count as a local maximum
     */
    bool isLocalMaximum(bool strict = false) const;
};

} // namespace cura
#endif
