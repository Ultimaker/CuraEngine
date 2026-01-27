// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SKELETAL_TRAPEZOIDATION_HALF_EDGE_H
#define SKELETAL_TRAPEZOIDATION_HALF_EDGE_H

#include <optional>

#include "arachne/SkeletalTrapezoidationEdge.h"

namespace cura
{

class SkeletalTrapezoidationJoint;
class STHalfEdgeNode;

class STHalfEdge
{
    using node_data_t = SkeletalTrapezoidationJoint;
    using edge_data_t = SkeletalTrapezoidationEdge;
    using edge_t = STHalfEdge;
    using node_t = STHalfEdgeNode;

public:
    edge_data_t data_;
    edge_t* twin_ = nullptr;
    edge_t* next_ = nullptr;
    edge_t* prev_ = nullptr;
    node_t* from_ = nullptr;
    node_t* to_ = nullptr;

    STHalfEdge(const SkeletalTrapezoidationEdge& data);

    /*!
     * Check (recursively) whether there is any upward edge from the distance_to_boundary of the from of the \param edge
     *
     * \param strict Whether equidistant edges can count as a local maximum
     */
    bool canGoUp(bool strict = false) const;

    /*!
     * Check whether the edge goes from a lower to a higher distance_to_boundary.
     * Effectively deals with equidistant edges by looking beyond this edge.
     *
     * \param strict Whether equidistant edges can count as going upwards
     */
    bool isUpward(const bool strict = false) const;

    /*!
     * Calculate the traversed distance until we meet an upward edge.
     * Useful for calling on edges between equidistant points.
     *
     * If we can go up then the distance includes the length of the \param edge
     */
    std::optional<coord_t> distToGoUp() const;

    STHalfEdge* getNextUnconnected();
};

} // namespace cura
#endif
