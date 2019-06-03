//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_JOINT_H
#define VORONOI_QUADRILATERALIZATION_JOINT_H

#include "utils/IntPoint.h"

namespace arachne
{

class VoronoiQuadrangulationJoint
{
public:
    coord_t distance_to_boundary;
    coord_t bead_count;
    float transition_rest; //! The distance near the skeleton to leave free because this joint is in the middle of a transition, as a fraction of the inner bead width of the bead at the higher transition.
    VoronoiQuadrangulationJoint()
    : distance_to_boundary(-1)
    , bead_count(-1)
    , transition_rest(0)
    {}
};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_JOINT_H
