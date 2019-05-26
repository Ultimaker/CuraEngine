//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_JOINT_H
#define VORONOI_QUADRILATERALIZATION_JOINT_H

#include "utils/IntPoint.h"

namespace arachne
{

class VoronoiQuadrilateralizationJoint
{
public:
    coord_t distance_to_boundary;
    float bead_count;
    VoronoiQuadrilateralizationJoint()
    {}
};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_JOINT_H
