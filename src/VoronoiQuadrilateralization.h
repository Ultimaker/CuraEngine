//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_H
#define VORONOI_QUADRILATERALIZATION_H

#include <boost/polygon/voronoi.hpp>

#include "utils/HalfEdgeGraph.h"
#include "utils/polygon.h"
#include "utils/PolygonsPointIndex.h"
#include "VoronoiQuadrilateralizationEdge.h"
#include "VoronoiQuadrilateralizationJoint.h"

namespace arachne
{

class VoronoiQuadrilateralization
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
public:
    using Segment = PolygonsPointIndex;
    VoronoiQuadrilateralization(const Polygons& polys);
    HalfEdgeGraph<VoronoiQuadrilateralizationJoint, VoronoiQuadrilateralizationEdge> graph;
};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
