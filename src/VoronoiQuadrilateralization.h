//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_H
#define VORONOI_QUADRILATERALIZATION_H

#include <boost/polygon/voronoi.hpp>

#include <unordered_map>

#include "utils/HalfEdgeGraph.h"
#include "utils/polygon.h"
#include "utils/PolygonsSegmentIndex.h"
#include "VoronoiQuadrilateralizationEdge.h"
#include "VoronoiQuadrilateralizationJoint.h"

namespace arachne
{

class VoronoiQuadrilateralization
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
    using graph_t = HalfEdgeGraph<VoronoiQuadrilateralizationJoint, VoronoiQuadrilateralizationEdge>;
    using edge_t = HalfEdge<VoronoiQuadrilateralizationJoint, VoronoiQuadrilateralizationEdge>;
    using node_t = HalfEdgeNode<VoronoiQuadrilateralizationJoint, VoronoiQuadrilateralizationEdge>;

    coord_t snap_dist = 10; // generic arithmatic inaccuracy
    coord_t rib_snap_distance = 100; // smallest segment cut off of an outline segment by an introduced rib. smaller than this the rib is canceled because it lies too close to an existing edge
public:
    using Segment = PolygonsSegmentIndex;
    VoronoiQuadrilateralization(const Polygons& polys);
    HalfEdgeGraph<VoronoiQuadrilateralizationJoint, VoronoiQuadrilateralizationEdge> graph;
protected:
    std::unordered_map<vd_t::edge_type*, edge_t*> vd_edge_to_he_edge;
    std::unordered_map<vd_t::vertex_type*, node_t*> vd_node_to_he_node;
    node_t& make_node(vd_t::vertex_type& vd_node, Point p);
    edge_t& make_edge(Point from, Point to, vd_t::edge_type& vd_edge);
    void debugCheckGraphCompleteness();
    void debugOutput(SVG& svg);
    SVG::Color getColor(edge_t& edge);
};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
