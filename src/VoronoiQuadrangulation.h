//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_H
#define VORONOI_QUADRILATERALIZATION_H

#include <boost/polygon/voronoi.hpp>

#include <unordered_map>

#include "utils/HalfEdgeGraph.h"
#include "utils/polygon.h"
#include "utils/PolygonsSegmentIndex.h"
#include "VoronoiQuadrangulationEdge.h"
#include "VoronoiQuadrangulationJoint.h"

namespace arachne
{

class VoronoiQuadrangulation
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
    using graph_t = HalfEdgeGraph<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge>;
    using edge_t = HalfEdge<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge>;
    using node_t = HalfEdgeNode<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge>;

    coord_t snap_dist = 10; // generic arithmatic inaccuracy
    coord_t rib_snap_distance = 100; // smallest segment cut off of an outline segment by an introduced rib. smaller than this the rib is canceled because it lies too close to an existing edge
public:
    using Segment = PolygonsSegmentIndex;
    VoronoiQuadrangulation(const Polygons& polys);
    HalfEdgeGraph<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge> graph;
protected:
    std::unordered_map<vd_t::edge_type*, edge_t*> vd_edge_to_he_edge;
    std::unordered_map<vd_t::vertex_type*, node_t*> vd_node_to_he_node;
    node_t& make_node(vd_t::vertex_type& vd_node, Point p);
    edge_t& make_edge(Point from, Point to, vd_t::edge_type& vd_edge);
    edge_t* make_rib(edge_t* prev_edge, Point start_source_point, Point end_source_point, bool is_next_to_start_or_end);
    
    bool computePointCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments);
    void computeSegmentCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments);
    
    void debugCheckGraphCompleteness();
    void debugOutput(SVG& svg);
    SVG::Color getColor(edge_t& edge);
};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
