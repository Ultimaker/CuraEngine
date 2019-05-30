//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_H
#define VORONOI_QUADRILATERALIZATION_H

#include <boost/polygon/voronoi.hpp>

#include <unordered_map>
#include <utility> // pair

#include "utils/HalfEdgeGraph.h"
#include "utils/polygon.h"
#include "utils/PolygonsSegmentIndex.h"
#include "VoronoiQuadrangulationEdge.h"
#include "VoronoiQuadrangulationJoint.h"
#include "BeadingStrategy.h"

namespace arachne
{

class VoronoiQuadrangulation
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
    using graph_t = HalfEdgeGraph<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge>;
    using edge_t = HalfEdge<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge>;
    using node_t = HalfEdgeNode<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge>;

    coord_t snap_dist = 20; // generic arithmatic inaccuracy
    coord_t rib_snap_distance = 100; // smallest segment cut off of an outline segment by an introduced rib. smaller than this the rib is canceled because it lies too close to an existing edge
    coord_t discretization_step_size = 400;
public:
    using Segment = PolygonsSegmentIndex;
    VoronoiQuadrangulation(const Polygons& polys);
    HalfEdgeGraph<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge> graph;
    Polygons generateToolpaths(const BeadingStrategy& beading_strategy);
protected:

    void init(const Polygons& polys);

    std::unordered_map<vd_t::edge_type*, edge_t*> vd_edge_to_he_edge;
    std::unordered_map<vd_t::vertex_type*, node_t*> vd_node_to_he_node;
    node_t& make_node(vd_t::vertex_type& vd_node, Point p);
    /*!
     * \p prev_edge serves as input and output. May be null as input.
     */
    void transfer_edge(Point from, Point to, vd_t::edge_type& vd_edge, edge_t*& prev_edge, Point& start_source_point, Point& end_source_point, const std::vector<Point>& points, const std::vector<Segment>& segments);
    void make_rib(edge_t*& prev_edge, Point start_source_point, Point end_source_point, bool is_next_to_start_or_end);
    std::vector<Point> discretize(const vd_t::edge_type& segment, const std::vector<Point>& points, const std::vector<Segment>& segments);
    
    bool computePointCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments);
    void computeSegmentCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments);

    // ^ init | v transitioning

    void setMarking(); //! set the is_marked flag for each edge
    void generateTransitioningRibs(const BeadingStrategy& beading_strategy);

    /*!
     * Return the last edge of the edges replacing \p edge pointing to the same node
     */
    edge_t* generateTransition(edge_t& edge, coord_t mid_R, const BeadingStrategy& beading_strategy, coord_t transition_lower_bead_count);

    /*!
     * \p start_rest and \p end_rest refer to gap distances at the start adn end pos.
     * 
     * \p end_pos_along_edge may be beyond this edge!
     * In this case we need to interpolate the rest value at the locations in between
     * 
     * Return the last edge of the edges replacing \p edge pointing to the same node
     */
    edge_t* generateTransitionEnd(edge_t& edge, coord_t start_pos, coord_t end_pos, coord_t start_rest, coord_t end_rest, coord_t transition_lower_bead_count);

    /*!
     * Return the last edge of the edges replacing \p edge pointing to the same node
     */
    VoronoiQuadrangulation::edge_t* insertRib(edge_t& edge, node_t* mid_node);

    std::pair<Point, Point> getSource(const edge_t& edge);
    bool isEndOfMarking(const edge_t& edge);

    // ^ transitioning | v helpers

public:
    void debugCheckGraphCompleteness();
    void debugCheckGraphConsistency();
    void debugOutput(SVG& svg, bool draw_arrows, bool draw_dists, bool draw_bead_counts = false);
protected:
    SVG::Color getColor(edge_t& edge);

};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
