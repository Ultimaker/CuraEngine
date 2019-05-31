//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_H
#define VORONOI_QUADRILATERALIZATION_H

#include <boost/polygon/voronoi.hpp>

#include <unordered_map>
#include <utility> // pair

#include "utils/HalfEdgeGraph.h"
#include "utils/polygon.h"
#include "utils/PolygonsSegmentIndex.h"
#include "utils/ExtrusionSegment.h"
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
    using Beading = BeadingStrategy::Beading;

    coord_t snap_dist = 20; // generic arithmatic inaccuracy
    coord_t rib_snap_distance = 100; // smallest segment cut off of an outline segment by an introduced rib. smaller than this the rib is canceled because it lies too close to an existing edge
    coord_t discretization_step_size = 400;
public:
    using Segment = PolygonsSegmentIndex;
    VoronoiQuadrangulation(const Polygons& polys);
    HalfEdgeGraph<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge> graph;
    Polygons generateToolpaths(const BeadingStrategy& beading_strategy);
    const Polygons& polys;
protected:

    void init();

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

    void filterMarkedLocalOptima(const BeadingStrategy& beading_strategy);
    /*!
     * \param edge_to edge pointing to the node to check
     */
    bool isMarkedLocalOptimum(edge_t* edge_to);

    /*!
     * check whether a marked region around a given node with a constant optimal bead count is too small for a transition_lower_bead_count
     */
    bool isSmallMarkedLocalOptimum(edge_t* edge_to, const BeadingStrategy& beading_strategy);

    /*!
     * Get the maximum distance traveled outward along which the optimum bead count has the same value
     * 
     * \param stop_distance Stop computing if the return value would be larger than this
     * \param traveled_distance already traveled distance we did to get to \p outgoing.from
     */
    coord_t getLocalOptimumSize(edge_t* outgoing, coord_t bead_count, coord_t stop_distance, coord_t traveled_distance, const BeadingStrategy& beading_strategy);

    void dissolveMarkedLocalOptimum(edge_t* edge_to);

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

    // ^ transitioning | v toolpath generation

    /*!
     * \param segments maps segment start to segment end
     */
    void generateSegments(std::unordered_map<Point, ExtrusionSegment>& segments, const BeadingStrategy& beading_strategy);

    coord_t getQuadMaxR(edge_t* quad_start_edge);
    edge_t* getQuadMaxRedgeTo(edge_t* quad_start_edge);

    struct Junction
    {
        Point p;
        coord_t w;
        Junction(Point p, coord_t w)
        : p(p), w(w) {}
    };

    /*!
     * \p edge is assumed to point upward to higher R; otherwise take its twin
     * 
     * \param include_odd_start_junction Whether to leave out the first junction if it coincides with \p edge.from->p
     */
    const std::vector<Junction>& getJunctions(edge_t* edge, std::unordered_map<node_t*, BeadingStrategy::Beading>& node_to_beading, std::unordered_map<edge_t*, std::vector<Junction>>& edge_to_junctions, const BeadingStrategy& beading_strategy);

    // ^ toolpath generation | v helpers

public:
    void debugCheckGraphCompleteness();
    void debugCheckGraphConsistency();
    void debugCheckDecorationConsistency();
    void debugOutput(SVG& svg, bool draw_arrows, bool draw_dists, bool draw_bead_counts = false, bool draw_locations = false);
protected:
    SVG::Color getColor(edge_t& edge);

};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
