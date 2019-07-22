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
#include "utils/ExtrusionJunction.h"
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
    coord_t discretization_step_size = 200;
    coord_t transition_filter_dist = 1000; // filter transitions closer together than this
    coord_t marking_filter_dist = 20; // filter marking areas smaller than this
    coord_t beading_propagation_transition_dist = 400;

public:
    using Segment = PolygonsSegmentIndex;
    VoronoiQuadrangulation(const Polygons& polys, float transitioning_angle);
    HalfEdgeGraph<VoronoiQuadrangulationJoint, VoronoiQuadrangulationEdge> graph;
    std::vector<ExtrusionSegment> generateToolpaths(const BeadingStrategy& beading_strategy);

protected:
    const Polygons& polys;
    float transitioning_angle;

    void init();

    /*!
     * mapping each voronoi VD edge to the corresponding halfedge HE edge
     * In case the result segment is discretized, we map the VD edge to the *last* HE edge
     */
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

    /*!
     * For VD cells associated with an input polygon vertex, we need to separate the node at the end and start of the cell into two
     * That way we can reach both the quad_start and the quad_end from the [some_edge] of the two new nodes
     * Otherwise if node.some_edge = quad_start you couldnt reach quad_end.twin by normal iteration (i.e. it = it.twin.next)
     */
    void separatePointyQuadEndNodes();
    void removeZeroLengthSegments();
    void fixNodeDuplication();

    // ^ init | v transitioning

    void setMarking(const BeadingStrategy& beading_strategy); //! set the is_marked flag for each edge

    void filterMarking(coord_t max_length); //! Filter out small marked areas

    /*!
     * Filter markings connected to starting_edge recursively.
     * 
     * \return Whether we should unmark this marked section
     */
    bool filterMarking(edge_t* starting_edge, coord_t traveled_dist, coord_t max_length);

    /*!
     * set bead count in marked regions
     */
    void setBeadCount(const BeadingStrategy& beading_strategy);

    /*!
     * Add marked regions and set bead counts
     * where there is an end of marking and when traveling upward we get to another region with the same bead count
     */
    void filterUnmarkedRegions(const BeadingStrategy& beading_strategy);

    /*!
     * 
     * \return Whether to set the bead count on the way back
     */
    bool filterUnmarkedRegions(edge_t* to_edge, coord_t bead_count, coord_t traveled_dist, coord_t max_dist, const BeadingStrategy& beading_strategy);

    struct TransitionMiddle
    {
        coord_t pos; //! position along edge as measure from edge.from.p
        coord_t lower_bead_count;
        TransitionMiddle(coord_t pos, coord_t lower_bead_count)
        : pos(pos), lower_bead_count(lower_bead_count)
        {}
    };

    struct TransitionMidRef
    {
        std::unordered_map<edge_t*, std::list<TransitionMiddle>>::iterator pair_it;
        std::list<TransitionMiddle>::iterator transition_it;
        TransitionMidRef(std::unordered_map<edge_t*, std::list<TransitionMiddle>>::iterator pair_it, std::list<TransitionMiddle>::iterator transition_it)
        : pair_it(pair_it)
        , transition_it(transition_it)
        {}
    };

    struct TransitionEnd
    {
        coord_t pos; //!< position along edge as measure from edge.from.p, where the edge is always the half edge oriented from lower to higher R
        coord_t lower_bead_count;
        bool is_lower_end; //!< whether this is the ed of the transition with lower bead count
        TransitionEnd(coord_t pos, coord_t lower_bead_count, bool is_lower_end)
        : pos(pos), lower_bead_count(lower_bead_count), is_lower_end(is_lower_end)
        {}
    };

    void generateTransitionMids(const BeadingStrategy& beading_strategy, std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions);

    void filterTransitionMids(std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions, const BeadingStrategy& beading_strategy);

    /*!
     * 
     * \param edge_to_start edge pointing to the node from which to start traveling in all directions except along \p edge_to_start
     * \param origin_transition The transition for which we are checking nearby transitions
     * \param traveled_dist the distance traveled before we came to \p edge_to_start.to
     * \param going_up Whether we are traveling in the upward direction as seen from the \p origin_transition. If this doesn't align with the direction according to the R diff on a consecutive edge we know there was a local optimum
     * \return whether the origin transition should be dissolved
     */
    std::list<TransitionMidRef> dissolveNearbyTransitions(edge_t* edge_to_start, TransitionMiddle& origin_transition, coord_t traveled_dist, coord_t max_dist, bool going_up, std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions, const BeadingStrategy& beading_strategy);

    void dissolveBeadCountRegion(edge_t* edge_to_start, coord_t from_bead_count, coord_t to_bead_count);

    bool filterEndOfMarkingTransition(edge_t* edge_to_start, coord_t traveled_dist, coord_t max_dist, coord_t replacing_bead_count, const BeadingStrategy& beading_strategy);

    void generateTransitionEnds(const BeadingStrategy& beading_strategy, std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions, std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends);

    /*!
     * Also set the rest values at nodes in between the transition ends
     */
    void applyTransitions(std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends);

    /*!
     * Insert a node into the graph and connect it to the input polygon using ribs
     * 
     * \return the last edge which replaced [edge], which points to the same [to] node
     */
    edge_t* insertNode(edge_t* edge, Point mid, coord_t mide_node_bead_count);

    void filterMarkedLocalOptima(const BeadingStrategy& beading_strategy);

    void generateTransitioningRibs(const BeadingStrategy& beading_strategy);

    /*!
     * 
     */
    void generateTransition(edge_t& edge, coord_t mid_R, const BeadingStrategy& beading_strategy, coord_t transition_lower_bead_count, std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends);

    /*!
     * \p start_rest and \p end_rest refer to gap distances at the start and end pos in terms of ratios w.r.t. the inner bead width at the high end of the transition
     * 
     * \p end_pos_along_edge may be beyond this edge!
     * In this case we need to interpolate the rest value at the locations in between
     * 
     * \return whether the subgraph is going downward
     */
    bool generateTransitionEnd(edge_t& edge, coord_t start_pos, coord_t end_pos, coord_t transition_half_length, float start_rest, float end_rest, coord_t transition_lower_bead_count, std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends);

    bool isGoingDown(edge_t* outgoing, coord_t traveled_dist, coord_t transition_half_length, coord_t lower_bead_count) const;

    /*!
     * Return the first and last edge of the edges replacing \p edge pointing to the same node
     */
    std::pair<VoronoiQuadrangulation::edge_t*, VoronoiQuadrangulation::edge_t*> insertRib(edge_t& edge, node_t* mid_node);

    std::pair<Point, Point> getSource(const edge_t& edge);
    bool isEndOfMarking(const edge_t& edge) const;

    bool isLocalMaximum(const node_t& node) const;

    bool isMarked(const node_t* node) const;

    // ^ transitioning | v toolpath generation



    /*!
     * \param[out] segments the generated segments
     */
    void generateSegments(std::vector<ExtrusionSegment>& segments, const BeadingStrategy& beading_strategy);

    edge_t* getQuadMaxRedgeTo(edge_t* quad_start_edge);

    struct BeadingPropagation
    {
        Beading beading;
        coord_t dist_to_bottom_source;
        coord_t dist_from_top_source;
        bool is_finished;
        BeadingPropagation(const Beading& beading)
        : beading(beading)
        , dist_to_bottom_source(0)
        , dist_from_top_source(0)
        , is_finished(false)
        {}
    };

    /*!
     * propagate beading info from lower R nodes to higher R nodes
     * 
     * only propagate from nodes with beading info upward to nodes without beading info
     * 
     * edges are sorted so that we can do a depth-first walk without employing a recursive algorithm
     * 
     * In upward propagated beadings we store the distance traveled, so that we can merge these beadings with the downward propagated beadings in \ref propagateBeadingsDownward(.)
     * 
     * \param upward_quad_mids all upward halfedges of the inner skeletal edges (not directly connected to the outline) sorted on their highest [distance_to_boundary]. Higher dist first.
     */
    void propagateBeadingsUpward(std::vector<edge_t*>& upward_quad_mids, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy);

    /*!
     * propagate beading info from higher R nodes to lower R nodes
     * 
     * merge with upward propagated beadings if they are encountered
     * 
     * don't transfer to nodes which lie on the outline polygon
     * 
     * edges are sorted so that we can do a depth-first walk without employing a recursive algorithm
     * 
     * \param upward_quad_mids all upward halfedges of the inner skeletal edges (not directly connected to the outline) sorted on their highest [distance_to_boundary]. Higher dist first.
     */
    void propagateBeadingsDownward(std::vector<edge_t*>& upward_quad_mids, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy);
    void propagateBeadingsDownward(edge_t* edge_to_peak, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy);

    /*!
     * \param switching_radius The radius at which we switch from the left beading to the merged
     */
    Beading interpolate(const Beading& left, float ratio_left_to_whole, const Beading& right, coord_t switching_radius) const;
    Beading interpolate(const Beading& left, float ratio_left_to_whole, const Beading& right) const;

    BeadingPropagation& getBeading(node_t* node, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy);

    /*!
     * generate junctions for each bone
     * \param edge_to_junctions junctions ordered high R to low R
     */
    void generateJunctions(std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, std::unordered_map<edge_t*, std::vector<ExtrusionJunction>>& edge_to_junctions, const BeadingStrategy& beading_strategy);

    /*!
     * connect junctions in each quad
     * \param edge_to_junctions junctions ordered high R to low R
     * \param[out] segments the generated segments
     */
    void connectJunctions(std::unordered_map<edge_t*, std::vector<ExtrusionJunction>> edge_to_junctions, std::vector<ExtrusionSegment>& segments);

    /*!
     * Genrate small segments for local maxima where the beading would only result in a single bead
     * \param[out] segments the generated segments
     */
    void generateLocalMaximaSingleBeads(std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, std::vector<ExtrusionSegment>& segments);

    /*!
     * \p edge is assumed to point upward to higher R; otherwise take its twin
     * 
     * \param include_odd_start_junction Whether to leave out the first junction if it coincides with \p edge.from->p
     */
    const std::vector<ExtrusionJunction>& getJunctions(edge_t* edge, std::unordered_map<edge_t*, std::vector<ExtrusionJunction>>& edge_to_junctions);
    
    // ^ toolpath generation | v helpers

public:
    void debugCheckGraphCompleteness(); //!< Checks whether all member fields of edges and nodes are filled. Should be true after initialization.
    void debugCheckEndpointUniqueness(); //!< Checks whether the end points of qauds have unique verts. Should be true after separatePointyQuadEndNodes().
    void debugCheckGraphExistance(); //!< Checks whether all member fields of edges and nodes are existing nodes/edges recorded in graph.nodes and graph.edges. Should be true after any graph update.
    void debugCheckGraphStructure(); //!< Checks whether iterating around a node (using it = it.twin.next) ends up where it started. Should be true after init.
    void debugCheckGraphReachability(); //!< Checks whether an edge is reachable from iterating around its from node. Should be true after init.
    void debugCheckGraphConsistency(bool ignore_duplication = false); //!< Checks whether edge and node relations fit with each other. Should be true after any graph update.
    void debugCheckDecorationConsistency(bool transitioned); //!< Check logical relationships relting to distance_to_boundary and is_marked etc. Should be true anywhere after setMarking(.)
    void debugCheckTransitionMids(const std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions) const;
    void debugOutput(SVG& svg, bool draw_arrows, bool draw_dists, bool draw_bead_counts = false, bool draw_locations = false);
protected:
    SVG::Color getColor(edge_t& edge);

};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
