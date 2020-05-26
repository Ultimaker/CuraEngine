//Copyright (c) 2020 Ultimaker B.V.


#ifndef SKELETAL_TRAPEZOIDATION_H
#define SKELETAL_TRAPEZOIDATION_H

#include <boost/polygon/voronoi.hpp>

#include <memory> // smart pointers
#include <unordered_map>
#include <utility> // pair

#include "utils/HalfEdgeGraph.h"
#include "utils/polygon.h"
#include "utils/PolygonsSegmentIndex.h"
#include "utils/ExtrusionJunction.h"
#include "utils/ExtrusionLine.h"
#include "SkeletalTrapezoidationEdge.h"
#include "SkeletalTrapezoidationJoint.h"
#include "BeadingStrategy/BeadingStrategy.h"
#include "SkeletalTrapezoidationGraph.h"


namespace arachne
{
    using namespace cura;

/*!
 * Main class of the dynamic beading strategies.
 * 
 * The input polygon region is decomposed into trapezoids and represented as a half-edge data-structure.
 * 
 * We determine which edges are 'central' accordinding to the transitioning_angle of the beading strategy,
 * and determine the bead count for these central regions and apply them outward when generating toolpaths. [oversimplified]
 * 
 * The method can be visually explained as generating the 3D union of cones surface on the outline polygons,
 * and changing the heights along central regions of that surface so that they are flat. 
 * For more info, please consult the paper "A framework for adaptive width control of dense contour-parallel toolpaths in fused
deposition modeling" by Kuipers et al. 
 * This visual explanation aid explains the use of "upward", "lower" etc,
 * i.e. the radial distance and/or the bead count are used as heights of this visualization, there is no coordinate called 'Z'.
 * 
 * TODO: split this class into two:
 * 1. Class for generating the decomposition and aux functions for performing updates
 * 2. Class for editing the structure for our purposes.
 */
class SkeletalTrapezoidation
{
    using pos_t = double;
    using vd_t = boost::polygon::voronoi_diagram<pos_t>;
    using graph_t = SkeletalTrapezoidationGraph;
    using edge_t = HalfEdge<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge>;
    using node_t = HalfEdgeNode<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge>;
    using Beading = BeadingStrategy::Beading;
    using BeadingPropagation = SkeletalTrapezoidationJoint::BeadingPropagation;
    using TransitionMiddle = SkeletalTrapezoidationEdge::TransitionMiddle;
    using TransitionEnd = SkeletalTrapezoidationEdge::TransitionEnd;

    template<typename T>
    using ptr_vector_t = std::vector<std::shared_ptr<T>>;

    AngleRadians transitioning_angle; //!< How pointy a region should be before we apply the method. Equals 180* - limit_bisector_angle
    coord_t discretization_step_size; //!< approximate size of segments when parabolic VD edges get discretized (and vertex-vertex edges)
    coord_t transition_filter_dist; //!< Filter transition mids (i.e. anchors) closer together than this
    coord_t beading_propagation_transition_dist; //!< When there are different beadings propagated from below and from above, use this transitioning distance
    static constexpr coord_t marking_filter_dist = 20; //!< Filter areas marked as 'central' smaller than this
    static constexpr coord_t snap_dist = 20; //!< Generic arithmatic inaccuracy. Only used to determine whether a transition really needs to insert an extra edge.

    /*!
     * The strategy to use to fill a certain shape with lines.
     *
     * Various BeadingStrategies are available that differ in which lines get to
     * print at their optimal width, where the play is being compensated, and
     * how the joints are handled where we transition to different numbers of
     * lines.
     */
    const BeadingStrategy& beading_strategy;

public:
    using Segment = PolygonsSegmentIndex;

    /*!
     * Construct a new trapezoidation problem to solve.
     * \param polys The shapes to fill with walls.
     * \param beading_strategy The strategy to use to fill these shapes.
     * \param transitioning_angle Where we transition to a different number of
     * walls, how steep should this transition be? A lower angle means that the
     * transition will be longer.
     * \param discretization_step_size Since g-code can't represent smooth
     * transitions in line width, the line width must change with discretized
     * steps. This indicates how long the line segments between those steps will
     * be.
     * \param transition_filter_dist The minimum length of transitions.
     * Transitions shorter than this will be considered for dissolution.
     * \param beading_propagation_transition_dist When there are different
     * beadings propagated from below and from above, use this transitioning
     * distance.
     */
    SkeletalTrapezoidation(const Polygons& polys, 
                           const BeadingStrategy& beading_strategy,
                           AngleRadians transitioning_angle
    , coord_t discretization_step_size = 200
    , coord_t transition_filter_dist = 1000
    , coord_t beading_propagation_transition_dist = 400);

    /*!
     * A skeletal graph through the polygons that we need to fill with beads.
     *
     * The skeletal graph represents the medial axes through each part of the
     * polygons, and the lines from these medial axes towards each vertex of the
     * polygons. The graph can be used to see what the width is of a polygon in
     * each place and where the width transitions.
     */
    graph_t graph;

    /*!
     * Generate the paths that the printer must extrude, to print the outlines
     * in the input polygons.
     * \param filter_outermost_marked_edges Some edges are "central" but still
     * touch the outside of the polygon. If enabled, don't treat these as
     * "central" but as if it's a obtuse corner. As a result, sharp corners will
     * no longer end in a single line but will just loop.
     */
    std::vector<std::list<ExtrusionLine>> generateToolpaths(bool filter_outermost_marked_edges = false);

protected:
    /*!
     * Auxiliary for referencing one transition along an edge which may contain multiple transitions
     */
    struct TransitionMidRef
    {
        edge_t* edge;
        std::list<TransitionMiddle>::iterator transition_it;
        TransitionMidRef(edge_t* edge, std::list<TransitionMiddle>::iterator transition_it)
            : edge(edge)
            , transition_it(transition_it)
        {}
    };

    /*!
     * Compute the skeletal trapezoidation decomposition of the input shape.
     * 
     * Compute the Voronoi Diagram (VD) and transfer all inside edges into our half-edge (HE) datastructure.
     * 
     * The algorithm is currently a bit overcomplicated, because the discretization of parabolic edges is performed at the same time as all edges are being transfered,
     * which means that there is no one-to-one mapping from VD edges to HE edges.
     * Instead we map from a VD edge to the last HE edge.
     * This could be cimplified by recording the edges which should be discretized and discretizing the mafterwards.
     * 
     * Another complication arises because the VD uses floating logic, which can result in zero-length segments after rounding to integers.
     * We therefore collapse edges and their whole cells afterwards.
     */
    void constructFromPolygons(const Polygons& polys);

    /*!
     * mapping each voronoi VD edge to the corresponding halfedge HE edge
     * In case the result segment is discretized, we map the VD edge to the *last* HE edge
     */
    std::unordered_map<vd_t::edge_type*, edge_t*> vd_edge_to_he_edge;
    std::unordered_map<vd_t::vertex_type*, node_t*> vd_node_to_he_node;
    node_t& makeNode(vd_t::vertex_type& vd_node, Point p); //!< Get the node which the VD node maps to, or create a new mapping if there wasn't any yet.
    
    /*!
     * Transfer an edge from the VD to the HE and perform discretization of parabolic edges (and vertex-vertex edges)
     * \p prev_edge serves as input and output. May be null as input.
     */
    void transferEdge(Point from, Point to, vd_t::edge_type& vd_edge, edge_t*& prev_edge, Point& start_source_point, Point& end_source_point, const std::vector<Point>& points, const std::vector<Segment>& segments);

    /*!
     * Discretize a Voronoi edge that represents the medial axis of a vertex-
     * line region or vertex-vertex region into small segments that can be
     * considered to have a straight medial axis and a linear line width
     * transition.
     *
     * The medial axis between a point and a line is a parabola. The rest of the
     * algorithm doesn't want to have to deal with parabola, so this discretises
     * the parabola into straight line segments. This is necessary if there is a
     * sharp inner corner (acts as a point) that comes close to a straight edge.
     *
     * The medial axis between a point and a point is a straight line segment.
     * However the distance from the medial axis to either of those points draws
     * a parabola as you go along the medial axis. That means that the resulting
     * line width along the medial axis would not be linearly increasing or
     * linearly decreasing, but needs to take the shape of a parabola. Instead,
     * we'll break this edge up into tiny line segments that can approximate the
     * parabola with tiny linear increases or decreases in line width.
     * \param segment The variable-width Voronoi edge to discretize.
     * \param points All vertices of the original Polygons to fill with beads.
     * \param segments All line segments of the original Polygons to fill with
     * beads.
     * \return A number of coordinates along the edge where the edge is broken
     * up into discrete pieces.
     */
    std::vector<Point> discretize(const vd_t::edge_type& segment, const std::vector<Point>& points, const std::vector<Segment>& segments);

    /*!
     * Compute the range of line segments that surround a cell of the skeletal
     * graph that belongs to a point on the medial axis.
     *
     * This should only be used on cells that belong to a corner in the skeletal
     * graph, e.g. triangular cells, not trapezoid cells.
     *
     * The resulting line segments is just the first and the last segment. They
     * are linked to the neighboring segments, so you can iterate over the
     * segments until you reach the last segment.
     * \param cell The cell to compute the range of line segments for.
     * \param[out] start_source_point The start point of the source segment of
     * this cell.
     * \param[out] end_source_point The end point of the source segment of this
     * cell.
     * \param[out] starting_vd_edge The edge of the Voronoi diagram where the
     * loop around the cell starts.
     * \param[out] ending_vd_edge The edge of the Voronoi diagram where the loop
     * around the cell ends.
     * \param points All vertices of the input Polygons.
     * \param segments All edges of the input Polygons.
     * /return Whether the cell is inside of the polygon. If it's outside of the
     * polygon we should skip processing it altogether.
     */
    bool computePointCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments);

    /*!
     * Compute the range of line segments that surround a cell of the skeletal
     * graph that belongs to a line segment of the medial axis.
     *
     * This should only be used on cells that belong to a central line segment
     * of the skeletal graph, e.g. trapezoid cells, not triangular cells.
     *
     * The resulting line segments is just the first and the last segment. They
     * are linked to the neighboring segments, so you can iterate over the
     * segments until you reach the last segment.
     * \param cell The cell to compute the range of line segments for.
     * \param[out] start_source_point The start point of the source segment of
     * this cell.
     * \param[out] end_source_point The end point of the source segment of this
     * cell.
     * \param[out] starting_vd_edge The edge of the Voronoi diagram where the
     * loop around the cell starts.
     * \param[out] ending_vd_edge The edge of the Voronoi diagram where the loop
     * around the cell ends.
     * \param points All vertices of the input Polygons.
     * \param segments All edges of the input Polygons.
     * /return Whether the cell is inside of the polygon. If it's outside of the
     * polygon we should skip processing it altogether.
     */
    void computeSegmentCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments);

    /*!
     * For VD cells associated with an input polygon vertex, we need to separate the node at the end and start of the cell into two
     * That way we can reach both the quad_start and the quad_end from the [some_edge] of the two new nodes
     * Otherwise if node.some_edge = quad_start you couldnt reach quad_end.twin by normal iteration (i.e. it = it.twin.next)
     */
    void separatePointyQuadEndNodes();


    // ^ init | v transitioning

    void updateMarking(); // Update the "is_marked" flag for each edge based on the transitioning_angle

    /*!
     * Filter out small marked areas.
     * 
     * Only used to get rid of small edges which get marked because of rounding errors because hte region is so small.
     */
    void filterMarking(coord_t max_length);

    /*!
     * Filter markings connected to starting_edge recursively.
     * 
     * \return Whether we should unmark this marked section on the way back out of the recursion
     */
    bool filterMarking(edge_t* starting_edge, coord_t traveled_dist, coord_t max_length);

    /*!
     * Unmark the outermost edges directly connected to the outline.
     * 
     * Only used to emulate some related literature.
     * 
     * The paper shows that this function is bad for the stability of the framework.
     */
    void filterOuterMarking();

    /*!
     * Set bead count in marked regions based on the optimal_bead_count of the beading strategy.
     */
    void updateBeadCount();

    /*!
     * Add marked regions and set bead counts
     * where there is an end of marking and when traveling upward we get to another region with the same bead count
     */
    void filterUnmarkedRegions();

    /*!
     * 
     * \return Whether to set the bead count on the way back
     */
    bool filterUnmarkedRegions(edge_t* to_edge, coord_t bead_count, coord_t traveled_dist, coord_t max_dist);

    void generateTransitionMids(ptr_vector_t<std::list<TransitionMiddle>>& edge_transitions);

    void filterTransitionMids();

    /*!
     * 
     * \param edge_to_start edge pointing to the node from which to start traveling in all directions except along \p edge_to_start
     * \param origin_transition The transition for which we are checking nearby transitions
     * \param traveled_dist the distance traveled before we came to \p edge_to_start.to
     * \param going_up Whether we are traveling in the upward direction as seen from the \p origin_transition. If this doesn't align with the direction according to the R diff on a consecutive edge we know there was a local optimum
     * \return whether the origin transition should be dissolved
     */
    std::list<TransitionMidRef> dissolveNearbyTransitions(edge_t* edge_to_start, TransitionMiddle& origin_transition, coord_t traveled_dist, coord_t max_dist, bool going_up);

    void dissolveBeadCountRegion(edge_t* edge_to_start, coord_t from_bead_count, coord_t to_bead_count);

    bool filterEndOfMarkingTransition(edge_t* edge_to_start, coord_t traveled_dist, coord_t max_dist, coord_t replacing_bead_count);

    void generateTransitionEnds(ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends);

    /*!
     * Also set the rest values at nodes in between the transition ends
     */
    void applyTransitions();

    void generateTransitioningRibs();

    /*!
     * \param edge_to_transition_mids From the upward halfedges to their transitions mids
     */
    void generateTransition(edge_t& edge, coord_t mid_R, coord_t transition_lower_bead_count, ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends);

    /*!
     * \p start_rest and \p end_rest refer to gap distances at the start and end pos in terms of ratios w.r.t. the inner bead width at the high end of the transition
     * 
     * \p end_pos_along_edge may be beyond this edge!
     * In this case we need to interpolate the rest value at the locations in between
     * 
     * \return whether the subgraph is going downward
     */
    bool generateTransitionEnd(edge_t& edge, coord_t start_pos, coord_t end_pos, coord_t transition_half_length, float start_rest, float end_rest, coord_t transition_lower_bead_count, ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends);

    bool isGoingDown(edge_t* outgoing, coord_t traveled_dist, coord_t transition_half_length, coord_t lower_bead_count) const;

    bool isEndOfMarking(const edge_t& edge) const;


    void generateExtraRibs();

    // ^ transitioning | v toolpath generation

    /*!
     * \param[out] segments the generated segments
     */
    void generateSegments(std::vector<std::list<ExtrusionLine>>& result_polylines_per_index);

    edge_t* getQuadMaxRedgeTo(edge_t* quad_start_edge);

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
    void propagateBeadingsUpward(std::vector<edge_t*>& upward_quad_mids, ptr_vector_t<BeadingPropagation>& node_beadings);

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
    void propagateBeadingsDownward(std::vector<edge_t*>& upward_quad_mids, ptr_vector_t<BeadingPropagation>& node_beadings);
    
    void propagateBeadingsDownward(edge_t* edge_to_peak, ptr_vector_t<BeadingPropagation>& node_beadings);

    /*!
     * \param switching_radius The radius at which we switch from the left beading to the merged
     */
    Beading interpolate(const Beading& left, float ratio_left_to_whole, const Beading& right, coord_t switching_radius) const;
    Beading interpolate(const Beading& left, float ratio_left_to_whole, const Beading& right) const;

    std::shared_ptr<BeadingPropagation> getOrCreateBeading(node_t* node, ptr_vector_t<BeadingPropagation>& node_beadings);

    /*!
     * In case we cannot find the beading of a node, get a beading from the nearest node
     */
    std::shared_ptr<BeadingPropagation> getNearestBeading(node_t* node, coord_t max_dist);

    /*!
     * generate junctions for each bone
     * \param edge_to_junctions junctions ordered high R to low R
     */
    void generateJunctions(ptr_vector_t<BeadingPropagation>& node_beadings, ptr_vector_t<std::vector<ExtrusionJunction>>& edge_junctions);

    /*!
     * connect junctions in each quad
     * \param edge_to_junctions junctions ordered high R to low R
     * \param[out] segments the generated segments
     */
    void connectJunctions(std::vector<std::list<ExtrusionLine>>& result_polylines_per_index);

    /*!
     * Genrate small segments for local maxima where the beading would only result in a single bead
     * \param[out] segments the generated segments
     */
    void generateLocalMaximaSingleBeads(std::vector<std::list<ExtrusionLine>>& result_polylines_per_index);
};

} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_H
