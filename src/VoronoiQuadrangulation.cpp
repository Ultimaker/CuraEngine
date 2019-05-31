//Copyright (c) 2019 Ultimaker B.V.
#include "VoronoiQuadrangulation.h"

#include <stack>

#include "utils/VoronoiUtils.h"

#include "utils/linearAlg2D.h"
#include "utils/IntPoint.h"

#include "utils/logoutput.h"

#include "utils/macros.h"

namespace boost {
namespace polygon {

template <>
struct geometry_concept<arachne::Point>
{
    typedef point_concept type;
};

template <>
struct point_traits<arachne::Point>
{
    typedef int coordinate_type;

    static inline coordinate_type get(
            const arachne::Point& point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.X : point.Y;
    }
};

template <>
struct geometry_concept<arachne::VoronoiQuadrangulation::Segment>
{
    typedef segment_concept type;
};

template <>
struct segment_traits<arachne::VoronoiQuadrangulation::Segment>
{
    typedef arachne::coord_t coordinate_type;
    typedef arachne::Point point_type;
    static inline point_type get(const arachne::VoronoiQuadrangulation::Segment& segment, direction_1d dir) {
        return dir.to_int() ? segment.p() : segment.next().p();
    }
};
}    // polygon
}    // boost



namespace arachne
{

VoronoiQuadrangulation::node_t& VoronoiQuadrangulation::make_node(vd_t::vertex_type& vd_node, Point p)
{
    auto he_node_it = vd_node_to_he_node.find(&vd_node);
    if (he_node_it == vd_node_to_he_node.end())
    {
        graph.nodes.emplace_front(VoronoiQuadrangulationJoint(), p);
        node_t& node = graph.nodes.front();
        vd_node_to_he_node.emplace(&vd_node, &node);
        return node;
    }
    else
    {
        return *he_node_it->second;
    }
}

void VoronoiQuadrangulation::transfer_edge(Point from, Point to, vd_t::edge_type& vd_edge, edge_t*& prev_edge, Point& start_source_point, Point& end_source_point, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    assert(from != to);
    auto he_edge_it = vd_edge_to_he_edge.find(vd_edge.twin());
    if (he_edge_it != vd_edge_to_he_edge.end())
    { // twin segment(s) already made
        edge_t* source_twin = he_edge_it->second;
        assert(source_twin);
        for (edge_t* twin = source_twin;
            ; //  !twin;
            twin = twin->prev->twin->prev)
        {
            assert(twin);
            graph.edges.emplace_front(VoronoiQuadrangulationEdge());
            edge_t* edge = &graph.edges.front();
            edge->from = twin->to;
            edge->to = twin->from;
            edge->twin = twin;
            twin->twin = edge;
            edge->from->some_edge = edge;
            
            if (prev_edge)
            {
                edge->prev = prev_edge;
                prev_edge->next = edge;
            }
            
            prev_edge = edge;
            
//             if (shorterThen(twin->from->p - to, snap_dist))
            if (twin->from->p == to)
            {
                break;
            }
            
            if (!twin->prev || !twin->prev->twin || !twin->prev->twin->prev)
            {
                RUN_ONCE(logError("Discretized segment behaves oddly!\n"));
                return;
            }
            assert(twin->prev); // forth rib
            assert(twin->prev->twin); // back rib
            assert(twin->prev->twin->prev); // prev segment along parabola
            bool is_next_to_start_or_end = false; // only ribs at the end of a cell should be skipped
            make_rib(prev_edge, start_source_point, end_source_point, is_next_to_start_or_end);
        }
        assert(prev_edge);
    }
    else
    {
        std::vector<Point> discretized = discretize(vd_edge, points, segments);
        assert(discretized.size() >= 2);
        
        assert(!prev_edge || prev_edge->to);
//         assert(!prev_edge || prev_edge->to == &make_node(*vd_edge.vertex0(), from)); // TODO: investigate whether boost:voronoi can produce multiple verts and violates consistency
        node_t* v0 = (prev_edge)? prev_edge->to : &make_node(*vd_edge.vertex0(), from);
        Point p0 = discretized.front();
        for (size_t p1_idx = 1; p1_idx < discretized.size(); p1_idx++)
        {
            Point p1 = discretized[p1_idx];
            node_t* v1;
            if (p1_idx < discretized.size() - 1)
            {
                graph.nodes.emplace_front(VoronoiQuadrangulationJoint(), p1);
                v1 = &graph.nodes.front();
            }
            else
            {
                v1 = &make_node(*vd_edge.vertex1(), to);
            }

            graph.edges.emplace_front(VoronoiQuadrangulationEdge());
            edge_t* edge = &graph.edges.front();
            edge->from = v0;
            edge->to = v1;
//             edge->twin = nullptr;
            edge->from->some_edge = edge;
            
            if (prev_edge)
            {
                edge->prev = prev_edge;
                prev_edge->next = edge;
            }
            
            prev_edge = edge;
            p0 = p1;
            v0 = v1;
            
            if (p1_idx < discretized.size() - 1)
            { // rib for last segment gets introduced outside this function!
                bool is_next_to_start_or_end = false; // only ribs at the end of a cell should be skipped
                make_rib(prev_edge, start_source_point, end_source_point, is_next_to_start_or_end);
            }
        }
        assert(prev_edge);
        vd_edge_to_he_edge.emplace(&vd_edge, prev_edge);
    }
}

void VoronoiQuadrangulation::make_rib(edge_t*& prev_edge, Point start_source_point, Point end_source_point, bool is_next_to_start_or_end)
{
    Point p = LinearAlg2D::getClosestOnLineSegment(prev_edge->to->p, start_source_point, end_source_point);
    coord_t dist = vSize(prev_edge->to->p - p);
    prev_edge->to->data.distance_to_boundary = dist;
    assert(dist > 0);

    if (start_source_point != end_source_point
        && is_next_to_start_or_end
        && (shorterThen(p - start_source_point, rib_snap_distance)
        || shorterThen(p - end_source_point, rib_snap_distance)))
        {
            return;
        }
    graph.nodes.emplace_front(VoronoiQuadrangulationJoint(), p);
    node_t* node = &graph.nodes.front();
    node->data.distance_to_boundary = 0;
    
    graph.edges.emplace_front(VoronoiQuadrangulationEdge(VoronoiQuadrangulationEdge::EXTRA_VD));
    edge_t* forth_edge = &graph.edges.front();
    graph.edges.emplace_front(VoronoiQuadrangulationEdge(VoronoiQuadrangulationEdge::EXTRA_VD));
    edge_t* back_edge = &graph.edges.front();
    
    prev_edge->next = forth_edge;
    forth_edge->prev = prev_edge;
    forth_edge->from = prev_edge->to;
    forth_edge->to = node;
    forth_edge->twin = back_edge;
    back_edge->twin = forth_edge;
    back_edge->from = node;
    back_edge->to = prev_edge->to;
    node->some_edge = back_edge;
    
    prev_edge = back_edge;
}

std::vector<Point> VoronoiQuadrangulation::discretize(const vd_t::edge_type& vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    const vd_t::cell_type* left_cell = vd_edge.cell();
    const vd_t::cell_type* right_cell = vd_edge.twin()->cell();
    Point start = VoronoiUtils::p(vd_edge.vertex0());
    Point end = VoronoiUtils::p(vd_edge.vertex1());
    
    bool point_left = left_cell->contains_point();
    bool point_right = right_cell->contains_point();
    if ((!point_left && !point_right)
        || vd_edge.is_secondary() // source vert is directly connected to source segment
    )
    {
        return std::vector<Point>({ start, end });
    }
    else if (point_left == !point_right)
    {
        const vd_t::cell_type* point_cell = left_cell;
        const vd_t::cell_type* segment_cell = right_cell;
        if (!point_left)
        {
            std::swap(point_cell, segment_cell);
        }
        Point p = VoronoiUtils::getSourcePoint(*point_cell, points, segments);
        const Segment& s = VoronoiUtils::getSourceSegment(*segment_cell, points, segments);
        return VoronoiUtils::discretizeParabola(p, s, start, end, discretization_step_size);
    }
    else
    {
        Point a = start;
        Point b = end;
        std::vector<Point> ret;
        ret.emplace_back(a);
        Point ab = b - a;
        coord_t ab_size = vSize(ab);
        coord_t step_count = (ab_size + discretization_step_size / 2) / discretization_step_size;
        for (coord_t step = 1; step < step_count; step++)
        {
            ret.emplace_back(a + ab * step / step_count);
        }
        ret.emplace_back(b);
        return ret;
    }
}


bool VoronoiQuadrangulation::computePointCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    if (cell.incident_edge()->is_infinite())
    {
        return false;
    }
    // check if any point of the cell is inside or outside polygon
    // copy whole cell into graph or not at all
    
    const Point source_point = VoronoiUtils::getSourcePoint(cell, points, segments);
    const PolygonsPointIndex source_point_index = VoronoiUtils::getSourcePointIndex(cell, points, segments);
    Point some_point = VoronoiUtils::p(cell.incident_edge()->vertex0());
    if (some_point == source_point)
    {
        some_point = VoronoiUtils::p(cell.incident_edge()->vertex1());
    }
    if (!LinearAlg2D::isInsideCorner(source_point_index.prev().p(), source_point_index.p(), source_point_index.next().p(), some_point))
    { // cell is outside of polygon
        return false; // don't copy any part of this cell
    }
    bool first = true;
    for (vd_t::edge_type* vd_edge = cell.incident_edge(); vd_edge != starting_vd_edge || first; vd_edge = vd_edge->next())
    {
        assert(vd_edge->is_finite());
        Point p1 = VoronoiUtils::p(vd_edge->vertex1());
        if (p1 == source_point)
        {
            start_source_point = source_point;
            end_source_point = source_point;
            starting_vd_edge = vd_edge->next();
            ending_vd_edge = vd_edge;
            assert(p1 != VoronoiUtils::p(starting_vd_edge->vertex1()));
        }
        first = false;
    }
    assert(starting_vd_edge && ending_vd_edge);
    assert(starting_vd_edge != ending_vd_edge);
    assert(start_source_point != VoronoiUtils::p(starting_vd_edge->vertex1()));
    return true;
}
void VoronoiQuadrangulation::computeSegmentCellRange(vd_t::cell_type& cell, Point& start_source_point, Point& end_source_point, vd_t::edge_type*& starting_vd_edge, vd_t::edge_type*& ending_vd_edge, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    const Segment& source_segment = VoronoiUtils::getSourceSegment(cell, points, segments);
    // find starting edge
    // find end edge
    bool first = true;
    for (vd_t::edge_type* edge = cell.incident_edge(); edge != cell.incident_edge() || first; edge = edge->next())
    {
        if (edge->is_infinite())
        {
            first = false;
            continue;
        }
        if (false && edge->is_secondary())
        { // edge crosses source segment
            // TODO: handle the case where two consecutive line segments are collinear!
            // that's the only case where a voronoi segment doesn't end in a polygon vertex, but goes though it
            if (LinearAlg2D::pointLiesOnTheRightOfLine(VoronoiUtils::p(edge->vertex1()), source_segment.from(), source_segment.to()))
            {
                ending_vd_edge = edge;
            }
            else
            {
                starting_vd_edge = edge;
            }
            first = false;
            continue;
        }
        if (VoronoiUtils::p(edge->vertex0()) == source_segment.to())
        {
            starting_vd_edge = edge;
        }
        if (VoronoiUtils::p(edge->vertex1()) == source_segment.from())
        {
            ending_vd_edge = edge;
        }
        first = false;
    }
    
    assert(starting_vd_edge && ending_vd_edge);
    assert(starting_vd_edge != ending_vd_edge);
    
    start_source_point = source_segment.to();
    end_source_point = source_segment.from();
}

VoronoiQuadrangulation::VoronoiQuadrangulation(const Polygons& polys)
: polys(polys)
{
    init();
}

void VoronoiQuadrangulation::init()
{
    std::vector<Point> points; // remains empty

    std::vector<Segment> segments;
    for (size_t poly_idx = 0; poly_idx < polys.size(); poly_idx++)
    {
        ConstPolygonRef poly = polys[poly_idx];
        for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            segments.emplace_back(&polys, poly_idx, point_idx);
        }
    }

    vd_t vd;
    construct_voronoi(points.begin(), points.end(),
                                        segments.begin(), segments.end(),
                                        &vd);

    VoronoiUtils::debugOutput("output/vd.svg", vd, points, segments);
    
    for (const vd_t::edge_type& edge : vd.edges())
    {
        assert(edge.vertex0() == edge.twin()->vertex1());
        assert(edge.vertex1() == edge.twin()->vertex0());
        assert(edge.vertex1() == edge.next()->vertex0());
        assert(edge.vertex0() == edge.prev()->vertex1());
    }

    
    for (vd_t::cell_type cell : vd.cells())
    {
        Point start_source_point;
        Point end_source_point;
        vd_t::edge_type* starting_vd_edge = nullptr;
        vd_t::edge_type* ending_vd_edge = nullptr;
        // compute and store result in above variables
        
        if (cell.contains_point())
        {
            bool keep_going = computePointCellRange(cell, start_source_point, end_source_point, starting_vd_edge, ending_vd_edge, points, segments);
            if (!keep_going)
            {
                continue;
            }
        }
        else
        {
            computeSegmentCellRange(cell, start_source_point, end_source_point, starting_vd_edge, ending_vd_edge, points, segments);
        }
        
        
        // copy start to end edge to graph
        
        edge_t* prev_edge = nullptr;
        transfer_edge(start_source_point, VoronoiUtils::p(starting_vd_edge->vertex1()), *starting_vd_edge, prev_edge, start_source_point, end_source_point, points, segments);
        node_t* starting_node = vd_node_to_he_node[starting_vd_edge->vertex0()];
        starting_node->data.distance_to_boundary = 0;
        // starting_edge->prev = nullptr;
//         starting_edge->from->data.distance_to_boundary = 0; // TODO

        make_rib(prev_edge, start_source_point, end_source_point, true);
        for (vd_t::edge_type* vd_edge = starting_vd_edge->next(); vd_edge != ending_vd_edge; vd_edge = vd_edge->next())
        {
            assert(vd_edge->is_finite());
            Point v1 = VoronoiUtils::p(vd_edge->vertex0());
            Point v2 = VoronoiUtils::p(vd_edge->vertex1());
            if (v1 == v2)
            {
                continue;
            }
            transfer_edge(v1, v2, *vd_edge, prev_edge, start_source_point, end_source_point, points, segments);

            make_rib(prev_edge, start_source_point, end_source_point, vd_edge->next() == ending_vd_edge);
        }

        transfer_edge(VoronoiUtils::p(ending_vd_edge->vertex0()), end_source_point, *ending_vd_edge, prev_edge, start_source_point, end_source_point, points, segments);
        // ending_edge->next = nullptr;
        prev_edge->to->data.distance_to_boundary = 0;

        debugCheckGraphConsistency();
    }

    {
        AABB aabb(polys);
        SVG svg("output/graph.svg", aabb);
        debugOutput(svg, true, true);
        svg.writePolygons(polys, SVG::Color::BLACK, 2);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph2.svg", aabb);
        debugOutput(svg, false, false);
        svg.writePolygons(polys, SVG::Color::BLACK, 2);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph3.svg", aabb);
        debugOutput(svg, false, false, false, true);
        svg.writePolygons(polys, SVG::Color::BLACK, 2);
    }

    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
    
    vd_edge_to_he_edge.clear();
    vd_node_to_he_node.clear();
}

//
// ^^^^^^^^^^^^^^^^^^^^^
//    INITIALIZATION
// =====================
//
// =====================
//    TRANSTISIONING
// vvvvvvvvvvvvvvvvvvvvv
//

Polygons VoronoiQuadrangulation::generateToolpaths(const BeadingStrategy& beading_strategy)
{
    setMarking();

        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

    // set bead count in marked regions
    for (edge_t& edge : graph.edges)
    {
        if (edge.data.is_marked)
        {
            edge.to->data.bead_count = beading_strategy.optimal_bead_count(edge.to->data.distance_to_boundary * 2);
        }
    }

//     filterMarkedLocalOptima(beading_strategy);
    // TODO: also filter out small minima
    // only if the minimum widths isnt violated!
    

        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

    generateTransitioningRibs(beading_strategy);
    // TODO: redo generateTransitioningRibs and filtering:
    // first generate all transition locations
    // then filter
    // then make ribs for all transition locations at once 
    // This prevents created ribs from inducing a new transition

    debugCheckDecorationConsistency();

    
    {
        AABB aabb(polys);
        SVG svg("output/graph.svg", aabb);
        debugOutput(svg, true, true);
        svg.writePolygons(polys, SVG::Color::BLACK, 2);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph2.svg", aabb);
        debugOutput(svg, false, false);
        svg.writePolygons(polys, SVG::Color::BLACK, 2);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph3.svg", aabb);
        debugOutput(svg, false, false, false, true);
        svg.writePolygons(polys, SVG::Color::BLACK, 2);
    }

    
    // fix bead count at locally maximal R
    // also for marked regions!! See TODOs in generateTransitionEnd(.)

    std::unordered_map<Point, ExtrusionSegment> segments;
    generateSegments(segments, beading_strategy);
    // junctions = generateJunctions

    printf("got %zu toolpath segments\n", segments.size());

    Polygons ret;
    for (auto key_val : segments)
    {
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(key_val.second.from);
        poly.emplace_back(key_val.second.to);
    }
    // ret = connect(junctions)
    return ret;
}

void VoronoiQuadrangulation::setMarking()
{
    for (edge_t& edge : graph.edges)
    {
        assert(edge.twin);
        if (edge.twin->data.is_marked != -1)
        {
            edge.data.is_marked = edge.twin->data.is_marked;
        }
        else
        {
            Point a = edge.from->p;
            Point b = edge.to->p;
            Point ab = b - a;
            coord_t dR = std::abs(edge.to->data.distance_to_boundary - edge.from->data.distance_to_boundary);
            coord_t dD = vSize(ab);
            edge.data.is_marked = dD > 2 * dR;
        }
    }
}



void VoronoiQuadrangulation::filterMarkedLocalOptima(const BeadingStrategy& beading_strategy)
{
    for (edge_t& edge : graph.edges)
    {
        if (!isMarkedLocalOptimum(&edge))
        {
            continue;
        }
        if (isSmallMarkedLocalOptimum(&edge, beading_strategy))
        {
            dissolveMarkedLocalOptimum(&edge);
        }
    }
}


bool VoronoiQuadrangulation::isMarkedLocalOptimum(edge_t* edge_to)
{
    if (!edge_to->data.is_marked)
    {
        return false;
    }
    bool first = true;
    bool has_neighbors = false;
    coord_t R = edge_to->to->data.distance_to_boundary;
    for (edge_t* incomming = edge_to; incomming && (first || incomming != edge_to); incomming = incomming->next->twin)
    {
        if (incomming->from->data.distance_to_boundary > R)
        {
            return false;
        }
        first = false;
        has_neighbors = true;
    }
    return has_neighbors;
}


bool VoronoiQuadrangulation::isSmallMarkedLocalOptimum(edge_t* edge_to, const BeadingStrategy& beading_strategy)
{
    assert(edge_to->data.is_marked);
    assert(edge_to->to->data.distance_to_boundary >= edge_to->from->data.distance_to_boundary);


    coord_t transition_length = beading_strategy.optimal_width; // TODO

    // distance in both directions along the marked region (or multiple if this is a 3-way intersection (hypothetically impossible))
    coord_t longest_dist = -1;
    coord_t second_longest_dist = -1;
    
    coord_t optimum_bead_count = edge_to->to->data.bead_count;
    
    bool first = true;
    for (edge_t* outgoing = edge_to->twin; first || outgoing != edge_to->twin; outgoing = outgoing->twin->next)
    {
//         if (!outgoing->data.is_marked)
//         {
//             continue;
//         }
        coord_t max_dist = getLocalOptimumSize(outgoing, optimum_bead_count, transition_length, 0, beading_strategy);
        if (max_dist >= longest_dist)
        {
            second_longest_dist = longest_dist;
            longest_dist = max_dist;
        }
        else if (max_dist > second_longest_dist)
        {
            second_longest_dist = max_dist;
        }
        first = false;
    }

    return longest_dist + second_longest_dist < transition_length * 3 / 2; // TODO: make filter size user speficied!
}

coord_t VoronoiQuadrangulation::getLocalOptimumSize(edge_t* outgoing, coord_t bead_count, coord_t stop_distance, coord_t traveled_distance, const BeadingStrategy& beading_strategy)
{
    if (outgoing->to->data.bead_count > bead_count
        || traveled_distance > stop_distance
    )
    {
        return stop_distance; // it is not a local region after all
    }
    coord_t start_R = outgoing->from->data.distance_to_boundary; // high R
    coord_t end_R = outgoing->to->data.distance_to_boundary; // low R
    coord_t ab_size = vSize(outgoing->from->p - outgoing->to->p);

    if (start_R == 507)
        printf("gerg\n");
    
    if (end_R < start_R)
    {
        coord_t transition_R = beading_strategy.transition_thickness(bead_count - 1) / 2; // where to transition from bead_count-1 to bead_count
        coord_t transition_pos = ab_size * (transition_R - start_R) / (end_R - start_R);

        if (transition_pos < ab_size)
        { // normal case: further away from local optimum is lower bead count
            return traveled_distance + transition_pos;
        }
    }

    // continue looking further away from the local optimum
    coord_t max_dist = 0;
    for (edge_t* further = outgoing->next; further && further != outgoing->twin; further = further->twin->next)
    {
        if (!further->data.is_marked)
        {
            continue;
        }
        coord_t sub_tree_max = getLocalOptimumSize(further, bead_count, stop_distance, traveled_distance + ab_size, beading_strategy);
        max_dist = std::max(max_dist, sub_tree_max);
    }
    return max_dist;
}

void VoronoiQuadrangulation::dissolveMarkedLocalOptimum(edge_t* edge_to)
{
    coord_t bead_count_to_dissolve = edge_to->to->data.bead_count;
    coord_t new_bead_count = bead_count_to_dissolve - 1;
    std::stack<edge_t*> nodes_to_check; // nodes as pointed to by edges from already checked nodes
    nodes_to_check.emplace(edge_to);
    nodes_to_check.emplace(edge_to->twin);
    while (!nodes_to_check.empty())
    {
        edge_t* to_check = nodes_to_check.top();
        nodes_to_check.pop();
        if (to_check->to->data.bead_count == bead_count_to_dissolve)
        {
            to_check->to->data.bead_count = new_bead_count;
            for (edge_t* outgoing = to_check->next; outgoing && outgoing->twin != to_check; outgoing = outgoing->twin->next)
            {
                nodes_to_check.emplace(outgoing);
            }
        }
    }
}


void VoronoiQuadrangulation::generateTransitioningRibs(const BeadingStrategy& beading_strategy)
{
    for (edge_t& edge : graph.edges)
    {
        assert(edge.data.is_marked != -1);
        if (!edge.data.is_marked)
        { // only marked regions introduce transitions
            continue;
        }
        coord_t start_R = edge.from->data.distance_to_boundary;
        coord_t end_R = edge.to->data.distance_to_boundary;
        coord_t start_bead_count = edge.from->data.bead_count;
        coord_t end_bead_count = edge.to->data.bead_count;

        if (start_R == end_R)
        { // no transitions occur when both end points have the same distance_to_boundary
            continue;
        }
        else if (start_R > end_R)
        { // only consider those half-edges which are going from a lower to a higher distance_to_boundary
            if (
                isEndOfMarking(edge)
                && end_bead_count > 0 // we cannot transition from -1 to 0 beads!
                && edge.to->data.transition_rest != 0 // the end isn't already part of a transition
            )
            { // check for lower end transitions
                isEndOfMarking(edge);
                coord_t transition_R = beading_strategy.transition_thickness(end_bead_count - 1) / 2;
                coord_t last_edge_size = vSize(edge.from->p - edge.to->p);

                coord_t transition_length = beading_strategy.optimal_width; // TODO
                float transition_mid_position = 0.5; // TODO
                coord_t inner_bead_width_after_transition = beading_strategy.optimal_width; // TODO

                coord_t mid_pos = last_edge_size * (transition_R - start_R) / (end_R - start_R);
                assert(mid_pos > last_edge_size);

                coord_t transition_end_pos = mid_pos - transition_mid_position * transition_length;
                if (transition_end_pos < last_edge_size)
                {
                    coord_t inv_transition_end_pos = last_edge_size - transition_end_pos;
                    coord_t last_edge_end_rest = inner_bead_width_after_transition * inv_transition_end_pos / transition_length;
                    edge.to->data.transition_rest = last_edge_end_rest;
                    edge.to->data.bead_count = end_bead_count - 1;
                    generateTransitionEnd(*edge.twin, 0, inv_transition_end_pos, last_edge_end_rest, inner_bead_width_after_transition, end_bead_count - 1);
                }
            }
            continue;
        }

        if (start_bead_count > beading_strategy.optimal_bead_count(start_R * 2)
            || end_bead_count > beading_strategy.optimal_bead_count(end_R * 2))
        { // wasn't the case earlier in this function because of already introduced transitions
            RUN_ONCE(logError("transitioning segment overlap! (?)\n"));
        }
        assert(start_R < end_R);
        edge_t* last_edge = &edge;
        for (coord_t transition_lower_bead_count = start_bead_count; transition_lower_bead_count < end_bead_count; transition_lower_bead_count++)
        {
            assert(last_edge);
            coord_t last_edge_start_R = last_edge->from->data.distance_to_boundary;
            coord_t last_edge_end_R = last_edge->to->data.distance_to_boundary;
            coord_t last_edge_size = vSize(last_edge->from->p - last_edge->to->p);
            coord_t mid_R = beading_strategy.transition_thickness(transition_lower_bead_count) / 2;
            if (mid_R > last_edge_end_R)
            {
                RUN_ONCE(logError("transition on segment lies outside of segment!\n"));
                mid_R = last_edge_end_R;
            }
            if (mid_R < last_edge_start_R)
            {
                RUN_ONCE(logError("transition on segment lies outside of segment!\n"));
                mid_R = last_edge_start_R;
            }
            coord_t mid_pos = last_edge_size * (mid_R - last_edge_start_R) / (last_edge_end_R - last_edge_start_R);
            assert(mid_pos >= 0);
            assert(mid_pos <= last_edge_size);
            last_edge = generateTransition(*last_edge, mid_pos, beading_strategy, transition_lower_bead_count);
        }
        
        // check for end-of-markings in both directions
        if (isEndOfMarking(*last_edge)
            && end_bead_count >= 0 // we cannot transition from -1 to 0
        )
        {
            coord_t last_edge_start_R = last_edge->from->data.distance_to_boundary;
            coord_t last_edge_end_R = last_edge->to->data.distance_to_boundary;
            assert(last_edge_end_R > last_edge_start_R);
            coord_t transition_R = beading_strategy.transition_thickness(end_bead_count) / 2;
            coord_t last_edge_size = vSize(last_edge->from->p - last_edge->to->p);

            coord_t transition_length = beading_strategy.optimal_width; // TODO
            float transition_mid_position = 0.5; // TODO
            coord_t inner_bead_width_after_transition = beading_strategy.optimal_width; // TODO

            coord_t mid_pos = last_edge_size * (transition_R - last_edge_start_R) / (last_edge_end_R - last_edge_start_R);
            assert(mid_pos > last_edge_size);

            coord_t transition_end_pos = mid_pos - transition_mid_position * transition_length;
            if (transition_end_pos < last_edge_size)
            {
                coord_t inv_transition_end_pos = last_edge_size - transition_end_pos;
                coord_t last_edge_end_rest = inner_bead_width_after_transition * inv_transition_end_pos / transition_length;
                last_edge->to->data.transition_rest = last_edge_end_rest;
                last_edge->to->data.bead_count = end_bead_count;
                generateTransitionEnd(*last_edge->twin, 0, inv_transition_end_pos, last_edge_end_rest, 0, end_bead_count);
            }
        }
    }
}

VoronoiQuadrangulation::edge_t* VoronoiQuadrangulation::generateTransition(edge_t& edge, coord_t mid_pos, const BeadingStrategy& beading_strategy, coord_t lower_bead_count)
{
    Point a = edge.from->p;
    Point b = edge.to->p;
    Point ab = b - a;
    coord_t ab_size = vSize(ab);

    coord_t transition_length = beading_strategy.optimal_width; // TODO
    float transition_mid_position = 0.5; // TODO
    coord_t inner_bead_width_after_transition = beading_strategy.optimal_width; // TODO

    coord_t start_rest = 0;
    coord_t mid_rest = transition_mid_position * inner_bead_width_after_transition;
    coord_t end_rest = inner_bead_width_after_transition;

    
        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

    // upper bead count transition end
    edge_t* replacing_last_edge = generateTransitionEnd(edge, mid_pos, mid_pos + (1.0 - transition_mid_position) * transition_length, mid_rest, end_rest, lower_bead_count);
        debugCheckGraphConsistency();
    // lower bead count transition end
    edge_t* edge_at_mid_position = (replacing_last_edge == &edge)? &edge : replacing_last_edge->prev->twin->prev;
    coord_t start_pos = vSize(b - normal(ab, ab_size - mid_pos) - edge_at_mid_position->to->p);
    coord_t end_pos = start_pos + transition_mid_position * transition_length;
    if (lower_bead_count == 0)
    {
        // put the transition end which trnsitions to 0 at the anchor point of the transition
        // there won't be a physical transition, so it doesn't require a distance.
        end_pos = start_pos;
        lower_bead_count = 1;
        // TODO: what about the upper end transition? I'm not sure we can leave it out.
        // it can't really hurt to have it there, though
    }
    generateTransitionEnd(*edge_at_mid_position->twin, start_pos, end_pos, mid_rest, start_rest, lower_bead_count);
    
        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();
    
    return replacing_last_edge;
}

VoronoiQuadrangulation::edge_t* VoronoiQuadrangulation::generateTransitionEnd(edge_t& edge, coord_t start_pos, coord_t end_pos, coord_t start_rest, coord_t end_rest, coord_t lower_bead_count)
{
    Point a = edge.from->p;
    Point b = edge.to->p;
    Point ab = b - a;
    coord_t ab_size = vSize(ab); // TODO: prevent recalculation of these values

    assert(start_pos <= ab_size);

    edge_t* last_edge_replacing_input = &edge;

    if (shorterThen(end_pos - ab_size, snap_dist))
    {
        edge.to->data.transition_rest = 0; // the transition rest should be zero at both sides of the transition; the intermediate values should only occur in between
        edge.data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
        return last_edge_replacing_input;
    }
    else if (end_pos > ab_size)
    { // recurse on all further edges
        edge.data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
        coord_t R = edge.to->data.distance_to_boundary;
        coord_t rest = end_rest - (start_rest - end_rest) * (end_pos - ab_size) / (start_pos - end_pos);
        assert(rest >= 0);
        assert(rest <= std::max(end_rest, start_rest));
        assert(rest >= std::min(end_rest, start_rest));
        edge.to->data.transition_rest = rest;
        edge.to->data.bead_count = lower_bead_count;
        int i = 0;
        for (edge_t* outgoing = edge.next; outgoing != edge.twin;)
        {
            edge_t* next = outgoing->twin->next; // before we change the outgoing edge itself
            assert(i < 10);
            if (!outgoing->data.is_marked)
            {
                outgoing = next;
                continue; // don't put transition ends in non-marked regions
            }
            if (outgoing->to->data.distance_to_boundary < R)
            { // what to do with decreasing R along marked regions?!?!?! TODO
                // if it is decreasing in all directions, should we unapply the whole transition? TODO
                RUN_ONCE(logError("what to do with decreasing R along marked regions?!?!?!\n"));
//                 continue; // TODO 
            }

            generateTransitionEnd(*outgoing, 0, end_pos - ab_size, rest, end_rest, lower_bead_count);
            outgoing = next;
        }
        return last_edge_replacing_input; // [edge] was not altered; we altered other edges
    }
    else // end_pos < ab_size
    { // split input edge
        assert(edge.data.is_marked);
        Point mid = a + normal(ab, end_pos);
        
        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

        graph.nodes.emplace_back(VoronoiQuadrangulationJoint(), mid);
        node_t* mid_node = &graph.nodes.back();
        mid_node->data.bead_count = (end_rest == 0)? lower_bead_count : lower_bead_count + 1;

        edge_t& twin = *edge.twin;
        last_edge_replacing_input = insertRib(edge, mid_node);
        edge_t* last_edge_replacing_twin = insertRib(twin, mid_node);

        edge_t* first_edge_replacing_input = last_edge_replacing_input->prev->twin->prev;
        edge_t* first_edge_replacing_twin = last_edge_replacing_twin->prev->twin->prev;
        first_edge_replacing_input->twin = last_edge_replacing_twin;
        last_edge_replacing_twin->twin = first_edge_replacing_input;
        last_edge_replacing_input->twin = first_edge_replacing_twin;
        first_edge_replacing_twin->twin = last_edge_replacing_input;

        first_edge_replacing_input->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
        last_edge_replacing_twin->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;

        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();
        return last_edge_replacing_input;
    }
}

VoronoiQuadrangulation::edge_t* VoronoiQuadrangulation::insertRib(edge_t& edge, node_t* mid_node)
{
    edge_t* edge_before = edge.prev;
    edge_t* edge_after = edge.next;
    node_t* node_before = edge.from;
    node_t* node_after = edge.to;
    
    Point p = mid_node->p;

    std::pair<Point, Point> source_segment = getSource(edge);
    Point px = LinearAlg2D::getClosestOnLineSegment(p, source_segment.first, source_segment.second);
    coord_t dist = vSize(p - px);
    assert(dist > 0);
    mid_node->data.distance_to_boundary = dist;
    mid_node->data.transition_rest = 0; // both transition end should have rest = 0

    graph.nodes.emplace_back(VoronoiQuadrangulationJoint(), px);
    node_t* source_node = &graph.nodes.back();
    source_node->data.distance_to_boundary = 0;

    edge_t* first = &edge;
    graph.edges.emplace_back(VoronoiQuadrangulationEdge());
    edge_t* second = &graph.edges.back();
    graph.edges.emplace_back(VoronoiQuadrangulationEdge(VoronoiQuadrangulationEdge::TRANSITION_END));
    edge_t* outward_edge = &graph.edges.back();
    graph.edges.emplace_back(VoronoiQuadrangulationEdge(VoronoiQuadrangulationEdge::TRANSITION_END));
    edge_t* inward_edge = &graph.edges.back();

    if (edge_before) edge_before->next = first;
    first->next = outward_edge;
    outward_edge->next = nullptr;
    inward_edge->next = second;
    second->next = edge_after;

    if (edge_after) edge_after->prev = second;
    second->prev = inward_edge;
    inward_edge->prev = nullptr;
    outward_edge->prev = first;
    first->prev = edge_before;

    first->to = mid_node;
    outward_edge->to = source_node;
    inward_edge->to = mid_node;
    second->to = node_after;

    first->from = node_before;
    outward_edge->from = mid_node;
    inward_edge->from = source_node;
    second->from = mid_node;

    node_before->some_edge = first;
    mid_node->some_edge = outward_edge;
    source_node->some_edge = inward_edge;
    if (edge_after) node_after->some_edge = edge_after;

    first->data.is_marked = true;
    outward_edge->data.is_marked = false; // TODO verify this is always the case.
    inward_edge->data.is_marked = false;
    second->data.is_marked = true;

    outward_edge->twin = inward_edge;
    inward_edge->twin = outward_edge;

    first->twin = nullptr; // we don't know these yet!
    second->twin = nullptr;

    assert(second->prev->from->data.distance_to_boundary == 0);

    return second;
}
std::pair<Point, Point> VoronoiQuadrangulation::getSource(const edge_t& edge)
{
    const edge_t* from_edge;
    for (from_edge = &edge; from_edge->prev; from_edge = from_edge->prev) {}
    const edge_t* to_edge;
    for (to_edge = &edge; to_edge->next; to_edge = to_edge->next) {}
    return std::make_pair(from_edge->from->p, to_edge->to->p);
}

bool VoronoiQuadrangulation::isEndOfMarking(const edge_t& edge_to)
{
    if (!edge_to.next)
    {
        return false;
    }
    assert(edge_to.data.is_marked); // Don't know why this function would otherwise be called
    for (const edge_t* edge = edge_to.next; edge && edge->twin && edge != &edge_to; edge = edge->twin->next)
    {
        if (edge->data.is_marked)
        {
            return false;
        }
    }
    return true;
}



//
// ^^^^^^^^^^^^^^^^^^^^^
//    TRANSTISIONING
// =====================
//
// =====================
//  TOOLPATH GENERATION
// vvvvvvvvvvvvvvvvvvvvv
//

void VoronoiQuadrangulation::generateSegments(std::unordered_map<Point, ExtrusionSegment>& segments, const BeadingStrategy& beading_strategy)
{
    std::vector<edge_t*> quad_starts;
    for (edge_t& edge : graph.edges)
    {
        if (!edge.prev)
        {
            quad_starts.emplace_back(&edge);
        }
    }
    std::sort(quad_starts.begin(), quad_starts.end(), [this](edge_t* a, edge_t* b) { return getQuadMaxR(a) < getQuadMaxR(b); });
    printf("got %zu cells\n", quad_starts.size());
    
    std::unordered_map<node_t*, Beading> node_to_beading;
    std::unordered_map<edge_t*, std::vector<Junction>> edge_to_junctions; // junctions ordered high R to low R
    for (edge_t* quad_start : quad_starts)
    {
        edge_t* edge_to_peak = getQuadMaxRedgeTo(quad_start);
        // walk down on both sides and connect junctions
        edge_t* edge_from_peak = edge_to_peak->next; assert(edge_from_peak);
        std::vector<Junction> from_junctions = getJunctions(edge_to_peak, node_to_beading, edge_to_junctions, beading_strategy);
        std::vector<Junction> to_junctions = getJunctions(edge_from_peak->twin, node_to_beading, edge_to_junctions, beading_strategy);
        if (edge_to_peak->prev)
        {
            std::vector<Junction> from_prev_junctions = getJunctions(edge_to_peak->prev, node_to_beading, edge_to_junctions, beading_strategy);
            from_junctions.reserve(from_junctions.size() + from_prev_junctions.size());
            from_junctions.insert(from_junctions.end(), from_prev_junctions.begin(), from_prev_junctions.end());
            assert(!edge_to_peak->prev->prev);
        }
        if (edge_from_peak->next)
        {
            std::vector<Junction> to_next_junctions = getJunctions(edge_from_peak->next->twin, node_to_beading, edge_to_junctions, beading_strategy);
            to_junctions.reserve(to_junctions.size() + to_next_junctions.size());
            to_junctions.insert(to_junctions.end(), to_next_junctions.begin(), to_next_junctions.end());
            assert(!edge_from_peak->next->next);
        }
        assert(std::abs(int(from_junctions.size()) - int(to_junctions.size())) <= 1); // at transitions one end has more beads
        
        // deal with transition to single bead
        if (quad_start->next->next && quad_start->next->data.is_marked && quad_start->to->data.bead_count % 2 + quad_start->next->to->data.bead_count % 2 == 1)
        {
            assert(quad_start->next->data.type == VoronoiQuadrangulationEdge::TRANSITION_MID);
//             if (quad_start->to->data.bead_count % 2 == 0)
//             {
                
        }
        
        size_t segment_count = std::min(from_junctions.size(), to_junctions.size());
        for (size_t junction_rev_idx = 0; junction_rev_idx < segment_count; junction_rev_idx++)
        {
            Junction& from = from_junctions[from_junctions.size() - 1 - junction_rev_idx];
            Junction& to = to_junctions[to_junctions.size() - 1 - junction_rev_idx];
            
            segments.emplace(from.p, ExtrusionSegment(from.p, from.w, to.p, to.w));
        }
    }
}

coord_t VoronoiQuadrangulation::getQuadMaxR(edge_t* quad_start_edge)
{
    return getQuadMaxRedgeTo(quad_start_edge)->to->data.distance_to_boundary;
}

VoronoiQuadrangulation::edge_t* VoronoiQuadrangulation::getQuadMaxRedgeTo(edge_t* quad_start_edge)
{
    assert(quad_start_edge->prev == nullptr);
    assert(quad_start_edge->from->data.distance_to_boundary == 0);
    coord_t max_R = -1;
    edge_t* ret = nullptr;
    for (edge_t* edge = quad_start_edge; edge; edge = edge->next)
    {
        coord_t r = edge->to->data.distance_to_boundary;
        if (r > max_R)
        {
            max_R = r;
            ret = edge;
        }
    }
    assert(ret->next);
    assert(ret);
    return ret;
}

const std::vector<VoronoiQuadrangulation::Junction>& VoronoiQuadrangulation::getJunctions(edge_t* edge, std::unordered_map<node_t*, BeadingStrategy::Beading>& node_to_beading, std::unordered_map<edge_t*, std::vector<Junction>>& edge_to_junctions, const BeadingStrategy& beading_strategy)
{
    assert(edge->to->data.distance_to_boundary >= edge->from->data.distance_to_boundary);
    auto ret_it = edge_to_junctions.find(edge);
    if (ret_it != edge_to_junctions.end())
    {
        return ret_it->second;
    }
    auto beading_it = node_to_beading.find(edge->to);
    if (beading_it == node_to_beading.end())
    {
        beading_it = node_to_beading.emplace(edge->to, beading_strategy.compute(edge->to->data.distance_to_boundary * 2, edge->to->data.bead_count)).first;
    }
    Beading* beading = &beading_it->second;
    std::vector<VoronoiQuadrangulation::Junction>& ret = edge_to_junctions[edge]; // emplace a new vector
    if (edge->to->data.bead_count == 0 && edge->from->data.bead_count == 0)
    {
        return ret;
    }

    Point a = edge->to->p;
    Point b = edge->from->p;
    Point ab = b - a;

    coord_t start_R = edge->to->data.distance_to_boundary; // higher R
    coord_t end_R = edge->from->data.distance_to_boundary; // lower R
    if (end_R == start_R)
    {
        return ret;
    }
    assert(end_R <= start_R);

    coord_t junction_idx;
    // compute starting junction_idx for this segment
    for (junction_idx = beading->toolpath_locations.size() / 2; junction_idx > 0; junction_idx--)
    {
        coord_t bead_R = beading->toolpath_locations[junction_idx];
        if (bead_R <= start_R)
        { // junction coinciding with start node is used in this function call
            break;
        }
    }

    for (; junction_idx >= 0; junction_idx--)
    {
        coord_t bead_R = beading->toolpath_locations[junction_idx];
        assert(bead_R > 0);
        if (bead_R < end_R)
        { // junction coinciding with a node is handled by the next segment
            break;
        }
        ret.emplace_back(a + ab * (bead_R - start_R) / (end_R - start_R), beading->bead_widths[junction_idx]);
    }

    return ret;
}

//
// ^^^^^^^^^^^^^^^^^^^^^
//  TOOLPATH GENERATION
// =====================
//
// =====================
//       HELPERS
// vvvvvvvvvvvvvvvvvvvvv
//


void VoronoiQuadrangulation::debugCheckGraphCompleteness()
{
    for (const node_t& node : graph.nodes)
    {
        if (!node.some_edge)
        {
            assert(false);
        }
    }
    for (const edge_t& edge : graph.edges)
    {
        if (!edge.twin || !edge.from || !edge.to)
        {
            assert(false);
        }
        assert(edge.next || edge.to->data.distance_to_boundary == 0);
        assert(edge.prev || edge.from->data.distance_to_boundary == 0);
    }
}

void VoronoiQuadrangulation::debugCheckGraphConsistency()
{
    auto vert_assert = [](const node_t* first, const node_t* second)
    {
        if (first != second)
        {
            if (first->p == second->p)
            {
                RUN_ONCE(logWarning("Unneccesary duplicatation of VoronoiQuadrangulation nodes!"));
            }
            else
            {
                assert(false && "connected edges don't refer to the same node!");
            }
        }
    };
    
    for (const edge_t& edge : graph.edges)
    {
        if (edge.twin)
        {
            if (!edge.to)
            {
                assert(!edge.twin->from);
                vert_assert(edge.twin->from, edge.to);
            }
            if (!edge.from)
            {
                assert(!edge.twin->to);
                vert_assert(edge.twin->to, edge.from);
            }
            assert(edge.twin->twin == &edge);
        }
        if (edge.next)
        {
            vert_assert(edge.next->from, edge.to);
        }
        if (edge.prev)
        {
            vert_assert(edge.prev->to, edge.from);
        }
    }
    for (const node_t& node : graph.nodes)
    {
        if (node.some_edge)
        {
            vert_assert(node.some_edge->from, &node);
            if (node.some_edge->twin)
            {
                for (const edge_t* outgoing = node.some_edge->twin->next; outgoing && outgoing->twin && outgoing != node.some_edge; outgoing = outgoing->twin->next)
                {
                    vert_assert(outgoing->from, &node);
                }
            }
        }
    }

    for (const edge_t& edge : graph.edges)
    {
        int i = 0;
        for (const edge_t* e = &edge; e; e = e->next) assert(++i < 10);
        for (const edge_t* e = &edge; e; e = e->prev) assert(++i < 10);
    }
}

void VoronoiQuadrangulation::debugCheckDecorationConsistency()
{
    for (const edge_t& edge : graph.edges)
    {
        assert(edge.data.type >= VoronoiQuadrangulationEdge::NORMAL && edge.data.type <= VoronoiQuadrangulationEdge::TRANSITION_MID);
        if (edge.data.type != VoronoiQuadrangulationEdge::NORMAL && edge.data.type != VoronoiQuadrangulationEdge::TRANSITION_MID)
        {
            if (edge.from->data.distance_to_boundary != -1 && edge.to->data.distance_to_boundary != -1)
            {
                assert(edge.from->data.distance_to_boundary == 0 || edge.to->data.distance_to_boundary == 0);
            }
            assert(!edge.data.is_marked);
        }
        if (edge.data.is_marked)
        {
            if (edge.from->data.bead_count != -1 && edge.to->data.bead_count != -1)
            {
                assert(std::abs(edge.from->data.bead_count - edge.to->data.bead_count) <= 1);
            }
        }
    }
}

SVG::Color VoronoiQuadrangulation::getColor(edge_t& edge)
{
    switch (edge.data.type)
    {
        case VoronoiQuadrangulationEdge::EXTRA_VD:
            return SVG::Color::ORANGE;
        case VoronoiQuadrangulationEdge::TRANSITION_END:
            return SVG::Color::MAGENTA;
        case VoronoiQuadrangulationEdge::TRANSITION_MID:
            return SVG::Color::MAGENTA;
        case VoronoiQuadrangulationEdge::NORMAL:
        default:
            return SVG::Color::RED;
    }
}

void VoronoiQuadrangulation::debugOutput(SVG& svg, bool draw_arrows, bool draw_dists, bool draw_bead_counts, bool draw_locations)
{
    for (edge_t& edge : graph.edges)
    {
        Point a = edge.from->p;
        Point b = edge.to->p;
        SVG::Color clr = getColor(edge);
        float stroke_width = 1;
        if (edge.data.is_marked == 1 && edge.data.type != VoronoiQuadrangulationEdge::TRANSITION_MID)
        {
            clr = SVG::Color::BLUE;
            stroke_width = 2;
        }
        if (draw_arrows)
        {
            svg.writeArrow(a, b, clr, stroke_width);
        }
        else
        {
            svg.writeLine(a, b, clr, stroke_width);
        }
    }
    for (node_t& node : graph.nodes)
    {
        if (draw_dists)
        {
            svg.writeText(node.p, std::to_string(node.data.distance_to_boundary));
        }
        if (draw_bead_counts && node.data.bead_count >= 0)
        {
            if (node.data.transition_rest != 0)
            {
                svg.writeText(node.p, std::to_string(float(node.data.transition_rest) / 400 + node.data.bead_count));
            }
            else
            {
                svg.writeText(node.p, std::to_string(node.data.bead_count));
            }
        }
        if (draw_locations)
        {
            svg.writePoint(node.p, true, 1);
        }
    }
}

} // namespace arachne
