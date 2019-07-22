//Copyright (c) 2019 Ultimaker B.V.
#include "VoronoiQuadrangulation.h"

#include <stack>
#include <functional>
#include <unordered_set>
#include <sstream>

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
    auto he_edge_it = vd_edge_to_he_edge.find(vd_edge.twin());
    if (he_edge_it != vd_edge_to_he_edge.end())
    { // twin segment(s) already made
        edge_t* source_twin = he_edge_it->second;
        assert(source_twin);
        auto end_node_it = vd_node_to_he_node.find(vd_edge.vertex1());
        assert(end_node_it != vd_node_to_he_node.end());
        node_t* end_node = end_node_it->second;
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

            if (prev_edge->to == end_node)
            {
                return;
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
    assert(dist >= 0);

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
        return VoronoiUtils::discretizeParabola(p, s, start, end, discretization_step_size, transitioning_angle);
    }
    else
    {
        Point left_point = VoronoiUtils::getSourcePoint(*left_cell, points, segments);
        Point right_point = VoronoiUtils::getSourcePoint(*right_cell, points, segments);
        coord_t d = vSize(right_point - left_point);
        Point middle = (left_point + right_point) / 2;
        Point x_axis_dir = turn90CCW(right_point - left_point);
        coord_t x_axis_length = vSize(x_axis_dir);

        const auto projected_x = [x_axis_dir, x_axis_length, middle](Point from)
        {
            Point vec = from - middle;
            coord_t x = dot(vec, x_axis_dir) / x_axis_length;
            return x;
        };
        coord_t start_x = projected_x(start);
        coord_t end_x = projected_x(end);

        float bound = 0.5 / tan((M_PI - transitioning_angle) * 0.5);
        coord_t marking_start_x = - d * bound;
        coord_t marking_end_x = d * bound;
        Point marking_start = middle + x_axis_dir * marking_start_x / x_axis_length;
        Point marking_end = middle + x_axis_dir * marking_end_x / x_axis_length;
        coord_t dir = 1;
        if (start_x > end_x)
        {
            dir = -1;
            std::swap(marking_start, marking_end);
            std::swap(marking_start_x, marking_end_x);
        }

        Point a = start;
        Point b = end;
        std::vector<Point> ret;
        ret.emplace_back(a);
        
        bool add_marking_start = marking_start_x * dir > start_x * dir;
        bool add_marking_end = marking_end_x * dir > start_x * dir;
        
        Point ab = b - a;
        coord_t ab_size = vSize(ab);
        coord_t step_count = (ab_size + discretization_step_size / 2) / discretization_step_size;
        if (step_count % 2 == 1)
        {
            step_count++; // enforce a discretization point being added in the middle
        }
        for (coord_t step = 1; step < step_count; step++)
        {
            Point here = a + ab * step / step_count;
            coord_t x_here = projected_x(here);
            if (add_marking_start && marking_start_x * dir < x_here * dir)
            {
                ret.emplace_back(marking_start);
                add_marking_start = false;
            }
            if (add_marking_end && marking_end_x * dir < x_here * dir)
            {
                ret.emplace_back(marking_end);
                add_marking_end = false;
            }
            ret.emplace_back(here);
        }
        if (add_marking_end && marking_end_x * dir < end_x * dir)
        {
            ret.emplace_back(marking_end);
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
        }
        first = false;
    }
    assert(starting_vd_edge && ending_vd_edge);
    assert(starting_vd_edge != ending_vd_edge);
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

VoronoiQuadrangulation::VoronoiQuadrangulation(const Polygons& polys, float transitioning_angle)
: polys(polys)
, transitioning_angle(transitioning_angle)
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

#ifdef DEBUG
    VoronoiUtils::debugOutput("output/vd.svg", vd, points, segments);
#endif

    for (const vd_t::edge_type& edge : vd.edges())
    {
        assert(edge.vertex0() == edge.twin()->vertex1());
        assert(edge.vertex1() == edge.twin()->vertex0());
        assert(edge.vertex1() == edge.next()->vertex0());
        assert(edge.vertex0() == edge.prev()->vertex1());
    }

    
    for (vd_t::cell_type cell : vd.cells())
    {
        if (!cell.incident_edge())
        { // there is no spoon
            continue;
        }
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
            transfer_edge(v1, v2, *vd_edge, prev_edge, start_source_point, end_source_point, points, segments);

            make_rib(prev_edge, start_source_point, end_source_point, vd_edge->next() == ending_vd_edge);
        }

        transfer_edge(VoronoiUtils::p(ending_vd_edge->vertex0()), end_source_point, *ending_vd_edge, prev_edge, start_source_point, end_source_point, points, segments);
        // ending_edge->next = nullptr;
        prev_edge->to->data.distance_to_boundary = 0;

        debugCheckGraphConsistency(true);
    }

    
    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
    debugCheckGraphExistance();

    separatePointyQuadEndNodes();

    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
    debugCheckGraphExistance();

    fixNodeDuplication();

    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
    debugCheckGraphExistance();
    debugCheckEndpointUniqueness();
    
    removeZeroLengthSegments();


    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
    debugCheckGraphExistance();

    { // set [some_edge] the the first possible edge
        // that way we can iterate over all reachable edges from node.some_edge without needing to iterate backward
        for (edge_t& edge : graph.edges)
        {
            if (!edge.prev)
            {
                edge.from->some_edge = &edge;
            }
        }
    }


    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
    debugCheckGraphStructure();
    debugCheckGraphReachability();
    debugCheckGraphExistance();
    
#ifdef DEBUG
    {
        AABB aabb(polys);
        SVG svg("output/graph.svg", aabb);
        debugOutput(svg, false, true); 
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph2.svg", aabb);
        debugOutput(svg, false, false, true);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph3.svg", aabb);
        debugOutput(svg, false, false, false, true);
    }
    debugCheckGraphCompleteness();
    debugCheckGraphConsistency();
#endif

    vd_edge_to_he_edge.clear();
    vd_node_to_he_node.clear();
}

void VoronoiQuadrangulation::separatePointyQuadEndNodes()
{
    std::unordered_set<node_t*> visited_nodes;
    for (edge_t& edge : graph.edges)
    {
        if (edge.prev) continue;
        edge_t* quad_start = &edge;
        if (visited_nodes.find(quad_start->from) == visited_nodes.end())
        {
            visited_nodes.emplace(quad_start->from);
        }
        else
        { // needs to be duplicated
            graph.nodes.emplace_back(*quad_start->from);
            node_t* new_node = &graph.nodes.back();
            new_node->some_edge = quad_start;
            quad_start->from = new_node;
            quad_start->twin->to = new_node;
        }
    }

    debugCheckEndpointUniqueness();
}

void VoronoiQuadrangulation::removeZeroLengthSegments()
{
    auto safelyRemoveEdge = [this](edge_t* to_be_removed, std::list<edge_t>::iterator& current_edge_it, bool& edge_it_is_updated)
        {
            if (current_edge_it != graph.edges.end()
                && to_be_removed == &*current_edge_it)
            {
                current_edge_it = graph.edges.erase(current_edge_it);
                edge_it_is_updated = true;
            }
            else
            {
                graph.edges.remove(*to_be_removed);
            }
        };

    for (auto edge_it = graph.edges.begin(); edge_it != graph.edges.end();)
    {
        if (edge_it->prev)
        {
            edge_it++;
            continue;
        }
        edge_t* quad_start = &*edge_it;
        edge_t* quad_end = quad_start; while (quad_end->next) quad_end = quad_end->next;
        edge_t* quad_mid = (quad_start->next == quad_end)? nullptr : quad_start->next;

        bool edge_it_is_updated = false;
        bool quad_mid_is_removed = false;
        if (quad_mid && quad_mid->from->p == quad_mid->to->p)
        {
            assert(quad_mid->twin);
            int count = 0;
            for (edge_t* edge_from_3 = quad_end; edge_from_3 && edge_from_3 != quad_mid->twin; edge_from_3 = edge_from_3->twin->next)
            {
                edge_from_3->from = quad_mid->from;
                edge_from_3->twin->to = quad_mid->from;
                if (count > 50)
                {
                    std::cerr << edge_from_3->from->p << " - " << edge_from_3->to->p << '\n';
                }
                assert(++count < 100);
                if (count > 1000) break;
            }
            if (quad_mid->from->some_edge == quad_mid)
            {
                if (quad_mid->twin->next)
                {
                    quad_mid->from->some_edge = quad_mid->twin->next;
                }
                else
                {
                    quad_mid->from->some_edge = quad_mid->prev->twin;
                }
            }
//             if (quad_mid->twin->from->some_edge == quad_mid->twin)
//             {
//                 quad_mid->twin->from->some_edge = quad_mid->next;
//             }
            graph.nodes.remove(*quad_mid->to);

            quad_mid->prev->next = quad_mid->next;
            quad_mid->next->prev = quad_mid->prev;
            quad_mid->twin->next->prev = quad_mid->twin->prev;
            quad_mid->twin->prev->next = quad_mid->twin->next;

            safelyRemoveEdge(quad_mid, edge_it, edge_it_is_updated);
            safelyRemoveEdge(quad_mid->twin, edge_it, edge_it_is_updated);
            quad_mid_is_removed = true;
        }

        if (quad_start->from->p == quad_end->to->p
            && quad_start->to->p == quad_end->from->p)
        { // collapse start and end edges and remove whole cell
            assert(!quad_mid || quad_mid_is_removed);

            quad_start->twin->to = quad_end->to;
            quad_end->to->some_edge = quad_end->twin;
            if (quad_end->from->some_edge == quad_end)
            {
                if (quad_end->twin->next)
                {
                    quad_end->from->some_edge = quad_end->twin->next;
                }
                else
                {
                    quad_end->from->some_edge = quad_end->prev->twin;
                }
            }
            graph.nodes.remove(*quad_start->from);

            quad_start->twin->twin = quad_end->twin;
            quad_end->twin->twin = quad_start->twin;
            safelyRemoveEdge(quad_start, edge_it, edge_it_is_updated);
            safelyRemoveEdge(quad_end, edge_it, edge_it_is_updated);
        }
        else
        {
            if (quad_start->from->p == quad_start->to->p)
            {
                if (quad_start->next)
                {
                    quad_start->next->prev = nullptr; assert(quad_start->prev == nullptr);
                }
                if (quad_start->twin && quad_start->twin->prev)
                {
                    quad_start->twin->prev->next = nullptr; assert(quad_start->twin->next == nullptr);
                }

                safelyRemoveEdge(quad_start, edge_it, edge_it_is_updated);
                safelyRemoveEdge(quad_start->twin, edge_it, edge_it_is_updated);
                graph.nodes.remove(*quad_start->from);
                quad_start->to->data.distance_to_boundary = 0; // might be slightly higher due to rounding errors
            }
            if (quad_end->from->p == quad_end->to->p)
            {
                quad_end->prev->next = nullptr; assert(quad_end->next == nullptr);
                quad_end->twin->next->prev = nullptr; assert(quad_end->twin->prev == nullptr);

                safelyRemoveEdge(quad_end, edge_it, edge_it_is_updated);
                safelyRemoveEdge(quad_end->twin, edge_it, edge_it_is_updated);
                graph.nodes.remove(*quad_end->to);
                quad_end->from->data.distance_to_boundary = 0; // might be slightly higher due to rounding errors
            }
        }
        if (!edge_it_is_updated)
        {
            edge_it++;
        }
    }
}

void VoronoiQuadrangulation::fixNodeDuplication()
{ // fix duplicate verts
    for (auto node_it = graph.nodes.begin(); node_it != graph.nodes.end();)
    {
        node_t* replacing_node = nullptr;
        for (edge_t* outgoing = node_it->some_edge; outgoing != node_it->some_edge; outgoing = outgoing->twin->next)
        {
            assert(outgoing);
            if (outgoing->from != &*node_it)
            {
                replacing_node = outgoing->from;
            }
            if (outgoing->twin->to != &*node_it)
            {
                replacing_node = outgoing->twin->to;
            }
        }
        if (replacing_node)
        {
            for (edge_t* outgoing = node_it->some_edge; outgoing != node_it->some_edge; outgoing = outgoing->twin->next)
            {
                outgoing->twin->to = replacing_node;
                outgoing->from = replacing_node;
            }
            node_it = graph.nodes.erase(node_it);
        }
        else
        {
            ++node_it;
        }
    }
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

std::vector<ExtrusionSegment> VoronoiQuadrangulation::generateToolpaths(const BeadingStrategy& beading_strategy)
{
    setMarking(beading_strategy);

//     filterMarking(marking_filter_dist);

        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

    setBeadCount(beading_strategy);

    {
        SVG svg("output/unfiltered.svg", AABB(polys));
        debugOutput(svg, false, false, true, false);
    }
    
    
    filterUnmarkedRegions(beading_strategy);

    debugCheckDecorationConsistency(false);

    {
        SVG svg("output/filtered.svg", AABB(polys));
        debugOutput(svg, false, false, true, false);
    }
    
    generateTransitioningRibs(beading_strategy);
    
    

#ifdef DEBUG
    {
        AABB aabb(polys);
        SVG svg("output/graph.svg", aabb);
        debugOutput(svg, false, true);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph2.svg", aabb);
        debugOutput(svg, false, false, true);
    }
    {
        AABB aabb(polys);
        SVG svg("output/graph3.svg", aabb);
        debugOutput(svg, false, false, false, true);
    }
#endif // DEBUG

    debugCheckDecorationConsistency(true);

    std::vector<ExtrusionSegment> segments;
    generateSegments(segments, beading_strategy);
    // junctions = generateJunctions

    printf("got %zu toolpath segments\n", segments.size());

    return segments;
}

void VoronoiQuadrangulation::setMarking(const BeadingStrategy& beading_strategy)
{
    //                                            _.-'^`      .
    //                                      _.-'^`            .
    //                                _.-'^` \                .
    //                          _.-'^`        \               .
    //                    _.-'^`               \ R2           .
    //              _.-'^` \              _.-'\`\             .
    //        _.-'^`        \R1     _.-'^`     '`\ dR         .
    //  _.-'^`a/2            \_.-'^`a             \           .
    //  `^'-._````````````````A```````````v````````B```````   .
    //        `^'-._                     dD = |AB|            .
    //              `^'-._                                    .
    //                             sin a = dR / dD            .
    float cap = 1.0 / sin(beading_strategy.transitioning_angle * 0.5);
    for (edge_t& edge : graph.edges)
    {
        assert(edge.twin);
        if (edge.twin->data.markingIsSet())
        {
            edge.data.setMarked(edge.twin->data.isMarked());
        }
        else if (edge.data.type == VoronoiQuadrangulationEdge::EXTRA_VD)
        {
            edge.data.setMarked(false);
        }
        else
        {
            Point a = edge.from->p;
            Point b = edge.to->p;
            Point ab = b - a;
            coord_t dR = std::abs(edge.to->data.distance_to_boundary - edge.from->data.distance_to_boundary);
            coord_t dD = vSize(ab);
            edge.data.setMarked(dD > cap * dR);
        }
    }
}


void VoronoiQuadrangulation::filterMarking(coord_t max_length)
{
    for (edge_t& edge : graph.edges)
    {
        if (isEndOfMarking(edge))
        {
            filterMarking(edge.twin, 0, max_length);
        }
    }
}

bool VoronoiQuadrangulation::filterMarking(edge_t* starting_edge, coord_t traveled_dist, coord_t max_length)
{
    coord_t length = vSize(starting_edge->from->p - starting_edge->to->p);
    if (traveled_dist + length > max_length)
    {
        return false;
    }
    bool should_dissolve = true;
    for (edge_t* next_edge = starting_edge->next; next_edge && next_edge != starting_edge->twin; next_edge = next_edge->twin->next)
    {
        if (next_edge->data.isMarked())
        {
            should_dissolve &= filterMarking(next_edge, traveled_dist + length, max_length);
        }
    }
    if (should_dissolve)
    {
        starting_edge->data.setMarked(false);
        starting_edge->twin->data.setMarked(false);
    }
    return should_dissolve;
}

void VoronoiQuadrangulation::setBeadCount(const BeadingStrategy& beading_strategy)
{
    for (edge_t& edge : graph.edges)
    {
        if (edge.data.isMarked())
        {
            edge.to->data.bead_count = beading_strategy.optimal_bead_count(edge.to->data.distance_to_boundary * 2);
        }
    }

    // fix bead count at locally maximal R
    // also for marked regions!! See TODOs in generateTransitionEnd(.)
    for (node_t& node : graph.nodes)
    {
        if (isLocalMaximum(node))
        {
            if (node.data.distance_to_boundary < 0)
            {
                RUN_ONCE(logWarning("Distance to boundary not yet computed for local maximum!\n"));
                node.data.distance_to_boundary = std::numeric_limits<coord_t>::max();
                bool first = true;
                for (edge_t* edge = node.some_edge; first || edge != node.some_edge; edge = edge->twin->next)
                {
                    node.data.distance_to_boundary = std::min(node.data.distance_to_boundary, edge->to->data.distance_to_boundary + vSize(edge->from->p - edge->to->p));
                }
            }
            coord_t bead_count = beading_strategy.optimal_bead_count(node.data.distance_to_boundary * 2);
            node.data.bead_count = bead_count;
        }
    }
}

void VoronoiQuadrangulation:: filterUnmarkedRegions(const BeadingStrategy& beading_strategy)
{
    for (edge_t& edge : graph.edges)
    {
        if (!isEndOfMarking(edge))
        {
            continue;
        }
        if (edge.to->data.bead_count < 0 && edge.to->data.distance_to_boundary > 0)
            isEndOfMarking(edge);
        assert(edge.to->data.bead_count >= 0 || edge.to->data.distance_to_boundary == 0);
        coord_t max_dist = 400; // beading_strategy.getTransitioningLength(edge.to->data.bead_count)
        filterUnmarkedRegions(&edge, edge.to->data.bead_count, 0, max_dist, beading_strategy);
    }
}

bool VoronoiQuadrangulation::filterUnmarkedRegions(edge_t* to_edge, coord_t bead_count, coord_t traveled_dist, coord_t max_dist, const BeadingStrategy& beading_strategy)
{
    coord_t r = to_edge->to->data.distance_to_boundary;
    bool dissolve = false;
    for (edge_t* next_edge = to_edge->next; next_edge && next_edge != to_edge->twin; next_edge = next_edge->twin->next)
    {
        coord_t length = vSize(next_edge->to->p - next_edge->from->p);
        if (next_edge->to->data.distance_to_boundary < r && !shorterThen(next_edge->to->p - next_edge->from->p, 10))
        { // only walk upward
            continue;
        }
        if (next_edge->to->data.bead_count == bead_count)
        {
            dissolve = true;
        }
        else if (next_edge->to->data.bead_count < 0)
        {
            dissolve = filterUnmarkedRegions(next_edge, bead_count, traveled_dist + length, max_dist, beading_strategy);
        }
        else // upward bead count is different
        {
            // dissolve if two marked regions with different bead count are closer together than the max_dist (= transition distance)
            dissolve = (traveled_dist + length < max_dist);
        }
        if (dissolve)
        {
            next_edge->data.setMarked(true);
            next_edge->twin->data.setMarked(true);
            next_edge->to->data.bead_count = beading_strategy.optimal_bead_count(next_edge->to->data.distance_to_boundary * 2);
            next_edge->to->data.transition_ratio = 0;
        }
        return dissolve; // dissolving only depend on the one edge going upward. There cannot be multiple edges going upward.
    }
    return dissolve;
}

void VoronoiQuadrangulation::generateTransitioningRibs(const BeadingStrategy& beading_strategy)
{
        debugCheckGraphCompleteness();

    std::unordered_map<edge_t*, std::list<TransitionMiddle>> edge_to_transitions; // maps the upward edge to the transitions. WE only map the halfedge for which the distance_to_boundary is higher at the end than at the beginning
    generateTransitionMids(beading_strategy, edge_to_transitions);

    for (edge_t& edge : graph.edges)
    { // check if there is a transition in between nodes with different bead counts
        if (edge.data.isMarked() && edge.from->data.bead_count != edge.to->data.bead_count)
            assert(edge_to_transitions.find(&edge) != edge_to_transitions.end()
                || edge_to_transitions.find(edge.twin) != edge_to_transitions.end() );
    }
    
        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

#ifdef DEBUG
    {
        SVG svg("output/transition_mids_unfiltered.svg", AABB(polys));
        debugOutput(svg, false, false, true, false);
        for (auto pair : edge_to_transitions)
        {
            edge_t* edge = pair.first;
            Point a = edge->from->p;
            Point b = edge->to->p;
            Point ab = b - a;
            coord_t ab_length = vSize(ab);
            for (TransitionMiddle& transition : pair.second)
            {
                Point p = a + ab * transition.pos / ab_length;
                svg.writePoint(p, false, 3, SVG::Color::MAGENTA);
                std::ostringstream ss;
                ss << transition.lower_bead_count;
                svg.writeText(p, ss.str(), SVG::Color::GRAY);
            }
        }
    }
#endif

    filterTransitionMids(edge_to_transitions, beading_strategy);

#ifdef DEBUG
    {
        SVG svg("output/transition_mids.svg", AABB(polys));
        debugOutput(svg, false, false, true, false);
        for (auto pair : edge_to_transitions)
        {
            edge_t* edge = pair.first;
            Point a = edge->from->p;
            Point b = edge->to->p;
            Point ab = b - a;
            coord_t ab_length = vSize(ab);
            for (TransitionMiddle& transition : pair.second)
            {
                Point p = a + ab * transition.pos / ab_length;
                svg.writePoint(p, false, 3, SVG::Color::MAGENTA);
                std::ostringstream ss;
                ss << transition.lower_bead_count;
                svg.writeText(p, ss.str(), SVG::Color::GRAY);
            }
        }
    }
#endif

    debugCheckTransitionMids(edge_to_transitions);

    std::unordered_map<edge_t*, std::list<TransitionEnd>> edge_to_transition_ends; // we only map the half edge in the upward direction. mapped items are not sorted
    generateTransitionEnds(beading_strategy, edge_to_transitions, edge_to_transition_ends);

#ifdef DEBUG
    {
        SVG svg("output/transition_ends.svg", AABB(polys));
        debugOutput(svg, false, false, true, false);
        for (auto pair : edge_to_transition_ends)
        {
            edge_t* edge = pair.first;
            Point a = edge->from->p;
            Point b = edge->to->p;
            Point ab = b - a;
            coord_t ab_length = vSize(ab);
            for (TransitionEnd& transition : pair.second)
            {
                Point p = a + ab * transition.pos / ab_length;
                svg.writePoint(p, false, 3, transition.is_lower_end? SVG::Color::MAGENTA : SVG::Color::RED);
                std::ostringstream ss;
                ss << transition.lower_bead_count;
                svg.writeText(p, ss.str(), SVG::Color::GRAY);
            }
        }
        for (auto pair : edge_to_transitions)
        {
            edge_t* edge = pair.first;
            Point a = edge->from->p;
            Point b = edge->to->p;
            Point ab = b - a;
            coord_t ab_length = vSize(ab);
            for (TransitionMiddle& transition : pair.second)
            {
                Point p = a + ab * transition.pos / ab_length;
                svg.writePoint(p, false, 3, SVG::Color::GREEN);
                std::ostringstream ss;
                ss << transition.lower_bead_count;
                svg.writeText(p, ss.str(), SVG::Color::GRAY);
            }
        }
    }
#endif

    applyTransitions(edge_to_transition_ends);
}


void VoronoiQuadrangulation::generateTransitionMids(const BeadingStrategy& beading_strategy, std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions)
{
    for (edge_t& edge : graph.edges)
    {
        assert(edge.data.markingIsSet());
        if (!edge.data.isMarked())
        { // only marked regions introduce transitions
            continue;
        }
        coord_t start_R = edge.from->data.distance_to_boundary;
        coord_t end_R = edge.to->data.distance_to_boundary;
        coord_t start_bead_count = edge.from->data.bead_count;
        coord_t end_bead_count = edge.to->data.bead_count;

        if (start_R == end_R)
        { // no transitions occur when both end points have the same distance_to_boundary
            assert(edge.from->data.bead_count == edge.to->data.bead_count);// TODO: what to do in this case?
            continue;
        }
        else if (start_R > end_R)
        { // only consider those half-edges which are going from a lower to a higher distance_to_boundary
            continue;
        }

        if (edge.from->data.bead_count == edge.to->data.bead_count)
        { // no transitions should accur according to the enforced bead counts
            continue;
        }

        if (start_bead_count > beading_strategy.optimal_bead_count(start_R * 2)
            || end_bead_count > beading_strategy.optimal_bead_count(end_R * 2))
        { // wasn't the case earlier in this function because of already introduced transitions
            RUN_ONCE(logError("transitioning segment overlap! (?)\n"));
        }
        assert(start_R < end_R);
        coord_t edge_size = vSize(edge.from->p - edge.to->p);
        for (coord_t transition_lower_bead_count = start_bead_count; transition_lower_bead_count < end_bead_count; transition_lower_bead_count++)
        {
            coord_t mid_R = beading_strategy.transition_thickness(transition_lower_bead_count) / 2;
            if (mid_R > end_R)
            {
                RUN_ONCE(logError("transition on segment lies outside of segment!\n"));
                mid_R = end_R;
            }
            if (mid_R < start_R)
            {
                RUN_ONCE(logError("transition on segment lies outside of segment!\n"));
                mid_R = start_R;
            }
            coord_t mid_pos = edge_size * (mid_R - start_R) / (end_R - start_R);
            assert(mid_pos >= 0);
            assert(mid_pos <= edge_size);
            assert(edge_to_transitions[&edge].empty() || mid_pos >= edge_to_transitions[&edge].back().pos);
            edge_to_transitions[&edge].emplace_back(mid_pos, transition_lower_bead_count);
        }
        if (edge.from->data.bead_count != edge.to->data.bead_count)
        {
            assert(edge_to_transitions[&edge].size() >= 1);
        }
    }
}

void VoronoiQuadrangulation::filterTransitionMids(std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions, const BeadingStrategy& beading_strategy)
{
    for (auto pair_it = edge_to_transitions.begin(); pair_it != edge_to_transitions.end();)
    {
        std::pair<edge_t* const, std::list<TransitionMiddle>>& pair = *pair_it;
        edge_t* edge = pair.first;
        std::list<TransitionMiddle>& transitions = pair.second;
        if (transitions.empty())
        {
            pair_it = edge_to_transitions.erase(pair_it);
            continue;
        }
        assert(transitions.front().lower_bead_count <= transitions.back().lower_bead_count); // this is how stuff should be stored in edge_to_transitions
        assert(edge->from->data.distance_to_boundary <= edge->to->data.distance_to_boundary); // this is how stuff should be stored in edge_to_transitions
        Point a = edge->from->p;
        Point b = edge->to->p;
        Point ab = b - a;
        coord_t ab_size = vSize(ab);
        bool going_up = true;
        std::list<TransitionMidRef> to_be_dissolved_back = dissolveNearbyTransitions(edge, transitions.back(), ab_size - transitions.back().pos, transition_filter_dist, going_up, edge_to_transitions, beading_strategy);
        bool should_dissolve_back = !to_be_dissolved_back.empty();
        for (TransitionMidRef& ref : to_be_dissolved_back)
        {
            dissolveBeadCountRegion(edge, transitions.back().lower_bead_count + 1, transitions.back().lower_bead_count);
            if (ref.pair_it->second.size() <= 1)
            {
                edge_to_transitions.erase(ref.pair_it);
            }
            else
            {
                ref.pair_it->second.erase(ref.transition_it);
            }
        }
        should_dissolve_back |= filterEndOfMarkingTransition(edge, ab_size - transitions.back().pos, beading_strategy.getTransitioningLength(transitions.back().lower_bead_count), transitions.back().lower_bead_count, beading_strategy);
        if (should_dissolve_back)
        {
            transitions.pop_back();
        }
        if (transitions.empty())
        { // filterEndOfMarkingTransition gives inconsistent new bead count when executing for the same transition in two directions.
            pair_it = edge_to_transitions.erase(pair_it);
            continue;
        }
        going_up = false;
        std::list<TransitionMidRef> to_be_dissolved_front = dissolveNearbyTransitions(edge->twin, transitions.front(), transitions.front().pos, transition_filter_dist, going_up, edge_to_transitions, beading_strategy);
        bool should_dissolve_front = !to_be_dissolved_front.empty();
        for (TransitionMidRef& ref : to_be_dissolved_front)
        {
            dissolveBeadCountRegion(edge->twin, transitions.front().lower_bead_count, transitions.front().lower_bead_count + 1);
            if (ref.pair_it->second.size() <= 1)
            {
                edge_to_transitions.erase(ref.pair_it);
            }
            else
            {
                ref.pair_it->second.erase(ref.transition_it);
            }
        }
        should_dissolve_front |= filterEndOfMarkingTransition(edge->twin, transitions.front().pos, beading_strategy.getTransitioningLength(transitions.front().lower_bead_count), transitions.front().lower_bead_count + 1, beading_strategy);
        if (should_dissolve_front)
        {
            transitions.pop_front();
        }
        ++pair_it; // normal update of loop
    }
}

std::list<VoronoiQuadrangulation::TransitionMidRef> VoronoiQuadrangulation::dissolveNearbyTransitions(edge_t* edge_to_start, TransitionMiddle& origin_transition, coord_t traveled_dist, coord_t max_dist, bool going_up, std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions, const BeadingStrategy& beading_strategy)
{
    std::list<TransitionMidRef> to_be_dissolved;
    if (traveled_dist > max_dist)
    {
        return to_be_dissolved;
    }
    bool should_dissolve = true;
    for (edge_t* edge = edge_to_start->next; edge && edge != edge_to_start->twin; edge = edge->twin->next)
    {
        if (!edge->data.isMarked())
        {
            continue;
        }
        Point a = edge->from->p;
        Point b = edge->to->p;
        Point ab = b - a;
        coord_t ab_size = vSize(ab);
        bool is_aligned = edge->from->data.distance_to_boundary < edge->to->data.distance_to_boundary;
        edge_t* aligned_edge = is_aligned? edge : edge->twin;
        bool seen_transition_in_this_direction = false;
        auto edge_transitions_it = edge_to_transitions.find(aligned_edge);
        if (edge_transitions_it != edge_to_transitions.end())
        {
            std::list<TransitionMiddle>& transitions = edge_transitions_it->second;
            for (auto transition_it = transitions.begin(); transition_it != transitions.end(); ++ transition_it)
            { // note: this is not neccesarily iterating in the traveling direction!
                // check whether we should dissolve
                coord_t pos = is_aligned? transition_it->pos : ab_size - transition_it->pos;
                if (traveled_dist + pos < max_dist
                    && transition_it->lower_bead_count == origin_transition.lower_bead_count) // only dissolve local optima
                {
                    if (traveled_dist + pos < beading_strategy.getTransitioningLength(transition_it->lower_bead_count))
                    {
                        assert(going_up != is_aligned || transition_it->lower_bead_count == 0); // consecutive transitions both in/decreasing in bead count should never be closer together than the transition distance
                    }
                    to_be_dissolved.emplace_back(edge_transitions_it, transition_it);
                    seen_transition_in_this_direction = true;
                }
            }
        }
        if (!seen_transition_in_this_direction) // stop recursion once we have found the other transition to be dissolved
        {
            std::list<VoronoiQuadrangulation::TransitionMidRef> to_be_dissolved_here = dissolveNearbyTransitions(edge, origin_transition, traveled_dist + ab_size, max_dist, going_up, edge_to_transitions, beading_strategy);
            if (to_be_dissolved_here.empty())
            { // the region is too long to be dissolved in this direction, so it cannot be dissolved in any direction.
                to_be_dissolved.clear();
                return to_be_dissolved;
            }
            to_be_dissolved.splice(to_be_dissolved.end(), to_be_dissolved_here); // transfer to_be_dissolved_here into to_be_dissolved
            should_dissolve = should_dissolve && !to_be_dissolved.empty();
        }
    }
    if (!should_dissolve)
    {
        to_be_dissolved.clear();
    }
    return to_be_dissolved;
}


void VoronoiQuadrangulation::dissolveBeadCountRegion(edge_t* edge_to_start, coord_t from_bead_count, coord_t to_bead_count)
{
    if (edge_to_start->to->data.bead_count != from_bead_count)
    {
        return;
    }
    edge_to_start->to->data.bead_count = to_bead_count;
    for (edge_t* edge = edge_to_start->next; edge && edge != edge_to_start->twin; edge = edge->twin->next)
    {
        if (!edge->data.isMarked())
        {
            continue;
        }
        dissolveBeadCountRegion(edge, from_bead_count, to_bead_count);
    }
}

bool VoronoiQuadrangulation::filterEndOfMarkingTransition(edge_t* edge_to_start, coord_t traveled_dist, coord_t max_dist, coord_t replacing_bead_count, const BeadingStrategy& beading_strategy)
{
    if (traveled_dist > max_dist)
    {
        return false;
    }
    bool is_end_of_marking = true;
    bool should_dissolve = false;
    for (edge_t* next_edge = edge_to_start->next; next_edge && next_edge != edge_to_start->twin; next_edge = next_edge->twin->next)
    {
        if (next_edge->data.isMarked())
        {
            coord_t length = vSize(next_edge->to->p - next_edge->from->p);
            should_dissolve |= filterEndOfMarkingTransition(next_edge, traveled_dist + length, max_dist, replacing_bead_count, beading_strategy);
            is_end_of_marking = false;
        }
    }
    if (is_end_of_marking && traveled_dist < max_dist)
    {
        should_dissolve = true;
    }
    if (should_dissolve)
    {
        edge_to_start->to->data.bead_count = replacing_bead_count;
    }
    return should_dissolve;
}

void VoronoiQuadrangulation::generateTransitionEnds(const BeadingStrategy& beading_strategy, std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions, std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends)
{
    for (std::pair<edge_t*, std::list<TransitionMiddle>> pair : edge_to_transitions)
    {
        edge_t* edge = pair.first;
        std::list<TransitionMiddle>& transition_positions = pair.second;

        assert(edge->from->data.distance_to_boundary <= edge->to->data.distance_to_boundary);
        for (TransitionMiddle& transition_middle : transition_positions)
        {
            assert(transition_positions.front().pos <= transition_middle.pos);
            assert(transition_middle.pos <= transition_positions.back().pos);
            generateTransition(*edge, transition_middle.pos, beading_strategy, transition_middle.lower_bead_count, edge_to_transition_ends);
        }
    }
}

void VoronoiQuadrangulation::generateTransition(edge_t& edge, coord_t mid_pos, const BeadingStrategy& beading_strategy, coord_t lower_bead_count, std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends)
{
    Point a = edge.from->p;
    Point b = edge.to->p;
    Point ab = b - a;
    coord_t ab_size = vSize(ab);

    coord_t transition_length = beading_strategy.getTransitioningLength(lower_bead_count);
    float transition_mid_position = beading_strategy.getTransitionAnchorPos(lower_bead_count);
    float inner_bead_width_ratio_after_transition = 1.0;

    coord_t start_rest = 0;
    float mid_rest = transition_mid_position * inner_bead_width_ratio_after_transition;
    float end_rest = inner_bead_width_ratio_after_transition;

    
        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();

    { // lower bead count transition end
        coord_t start_pos = ab_size - mid_pos;
        coord_t transition_half_length = transition_mid_position * transition_length;
        coord_t end_pos = start_pos + transition_half_length;
        generateTransitionEnd(*edge.twin, start_pos, end_pos, transition_half_length, mid_rest, start_rest, lower_bead_count, edge_to_transition_ends);
    }
    debugCheckGraphConsistency();
    { // upper bead count transition end
        coord_t start_pos = mid_pos;
        coord_t transition_half_length = (1.0 - transition_mid_position) * transition_length;
        coord_t end_pos = mid_pos +  transition_half_length;
        bool is_going_down_everywhere = generateTransitionEnd(edge, start_pos, end_pos, transition_half_length, mid_rest, end_rest, lower_bead_count, edge_to_transition_ends);
        assert(!is_going_down_everywhere && "There must have been at least one direction in which the bead count is increasing enough for the transition to happen!");
    }

        debugCheckGraphCompleteness();
        debugCheckGraphConsistency();
}

bool VoronoiQuadrangulation::generateTransitionEnd(edge_t& edge, coord_t start_pos, coord_t end_pos, coord_t transition_half_length, float start_rest, float end_rest, coord_t lower_bead_count, std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends)
{
    Point a = edge.from->p;
    Point b = edge.to->p;
    Point ab = b - a;
    coord_t ab_size = vSize(ab); // TODO: prevent recalculation of these values

    assert(start_pos <= ab_size);

    bool going_up = end_rest > start_rest;

    assert(edge.data.isMarked());
    if (!edge.data.isMarked())
    { // This function shouldn't generate ends in or beyond unmarked regions
        return false;
    }

    /*
    if (shorterThen(end_pos - ab_size, snap_dist))
    {
        edge.to->data.transition_rest = 0; // the transition rest should be zero at both sides of the transition; the intermediate values should only occur in between
        edge.data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
        return last_edge_replacing_input;
    }
    else 
    */
    if (end_pos > ab_size)
    { // recurse on all further edges
        coord_t R = edge.to->data.distance_to_boundary;
        float rest = end_rest - (start_rest - end_rest) * (end_pos - ab_size) / (start_pos - end_pos);
        assert(rest >= 0);
        assert(rest <= std::max(end_rest, start_rest));
        assert(rest >= std::min(end_rest, start_rest));
        bool is_only_going_down = true;
        bool has_recursed = false;
        for (edge_t* outgoing = edge.next; outgoing && outgoing != edge.twin;)
        {
            edge_t* next = outgoing->twin->next; // before we change the outgoing edge itself
            if (!outgoing->data.isMarked())
            {
                outgoing = next;
                continue; // don't put transition ends in non-marked regions
            }
            if (going_up && isGoingDown(outgoing, 0, end_pos - ab_size, lower_bead_count))
            { // we're after a 3-way all marked junction node and going in the direction of lower bead count
                // don't introduce a transition end along this marked direction, because this direction is the downward direction
                outgoing = next;
                continue; // don't put transition ends in non-marked regions
            }
            bool is_going_down = generateTransitionEnd(*outgoing, 0, end_pos - ab_size, transition_half_length, rest, end_rest, lower_bead_count, edge_to_transition_ends);
            is_only_going_down &= is_going_down;
            outgoing = next;
            has_recursed = true;
        }
        if (!going_up || (has_recursed && !is_only_going_down))
        {
            edge.to->data.transition_ratio = rest;
            edge.to->data.bead_count = lower_bead_count;
        }
        return is_only_going_down;
    }
    else // end_pos < ab_size
    { // add transition end point here
//         assert(edge.data.isMarked() && "we should only be adding transition ends in marked regions");
        
        bool is_lower_end = end_rest == 0; // TODO collapse this parameter into the bool for which it is used here!
        std::list<TransitionEnd>* transitions = nullptr;
        coord_t pos = -1;
        bool is_aligned = edge.from->data.distance_to_boundary < edge.to->data.distance_to_boundary;
        if (is_aligned)
        {
            transitions = &edge_to_transition_ends[&edge];
            pos = end_pos;
        }
        else
        {
            transitions = &edge_to_transition_ends[edge.twin];
            pos = ab_size - end_pos;
        }
        assert(ab_size == vSize(edge.twin->from->p - edge.twin->to->p));
        assert(pos <= ab_size);
        if (transitions->empty() || pos < transitions->front().pos)
        { // preorder so that sorting later on is faster
            transitions->emplace_front(pos, lower_bead_count, is_lower_end);
        }
        else
        {
            transitions->emplace_back(pos, lower_bead_count, is_lower_end);
        }
        return false;
    }
}


bool VoronoiQuadrangulation::isGoingDown(edge_t* outgoing, coord_t traveled_dist, coord_t max_dist, coord_t lower_bead_count) const
{
    // NOTE: the logic below is not fully thought through.
    // TODO: take transition mids into account
    if (outgoing->to->data.distance_to_boundary == 0)
    {
        return true;
    }
    coord_t length = vSize(outgoing->to->p - outgoing->from->p);
    if (traveled_dist + length > max_dist)
    {
        return false;
    }
    if (outgoing->to->data.bead_count <= lower_bead_count
        && !(outgoing->to->data.bead_count == lower_bead_count && outgoing->to->data.transition_ratio > 0.0))
    {
        return true;
    }
    if (outgoing->to->data.bead_count > lower_bead_count + 1)
    {
        return false;
    }
    bool is_only_going_down = true;
    bool has_recursed = false;
    for (edge_t* next = outgoing->next; next && next != outgoing->twin; next = next->twin->next)
    {
        if (!next->data.isMarked())
        {
            continue;
        }
        bool is_going_down = isGoingDown(next, traveled_dist + length, transition_filter_dist, lower_bead_count);
        is_only_going_down &= is_going_down;
        has_recursed = true;
    }
    return has_recursed && is_only_going_down;
}

void VoronoiQuadrangulation::applyTransitions(std::unordered_map<edge_t*, std::list<TransitionEnd>>& edge_to_transition_ends)
{
    for (std::pair<edge_t* const, std::list<TransitionEnd>>& pair : edge_to_transition_ends)
    {
        edge_t* edge = pair.first;
        auto twin_ends_it = edge_to_transition_ends.find(edge->twin);
        if (twin_ends_it != edge_to_transition_ends.end())
        {
            coord_t length = vSize(edge->from->p - edge->to->p);
            for (TransitionEnd& end : twin_ends_it->second)
            {
                pair.second.emplace_back(length - end.pos, end.lower_bead_count, end.is_lower_end);
            }
            edge_to_transition_ends.erase(twin_ends_it);
        }
    }
    for (std::pair<edge_t* const, std::list<TransitionEnd>>& pair : edge_to_transition_ends)
    {
        edge_t* edge = pair.first;
        assert(edge->data.isMarked());

        std::list<TransitionEnd>& transitions = pair.second;

        transitions.sort([](const TransitionEnd& a, const TransitionEnd& b) { return a.pos < b.pos; } );

        node_t* from = edge->from;
        node_t* to = edge->to;
        Point a = from->p;
        Point b = to->p;
        Point ab = b - a;
        coord_t ab_size = vSize(ab);

        edge_t* last_edge_replacing_input = edge;
        for (TransitionEnd& transition_end : transitions)
        {
            coord_t new_node_bead_count = transition_end.is_lower_end? transition_end.lower_bead_count : transition_end.lower_bead_count + 1;
            coord_t end_pos = transition_end.pos;
            node_t* close_node = (end_pos < ab_size / 2)? from : to;
            if ((end_pos < snap_dist || end_pos > ab_size - snap_dist)
                && close_node->data.bead_count == new_node_bead_count
            )
            {
                assert(end_pos <= ab_size);
//                 close_node->data.bead_count = new_node_bead_count;
                close_node->data.transition_ratio = 0;
                if (!transition_end.is_lower_end)
                {
                    RUN_ONCE(logError("Transition_Mid labeling incorrect!\n"));
                    edge->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
                    edge->twin->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
                }
                continue;
            }
            Point mid = a + normal(ab, end_pos);
            
            debugCheckGraphCompleteness();
            debugCheckGraphConsistency();

            debugCheckDecorationConsistency(false);

            assert(last_edge_replacing_input->data.isMarked());
            assert(last_edge_replacing_input->data.type != VoronoiQuadrangulationEdge::EXTRA_VD);
            last_edge_replacing_input = insertNode(last_edge_replacing_input, mid, new_node_bead_count);
            assert(last_edge_replacing_input->data.type != VoronoiQuadrangulationEdge::EXTRA_VD);
            assert(last_edge_replacing_input->data.isMarked());
            /*
            if (transition_end.is_lower_end)
            {
                last_edge_replacing_input->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
                first_edge_replacing_twin->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
                first_edge_replacing_input->data.type = VoronoiQuadrangulationEdge::NORMAL;
                last_edge_replacing_twin->data.type = VoronoiQuadrangulationEdge::NORMAL;
            }
            else
            {
                last_edge_replacing_input->data.type = VoronoiQuadrangulationEdge::NORMAL;
                first_edge_replacing_twin->data.type = VoronoiQuadrangulationEdge::NORMAL;
                first_edge_replacing_input->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
                last_edge_replacing_twin->data.type = VoronoiQuadrangulationEdge::TRANSITION_MID;
            }
            */

            debugCheckGraphCompleteness();
            debugCheckGraphConsistency();
        }
    }
}

VoronoiQuadrangulation::edge_t* VoronoiQuadrangulation::insertNode(edge_t* edge, Point mid, coord_t mide_node_bead_count)
{
    edge_t* last_edge_replacing_input = edge;

    graph.nodes.emplace_back(VoronoiQuadrangulationJoint(), mid);
    node_t* mid_node = &graph.nodes.back();

    edge_t* twin = last_edge_replacing_input->twin;
    last_edge_replacing_input->twin = nullptr;
    twin->twin = nullptr;
    std::pair<VoronoiQuadrangulation::edge_t*, VoronoiQuadrangulation::edge_t*> left_pair
        = insertRib(*last_edge_replacing_input, mid_node);
    std::pair<VoronoiQuadrangulation::edge_t*, VoronoiQuadrangulation::edge_t*> right_pair
        = insertRib(*twin, mid_node);
    edge_t* first_edge_replacing_input = left_pair.first;
    last_edge_replacing_input = left_pair.second;
    edge_t* first_edge_replacing_twin = right_pair.first;
    edge_t* last_edge_replacing_twin = right_pair.second;

    first_edge_replacing_input->twin = last_edge_replacing_twin;
    last_edge_replacing_twin->twin = first_edge_replacing_input;
    last_edge_replacing_input->twin = first_edge_replacing_twin;
    first_edge_replacing_twin->twin = last_edge_replacing_input;

    mid_node->data.bead_count = mide_node_bead_count;

    return last_edge_replacing_input;
}

std::pair<VoronoiQuadrangulation::edge_t*, VoronoiQuadrangulation::edge_t*> VoronoiQuadrangulation::insertRib(edge_t& edge, node_t* mid_node)
{
    debugCheckGraphConsistency();
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
    mid_node->data.transition_ratio = 0; // both transition end should have rest = 0, because at the ends a whole number of beads fits without rest

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

    first->data.setMarked(true);
    outward_edge->data.setMarked(false); // TODO verify this is always the case.
    inward_edge->data.setMarked(false);
    second->data.setMarked(true);

    outward_edge->twin = inward_edge;
    inward_edge->twin = outward_edge;

    first->twin = nullptr; // we don't know these yet!
    second->twin = nullptr;

    assert(second->prev->from->data.distance_to_boundary == 0);

    debugCheckGraphConsistency();

    return std::make_pair(first, second);
}
std::pair<Point, Point> VoronoiQuadrangulation::getSource(const edge_t& edge)
{
    const edge_t* from_edge;
    for (from_edge = &edge; from_edge->prev; from_edge = from_edge->prev) {}
    const edge_t* to_edge;
    for (to_edge = &edge; to_edge->next; to_edge = to_edge->next) {}
    return std::make_pair(from_edge->from->p, to_edge->to->p);
}

bool VoronoiQuadrangulation::isEndOfMarking(const edge_t& edge_to) const
{
    if (!edge_to.data.isMarked())
    {
        return false;
    }
    if (!edge_to.next)
    {
        return true;
    }
    for (const edge_t* edge = edge_to.next; edge && edge != edge_to.twin; edge = edge->twin->next)
    {
        if (edge->data.isMarked())
        {
            return false;
        }
        assert(edge->twin);
    }
    return true;
}

bool VoronoiQuadrangulation::isLocalMaximum(const node_t& node) const
{
    if (node.data.distance_to_boundary == 0)
    {
        return false;
    }
    bool first = true;
    for (edge_t* edge = node.some_edge; first || edge != node.some_edge; edge = edge->twin->next)
    {
        if (edge->to->data.distance_to_boundary >= node.data.distance_to_boundary)
        {
            return false;
        }
        first = false;
        assert(edge->twin); if (!edge->twin) return false;
        if (!edge->twin->next)
        { // This point is on the boundary
            return false;
        }
    }
    return true;
}

bool VoronoiQuadrangulation::isMarked(const node_t* node) const
{
    bool first = true;
    for (edge_t* edge = node->some_edge; first || edge != node->some_edge; edge = edge->twin->next)
    {
        if (edge->data.isMarked())
        {
            return true;
        }
        first = false;
        assert(edge->twin); if (!edge->twin) return false;
    }
    return false;
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

void VoronoiQuadrangulation::generateSegments(std::vector<ExtrusionSegment>& segments, const BeadingStrategy& beading_strategy)
{
    std::vector<edge_t*> upward_quad_mids;
    for (edge_t& edge : graph.edges)
    {
        if (edge.prev && edge.next && (
            edge.from->data.distance_to_boundary < edge.to->data.distance_to_boundary
            || (edge.from->data.distance_to_boundary == edge.to->data.distance_to_boundary && edge.from->p < edge.to->p)
        ))
        {
            upward_quad_mids.emplace_back(&edge);
        }
    }
    std::sort(upward_quad_mids.begin(), upward_quad_mids.end(), [this](edge_t* a, edge_t* b)
        {
            return a->to->data.distance_to_boundary > b->to->data.distance_to_boundary;
        });
    printf("got %zu cells\n", upward_quad_mids.size());
    
    std::unordered_map<node_t*, BeadingPropagation> node_to_beading;
    { // store beading
        for (node_t& node : graph.nodes)
        {
            if (node.data.bead_count <= 0)
            {
                continue;
            }
            if (node.data.transition_ratio == 0)
            {
                node_to_beading.emplace(&node, beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count));
            }
            else
            {
                Beading low_count_beading = beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count);
                Beading high_count_beading = beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count + 1);
                Beading merged = interpolate(low_count_beading, 1.0 - node.data.transition_ratio, high_count_beading);
                node_to_beading.emplace(&node, merged);

            }
        }
    }
    
    propagateBeadingsUpward(upward_quad_mids, node_to_beading, beading_strategy);

    propagateBeadingsDownward(upward_quad_mids, node_to_beading, beading_strategy);
    
    std::unordered_map<edge_t*, std::vector<ExtrusionJunction>> edge_to_junctions; // junctions ordered high R to low R
    generateJunctions(node_to_beading, edge_to_junctions, beading_strategy);
    
    connectJunctions(edge_to_junctions, segments);
    
    generateLocalMaximaSingleBeads(node_to_beading, segments);
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
    if (!ret->next && ret->to->data.distance_to_boundary - 5 < ret->from->data.distance_to_boundary)
    {
        ret = ret->prev;
    }
    assert(ret);
    assert(ret->next);
    return ret;
}

void VoronoiQuadrangulation::propagateBeadingsUpward(std::vector<edge_t*>& upward_quad_mids, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy)
{
    for (auto upward_quad_mids_it = upward_quad_mids.rbegin(); upward_quad_mids_it != upward_quad_mids.rend(); ++upward_quad_mids_it)
    {
        edge_t* upward_edge = *upward_quad_mids_it;
        if (upward_edge->to->data.bead_count >= 0)
        { // don't override local beading
            continue;
        }
        auto lower_beading_it = node_to_beading.find(upward_edge->from);
        if (lower_beading_it == node_to_beading.end())
        { // only propagate if we have something to propagate
            continue;
        }
        auto upper_beading_it = node_to_beading.find(upward_edge->to);
        if (upper_beading_it != node_to_beading.end())
        { // only propagate to places where there is place
            continue;
        }
        assert(upward_edge->from->data.distance_to_boundary != upward_edge->to->data.distance_to_boundary && "zero difference R edges should always be marked");
        BeadingPropagation& lower_beading = lower_beading_it->second;
        coord_t length = vSize(upward_edge->to->p - upward_edge->from->p);
        BeadingPropagation upper_beading = lower_beading;
        upper_beading.dist_to_bottom_source += length;
        node_to_beading.emplace(upward_edge->to, upper_beading);
    }
}

void VoronoiQuadrangulation::propagateBeadingsDownward(std::vector<edge_t*>& upward_quad_mids, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy)
{
    for (edge_t* upward_quad_mid : upward_quad_mids)
    {
        // transfer beading information to lower nodes
        if (!upward_quad_mid->data.isMarked())
        {
            if (upward_quad_mid->from->data.distance_to_boundary == upward_quad_mid->to->data.distance_to_boundary
                && node_to_beading.find(upward_quad_mid->from) != node_to_beading.end()
                && node_to_beading.find(upward_quad_mid->to) == node_to_beading.end()
            )
            {
                propagateBeadingsDownward(upward_quad_mid->twin, node_to_beading, beading_strategy);
            }
            else
            {
                propagateBeadingsDownward(upward_quad_mid, node_to_beading, beading_strategy);
            }
        }
    }
}

void VoronoiQuadrangulation::propagateBeadingsDownward(edge_t* edge_to_peak, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy)
{
    coord_t length = vSize(edge_to_peak->to->p - edge_to_peak->from->p);
    BeadingPropagation& top_beading = getBeading(edge_to_peak->to, node_to_beading, beading_strategy);
    top_beading.is_finished = true;
    
    auto it = node_to_beading.find(edge_to_peak->from);
    if (it == node_to_beading.end())
    { // set new beading if there is no beading associatied with the node yet
        BeadingPropagation propagated_beading = top_beading;
        propagated_beading.dist_from_top_source += length;
        auto pair = node_to_beading.emplace(edge_to_peak->from, propagated_beading);
        assert(pair.second && "we emplaced something");
    }
    else // if (!it->second.is_finished)
    {
        BeadingPropagation& bottom_beading = it->second;
        coord_t total_dist = top_beading.dist_from_top_source + length + bottom_beading.dist_to_bottom_source;
        float ratio_of_top = static_cast<float>(bottom_beading.dist_to_bottom_source) / std::min(total_dist, beading_propagation_transition_dist);
        ratio_of_top = std::max(0.0f, ratio_of_top);
        if (ratio_of_top >= 1.0)
        {
            bottom_beading = top_beading;
            bottom_beading.dist_from_top_source += length;
        }
        else
        {
            Beading merged_beading = interpolate(top_beading.beading, ratio_of_top, bottom_beading.beading, edge_to_peak->from->data.distance_to_boundary);
            bottom_beading = BeadingPropagation(merged_beading);
        }
    }
}


VoronoiQuadrangulation::Beading VoronoiQuadrangulation::interpolate(const Beading& left, float ratio_left_to_whole, const Beading& right, coord_t switching_radius) const
{
    assert(ratio_left_to_whole >= 0.0 && ratio_left_to_whole <= 1.0);
    Beading ret = interpolate(left, ratio_left_to_whole, right);

    // TODO: don't use toolpath locations past the middle!
    // TODO: stretch bead widths and locations of the higher bead count beading to fit in the left over space
    coord_t next_inset_idx;
    for (next_inset_idx = left.toolpath_locations.size() - 1; next_inset_idx >= 0; next_inset_idx--)
    {
        if (switching_radius > left.toolpath_locations[next_inset_idx])
        {
            break;
        }
    }
    if (next_inset_idx < 0)
    { // there is no next inset, because there is only one
        assert(left.toolpath_locations.front() > switching_radius);
        assert(right.toolpath_locations.size() <= 2);
        return ret;
    }
    assert(next_inset_idx < left.toolpath_locations.size());
    assert(left.toolpath_locations[next_inset_idx] <= switching_radius);
    assert(left.toolpath_locations[next_inset_idx + 1] >= switching_radius);
    if (ret.toolpath_locations[next_inset_idx] > switching_radius)
    { // one inset disappeared between left and the merged one
        // solve for ratio f:
        // f*l + (1-f)*r = s
        // f*l + r - f*r = s
        // f*(l-r) + r = s
        // f*(l-r) = s - r
        // f = (s-r) / (l-r)
        float new_ratio = static_cast<float>(switching_radius - right.toolpath_locations[next_inset_idx]) / static_cast<float>(left.toolpath_locations[next_inset_idx] - right.toolpath_locations[next_inset_idx]);
        new_ratio = std::min(1.0, new_ratio + 0.1);
        return interpolate(left, new_ratio, right);
    }
    return ret;
}


VoronoiQuadrangulation::Beading VoronoiQuadrangulation::interpolate(const Beading& left, float ratio_left_to_whole, const Beading& right) const
{
    assert(ratio_left_to_whole >= 0.0 && ratio_left_to_whole <= 1.0);
    float ratio_right_to_whole = 1.0 - ratio_left_to_whole;

    Beading ret = (left.bead_widths.size() > right.bead_widths.size())? left : right;
    for (int inset_idx = 0; inset_idx < std::min(left.bead_widths.size(), right.bead_widths.size()); inset_idx++)
    {
        ret.bead_widths[inset_idx] = ratio_left_to_whole * left.bead_widths[inset_idx] + ratio_right_to_whole * right.bead_widths[inset_idx];
        ret.toolpath_locations[inset_idx] = ratio_left_to_whole * left.toolpath_locations[inset_idx] + ratio_right_to_whole * right.toolpath_locations[inset_idx];
    }
    return ret;
}

void VoronoiQuadrangulation::generateJunctions(std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, std::unordered_map<edge_t*, std::vector<ExtrusionJunction>>& edge_to_junctions, const BeadingStrategy& beading_strategy)
{
    for (edge_t& edge_ : graph.edges)
    {
        edge_t* edge = &edge_;
        if (edge->from->data.distance_to_boundary > edge->to->data.distance_to_boundary)
        { // only consider the upward half-edges
            continue;
        }

        Beading* beading = &getBeading(edge->to, node_to_beading, beading_strategy).beading;
        std::vector<ExtrusionJunction>& ret = edge_to_junctions[edge]; // emplace a new vector
        if (edge->to->data.bead_count == 0 && edge->from->data.bead_count == 0)
        {
            continue;
        }

        Point a = edge->to->p;
        Point b = edge->from->p;
        Point ab = b - a;

        coord_t start_R = edge->to->data.distance_to_boundary; // higher R
        coord_t end_R = edge->from->data.distance_to_boundary; // lower R
        if (end_R == start_R)
        {
            continue;
        }
        assert(end_R <= start_R);

        coord_t junction_idx;
        // compute starting junction_idx for this segment
        for (junction_idx = (beading->toolpath_locations.size() - 1) / 2; junction_idx >= 0 && junction_idx < beading->toolpath_locations.size(); junction_idx--)
        {
            coord_t bead_R = beading->toolpath_locations[junction_idx];
            if (bead_R <= start_R)
            { // junction coinciding with start node is used in this function call
                break;
            }
        }

        // rebustness against odd segments which might lie just slightly outside of the range due to rounding errors
        // not sure if this is really needed (TODO)
        if (junction_idx + 1 < beading->toolpath_locations.size()
            && beading->toolpath_locations[junction_idx + 1] <= start_R + 5
            && beading->total_thickness < start_R + 5
        )
        {
            junction_idx++;
        }

        for (; junction_idx >= 0 && junction_idx < coord_t(beading->toolpath_locations.size()); junction_idx--)
        {
            coord_t bead_R = beading->toolpath_locations[junction_idx];
            assert(bead_R > 0);
            if (bead_R < end_R)
            { // junction coinciding with a node is handled by the next segment
                break;
            }
            Point junction(a + ab * (bead_R - start_R) / (end_R - start_R));
            ret.emplace_back(junction, beading->bead_widths[junction_idx], junction_idx);
        }
    }
}

const std::vector<ExtrusionJunction>& VoronoiQuadrangulation::getJunctions(edge_t* edge, std::unordered_map<edge_t*, std::vector<ExtrusionJunction>>& edge_to_junctions)
{
    assert(edge->to->data.distance_to_boundary >= edge->from->data.distance_to_boundary);
    auto ret_it = edge_to_junctions.find(edge);
    assert(ret_it != edge_to_junctions.end());
    return ret_it->second;
}


VoronoiQuadrangulation::BeadingPropagation& VoronoiQuadrangulation::getBeading(node_t* node, std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, const BeadingStrategy& beading_strategy)
{
    auto beading_it = node_to_beading.find(node);
    if (beading_it == node_to_beading.end())
    {
        if (node->data.bead_count == -1)
        { // TODO: where does this bug come from?
            bool has_marked_edge = false;
            bool first = true;
            coord_t dist = std::numeric_limits<coord_t>::max();
            for (edge_t* edge = node->some_edge; edge && (first || edge != node->some_edge); edge = edge->twin->next)
            {
                if (edge->data.isMarked())
                {
                    has_marked_edge = true;
                }
                assert(edge->to->data.distance_to_boundary >= 0);
                dist = std::min(dist, edge->to->data.distance_to_boundary + vSize(edge->to->p - edge->from->p));
                first = false;
            }
            RUN_ONCE(logError("Unknown beading for unmarked node!\n"));
//             assert(false);
            assert(dist != std::numeric_limits<coord_t>::max());
            node->data.bead_count = beading_strategy.optimal_bead_count(dist);
        }
        assert(node->data.bead_count != -1);
        beading_it = node_to_beading.emplace(node, beading_strategy.compute(node->data.distance_to_boundary * 2, node->data.bead_count)).first;
    }
    return beading_it->second;
}


void VoronoiQuadrangulation::connectJunctions(std::unordered_map<edge_t*, std::vector<ExtrusionJunction>> edge_to_junctions, std::vector<ExtrusionSegment>& segments)
{
    // TODO: walk along cells in order of the input polygons, so that we can easily greedily optimize the order afterwards
    for (edge_t& edge : graph.edges)
    {
        if (edge.prev) continue;
        edge_t* quad_start = &edge;
        edge_t* quad_end = quad_start; while (quad_end->next) quad_end = quad_end->next;
        edge_t* edge_to_peak = getQuadMaxRedgeTo(quad_start);
        // walk down on both sides and connect junctions
        edge_t* edge_from_peak = edge_to_peak->next; assert(edge_from_peak);
        
        
        
        
        std::vector<ExtrusionJunction> from_junctions = getJunctions(edge_to_peak, edge_to_junctions);
        std::vector<ExtrusionJunction> to_junctions = getJunctions(edge_from_peak->twin, edge_to_junctions);
        if (edge_to_peak->prev)
        {
            std::vector<ExtrusionJunction> from_prev_junctions = getJunctions(edge_to_peak->prev, edge_to_junctions);
            if (!from_junctions.empty() && !from_prev_junctions.empty() && from_junctions.back().perimeter_index == from_prev_junctions.front().perimeter_index)
            {
                from_junctions.pop_back();
            }
            from_junctions.reserve(from_junctions.size() + from_prev_junctions.size());
            from_junctions.insert(from_junctions.end(), from_prev_junctions.begin(), from_prev_junctions.end());
            assert(!edge_to_peak->prev->prev);
        }
        if (edge_from_peak->next)
        {
            std::vector<ExtrusionJunction> to_next_junctions = getJunctions(edge_from_peak->next->twin, edge_to_junctions);
            if (!to_junctions.empty() && !to_next_junctions.empty() && to_junctions.back().perimeter_index == to_next_junctions.front().perimeter_index)
            {
                to_junctions.pop_back();
            }
            to_junctions.reserve(to_junctions.size() + to_next_junctions.size());
            to_junctions.insert(to_junctions.end(), to_next_junctions.begin(), to_next_junctions.end());
            assert(!edge_from_peak->next->next);
        }
        assert(std::abs(int(from_junctions.size()) - int(to_junctions.size())) <= 1); // at transitions one end has more beads
        
        // deal with transition to single bead
//         if (quad_start->next->next && quad_start->next->data.isMarked() && quad_start->to->data.bead_count % 2 + quad_start->next->to->data.bead_count % 2 == 1)
//         {
//             assert(quad_start->next->data.type == VoronoiQuadrangulationEdge::TRANSITION_MID);
//             if (quad_start->to->data.bead_count % 2 == 0)
//             {
                
//         }
        
        size_t segment_count = std::min(from_junctions.size(), to_junctions.size());
        for (size_t junction_rev_idx = 0; junction_rev_idx < segment_count; junction_rev_idx++)
        {
            ExtrusionJunction& from = from_junctions[from_junctions.size() - 1 - junction_rev_idx];
            ExtrusionJunction& to = to_junctions[to_junctions.size() - 1 - junction_rev_idx];
            assert(from.perimeter_index == to.perimeter_index);
            bool is_odd_segment = edge_to_peak->to->data.bead_count > 0 && edge_to_peak->to->data.bead_count % 2 == 1 // quad contains single bead segment
                && edge_to_peak->to->data.transition_ratio == 0 && edge_to_peak->from->data.transition_ratio == 0 && edge_from_peak->to->data.transition_ratio == 0 // we're not in a transition
                && junction_rev_idx == segment_count - 1 // is single bead segment
                && shorterThen(from.p - quad_start->to->p, 5) && shorterThen(to.p - quad_end->from->p, 5);
            if (is_odd_segment
                && from.p < to.p) // choose one
            {
                continue; // prevent duplication of single bead segments
            }
            segments.emplace_back(from, to, is_odd_segment);
        }
    }
}

void VoronoiQuadrangulation::generateLocalMaximaSingleBeads(std::unordered_map<node_t*, BeadingPropagation>& node_to_beading, std::vector<ExtrusionSegment>& segments)
{
    for (auto pair : node_to_beading)
    {
        node_t* node = pair.first;
        Beading& beading = pair.second.beading;
        if (beading.bead_widths.size() % 2 == 1
            && isLocalMaximum(*node)
            && !isMarked(node)
        )
        {
            coord_t inset_index = beading.bead_widths.size() / 2;
            ExtrusionJunction from(node->p, beading.bead_widths[inset_index], inset_index);
            ExtrusionJunction to(node->p + Point(10, 0), beading.bead_widths[inset_index], inset_index);
            segments.emplace_back(from, to, true);
        }
    }
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
#ifdef DEBUG
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
        assert((edge.next == nullptr) == (edge.twin->prev == nullptr));
        assert((edge.prev == nullptr) == (edge.twin->next == nullptr));
        assert(edge.next || edge.to->data.distance_to_boundary == 0);
        assert(edge.prev || edge.from->data.distance_to_boundary == 0);
    }
#endif
}

void VoronoiQuadrangulation::debugCheckEndpointUniqueness()
{
#ifdef DEBUG
    for (edge_t& edge : graph.edges)
    {
        if (edge.prev) continue;
        for (edge_t& e2 : graph.edges)
        {
            assert(e2.from != edge.from || edge == e2);
        }
    }
#endif
}

void VoronoiQuadrangulation::debugCheckGraphExistance()
{
#ifdef DEBUG
    std::unordered_set<edge_t*> all_edges;
    std::unordered_set<node_t*> all_nodes;
    for (node_t& node : graph.nodes)
    {
        all_nodes.emplace(&node);
    }
    for (edge_t& edge : graph.edges)
    {
        all_edges.emplace(&edge);
    }
    
    auto edge_exists = [&all_edges](edge_t* edge)
        {
            assert(edge == nullptr ||
                all_edges.find(edge) != all_edges.end());
        };
    auto node_exists = [&all_nodes](node_t* node)
        {
            assert(node == nullptr ||
                all_nodes.find(node) != all_nodes.end());
        };
    for (node_t& node : graph.nodes)
    {
        edge_exists(node.some_edge);
    }
    for (edge_t& edge : graph.edges)
    {
        edge_t* edge_ = &edge;
        edge_exists(edge.prev);
        edge_exists(edge.next);
        edge_exists(edge.twin);
        node_exists(edge.from);
        node_exists(edge.to);
    }
#endif
}

void VoronoiQuadrangulation::debugCheckGraphStructure()
{
#ifdef DEBUG
    for (edge_t& edge : graph.edges)
    {
        node_t* node = edge.from;
        bool first = true;
        coord_t count = 0;
        bool seen_edge = false;
        for (const edge_t* outgoing = node->some_edge; outgoing && (first || outgoing != node->some_edge); outgoing = outgoing->twin->next)
        {
            assert(++count < 100);
            if (outgoing == &edge) seen_edge = true;
            first = false;
            if (!outgoing->twin) break;
        }
        assert(seen_edge);
    }
#endif
}

void VoronoiQuadrangulation::debugCheckGraphReachability()
{
#ifdef DEBUG
    std::unordered_set<node_t*> reachable_nodes;
    for (edge_t& edge : graph.edges)
    {
        reachable_nodes.emplace(edge.from);
        reachable_nodes.emplace(edge.to);
    }
    for (node_t& node : graph.nodes)
    {
        assert(reachable_nodes.find(&node) != reachable_nodes.end());
    }

    std::unordered_set<edge_t*> reachable_edges;
    for (node_t& node : graph.nodes)
    {
        bool first = true;
        for (edge_t* outgoing = node.some_edge; outgoing && (first || outgoing != node.some_edge); outgoing = outgoing->twin->next)
        {
            reachable_edges.emplace(outgoing);
            first = false;
            if (!outgoing->twin) break;
        }
    }
    for (edge_t& edge : graph.edges)
    {
        edge_t* edge_ = &edge;
        if (reachable_edges.find(&edge) == reachable_edges.end())
        {
            std::cerr << "Cannot find " << edge.from->p << " - " << edge.to->p << " among edges around former!\n";
            bool seen = false;
            bool first = true;
            for (edge_t* outgoing = edge.from->some_edge; outgoing && (first || outgoing != edge.from->some_edge); outgoing = outgoing->twin->next)
            {
                std::cerr << outgoing->to->p << "\n";
                if (outgoing == edge_)
                {
                    seen = true;
                }
                first = false;
                if (!outgoing->twin) break;
            }
            assert(seen);
            assert(std::find(graph.nodes.begin(), graph.nodes.end(), *edge.from) != graph.nodes.end());
        }
        assert(reachable_edges.find(&edge) != reachable_edges.end());
    }
#endif
}

void VoronoiQuadrangulation::debugCheckGraphConsistency(bool ignore_duplication)
{
#ifdef DEBUG
    auto vert_assert = [ignore_duplication](const node_t* first, const node_t* second)
    {
        if (first != second)
        {
            if (first->p == second->p)
            {
                RUN_ONCE(logWarning("Unneccesary duplicatation of VoronoiQuadrangulation nodes!\n"));
                assert(ignore_duplication);
            }
            else
            {
                assert(false && "connected edges don't refer to the same node!");
            }
        }
    };
    
    for (const edge_t& edge : graph.edges)
    {
        const edge_t* edge_p = &edge;
        if (edge_p->twin)
        {
            if (!edge_p->to)
            {
                assert(!edge_p->twin->from);
                vert_assert(edge_p->twin->from, edge_p->to);
            }
            if (!edge_p->from)
            {
                assert(!edge_p->twin->to);
                vert_assert(edge_p->twin->to, edge_p->from);
            }
            assert((edge_p->from == nullptr) == (edge_p->twin->to == nullptr));
            assert((edge_p->to == nullptr) == (edge_p->twin->from == nullptr));
            assert(edge_p->twin->twin == &edge);
            assert(edge_p->twin != edge_p);
        }
        if (edge_p->next)
        {
            vert_assert(edge_p->next->from, edge_p->to);
        }
        if (edge_p->prev)
        {
            vert_assert(edge_p->prev->to, edge_p->from);
        }
    }
    for (const node_t& node : graph.nodes)
    {
        if (node.some_edge)
        {
            const node_t* node_ = &node;
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
#endif // DEBUG
}

void VoronoiQuadrangulation::debugCheckDecorationConsistency(bool transitioned)
{
#ifdef DEBUG
    for (const edge_t& edge : graph.edges)
    {
        const edge_t* edge_p = &edge;
        assert(edge.data.type >= VoronoiQuadrangulationEdge::NORMAL && edge.data.type <= VoronoiQuadrangulationEdge::TRANSITION_MID);
        if (edge.data.type != VoronoiQuadrangulationEdge::NORMAL && edge.data.type != VoronoiQuadrangulationEdge::TRANSITION_MID)
        {
            if (edge.from->data.distance_to_boundary != -1 && edge.to->data.distance_to_boundary != -1)
            {
                assert(edge.from->data.distance_to_boundary == 0 || edge.to->data.distance_to_boundary == 0);
            }
            assert(!edge.data.isMarked());
        }
        assert(edge.data.isMarked() == edge.twin->data.isMarked());
        if (edge.data.isMarked())
        {
            if (transitioned && edge.from->data.bead_count != -1 && edge.to->data.bead_count != -1)
            {
                assert(!edge.data.isMarked() || std::abs(edge.from->data.bead_count - edge.to->data.bead_count) <= 1);
            }
        }
    }
#endif // DEBUG
}

void VoronoiQuadrangulation::debugCheckTransitionMids(const std::unordered_map<edge_t*, std::list<TransitionMiddle>>& edge_to_transitions) const
{
#ifdef DEBUG
    for (std::pair<edge_t*, std::list<TransitionMiddle>> pair : edge_to_transitions)
    {
        const edge_t* edge = pair.first;
        const std::list<TransitionMiddle>& transition_positions = pair.second;
        
        assert(edge->from->data.distance_to_boundary <= edge->to->data.distance_to_boundary);
        
        const TransitionMiddle* prev = nullptr;
        for (const TransitionMiddle& here : transition_positions)
        {
            if (prev)
            {
                assert(here.pos > prev->pos);
                assert(here.lower_bead_count > prev->lower_bead_count);
                assert(std::abs(here.lower_bead_count - prev->lower_bead_count) == 1);
            }
            prev = &here;
        }
        
    }
#endif // DEBUG
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
    svg.writeAreas(polys, SVG::Color::NONE, SVG::Color::GRAY, 3);
    for (edge_t& edge : graph.edges)
    {
        Point a = edge.from->p;
        Point b = edge.to->p;
        SVG::Color clr = getColor(edge);
        float stroke_width = 1;
        if (edge.data.markingIsSet() && edge.data.isMarked() && edge.data.type != VoronoiQuadrangulationEdge::TRANSITION_MID)
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
            if (edge.to->p < edge.from->p)
            {
                svg.writeLine(a, b, clr, stroke_width);
            }
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
            if (node.data.transition_ratio != 0)
            {
                svg.writeText(node.p, std::to_string(node.data.transition_ratio + node.data.bead_count));
            }
            else
            {
                svg.writeText(node.p, std::to_string(node.data.bead_count));
            }
        }
        if (draw_locations)
        {
            svg.writePoint(node.p, false, 1);
            std::ostringstream ss;
            ss << node.p.X << "," << node.p.Y;
            svg.writeText(node.p, ss.str(), SVG::Color::BLACK, 4);
        }
    }
}

} // namespace arachne
