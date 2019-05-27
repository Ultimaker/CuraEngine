//Copyright (c) 2019 Ultimaker B.V.
#include "VoronoiQuadrangulation.h"


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
        
        node_t* v0 = &make_node(*vd_edge.vertex0(), from);
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
    if (!point_left && !point_right
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
        Point p0 = VoronoiUtils::p(vd_edge->vertex0());
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


    debugCheckGraphCompleteness();
}

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

SVG::Color VoronoiQuadrangulation::getColor(edge_t& edge)
{
    switch (edge.data.type)
    {
        case VoronoiQuadrangulationEdge::EXTRA_VD:
            return SVG::Color::ORANGE;
        case VoronoiQuadrangulationEdge::TRANSITION_END:
            return SVG::Color::BLUE;
        case VoronoiQuadrangulationEdge::NORMAL:
        default:
            return SVG::Color::RED;
    }
}

void VoronoiQuadrangulation::debugOutput(SVG& svg, bool draw_arrows, bool draw_dists)
{
    coord_t offset_length = 10;
    for (edge_t& edge : graph.edges)
    {
        Point a = edge.from->p;
        Point b = edge.to->p;
        Point ab = b - a;
        Point n = normal(turn90CCW(ab), offset_length);
        Point d = normal(ab, 3 * offset_length);
        if (draw_arrows)
        {
            svg.writeArrow(a, b, getColor(edge));
        }
        else
        {
            svg.writeLine(a, b, getColor(edge));
        }
    }
    if (draw_dists)
    {
        for (node_t& node : graph.nodes)
        {
            svg.writeText(node.p, std::to_string(node.data.distance_to_boundary));
        }
    }
}


} // namespace arachne
