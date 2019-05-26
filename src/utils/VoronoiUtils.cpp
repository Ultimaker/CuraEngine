//Copyright (c) 2018 Ultimaker B.V.

#include <optional>


#include "VoronoiUtils.h"

#include "linearAlg2D.h"
#include "SVG.h"

using boost::polygon::low;
using boost::polygon::high;

namespace arachne 
{

Point VoronoiUtils::p(const vd_t::vertex_type* node)
{
    return Point(node->x(), node->y());
}

bool VoronoiUtils::isSourcePoint(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments, coord_t snap_dist)
{
    if (cell.contains_point())
    {
        return shorterThen(p - getSourcePoint(cell, points, segments), snap_dist);
    }
    else
    {
        const Segment& segment = getSourceSegment(cell, points, segments);
        return shorterThen(p - segment.from(), snap_dist) || shorterThen(p - segment.to(), snap_dist);
    }
}

coord_t VoronoiUtils::getDistance(Point p, const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    if (cell.contains_point())
    {
        return vSize(p - getSourcePoint(cell, points, segments));
    }
    else
    {
        const Segment& segment = getSourceSegment(cell, points, segments);
        return sqrt(LinearAlg2D::getDist2FromLineSegment(segment.from(), p, segment.to()));
    }
}

Point VoronoiUtils::getSourcePoint(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    assert(cell.contains_point());
    switch (cell.source_category())
    {
    case boost::polygon::SOURCE_CATEGORY_SINGLE_POINT:
        return points[cell.source_index()];
        break;
    case boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT:
        assert(cell.source_index() - points.size() < segments.size());
        return segments[cell.source_index() - points.size()].to();
        break;
    case boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT:
        assert(cell.source_index() - points.size() < segments.size());
        return segments[cell.source_index() - points.size()].from();
        break;
    default:
        assert(false && "getSourcePoint should only be called on point cells!\n");
        break;
    }
}

PolygonsPointIndex VoronoiUtils::getSourcePointIndex(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    assert(cell.contains_point());
    assert(cell.source_category() != boost::polygon::SOURCE_CATEGORY_SINGLE_POINT);
    switch (cell.source_category())
    {
    case boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT:
    {
        assert(cell.source_index() - points.size() < segments.size());
        PolygonsPointIndex ret = segments[cell.source_index() - points.size()];
        ++ret;
        return ret;
        break;
    }
    case boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT:
    {
        assert(cell.source_index() - points.size() < segments.size());
        return segments[cell.source_index() - points.size()];
        break;
    }
    default:
        assert(false && "getSourcePoint should only be called on point cells!\n");
        break;
    }
}

const VoronoiUtils::Segment& VoronoiUtils::getSourceSegment(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    assert(cell.contains_segment());
    return segments[cell.source_index() - points.size()];
}

void VoronoiUtils::debugOutput(std::string filename, voronoi_diagram<voronoi_data_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points, bool show_coords, bool show_parabola_generators, bool draw_arrows)
{
    AABB aabb;
//     for (const voronoi_diagram<voronoi_data_t>::vertex_type& vert : vd.vertices())
//     {
//         aabb.include(Point(vert.x(), vert.y()));
//     }
    for (const Point& p : points)
    {
        aabb.include(p);
    }
    for (const Segment& s : segments)
    {
        aabb.include(s.from());
        aabb.include(s.to());
    }
    SVG svg(filename.c_str(), aabb);
    
    debugOutput(svg, vd, points, segments, draw_points, show_coords, show_parabola_generators, draw_arrows);
}

void VoronoiUtils::debugOutput(SVG& svg, voronoi_diagram<voronoi_data_t>& vd, std::vector<Point>& points, std::vector<Segment>& segments, bool draw_points, bool show_coords, bool show_parabola_generators, bool draw_arrows)
{
    
    for (const Point& p : points)
    {
        svg.writePoint(p, show_coords, 2);
    }
    for (const Segment& s : segments)
    {
        svg.writeLine(s.from(), s.to(), SVG::Color::BLACK, 2);
        if (draw_points) svg.writePoint(s.from(), show_coords, 2);
        if (draw_points) svg.writePoint(s.to(), show_coords, 2);
    }
    
    
    printf("%zu edges\n", vd.edges().size());
    
    for (const vd_t::edge_type& edge : vd.edges())
    {
        const vd_t::vertex_type* from = edge.vertex0();
        const vd_t::vertex_type* to = edge.vertex1();
        if (from && to)
        {
            Point from_(edge.vertex0()->x(), edge.vertex0()->y());
            Point to_(edge.vertex1()->x(), edge.vertex1()->y());
//             printf("(%lld,%lld)-(%lld,%lld)\n", from_.X, from_.Y, to_.X, to_.Y);
//             if (from_.X +from_.Y < to_.X + to_.Y) continue; // only process half of the half-edges
            if (edge.is_linear())
            {
                SVG::Color clr = (edge.is_primary())? SVG::Color::RED : SVG::Color::GREEN;
                svg.writeArrow(Point(from->x(), from->y()), Point(to->x(), to->y()), clr, 1, draw_arrows? 20 : 1000, draw_arrows? 20 : 0);
            }
            else
            {
                const vd_t::cell_type& left_cell = *edge.cell();
                const vd_t::cell_type& right_cell = *edge.twin()->cell();
                
                assert(left_cell.contains_point() == right_cell.contains_segment());
                const vd_t::cell_type& segment_cell = (left_cell.contains_segment())? left_cell : right_cell;
                const vd_t::cell_type& point_cell = (left_cell.contains_point())? left_cell : right_cell;
                
                Point point = getSourcePoint(point_cell, points, segments);
                const Segment& segment = getSourceSegment(segment_cell, points, segments);
                
                Point mid;
                Point s = segment.to() - segment.from();
                if ((dot(from_, s) < dot(point, s)) == (dot(to_, s) < dot(point, s)))
                {
                    svg.writeArrow(from_, to_, SVG::Color::BLUE, 1, draw_arrows? 20 : 1000, draw_arrows? 20 : 0);
                    mid = (from_ + to_) / 2;
                }
                else
                {
                    Point projected = LinearAlg2D::getClosestOnLineSegment(point, segment.from(), segment.to());
                    mid = (point + projected) / 2;
                    svg.writeArrow(from_, mid, SVG::Color::BLUE, 1, draw_arrows? 20 : 1000, draw_arrows? 20 : 0);
                    svg.writeArrow(mid, to_, SVG::Color::BLUE, 1, draw_arrows? 20 : 1000, draw_arrows? 20 : 0);
                    //std::vector<Point> discretization;
                    //boost::polygon::voronoi_visual_utils<voronoi_data_t>::discretize(point, *segment, 10, &discretization)
                }
                if (show_parabola_generators)
                {
                    svg.writeLine(mid, point, SVG::Color::GRAY);
                    svg.writeLine(mid, LinearAlg2D::getClosestOnLineSegment(mid, segment.from(), segment.to()), SVG::Color::GRAY);
                }
            }
        }
        else 
        {
            if (edge.is_infinite())
            {
//                 printf("Edge is infinite\n");
            }
            else
            {
//                 printf("Cannot draw edge\n");
            }
            if (edge.vertex0())
            {
                if (draw_points) svg.writePoint(Point(edge.vertex0()->x(), edge.vertex0()->y()), false, 3, SVG::Color::RED);
            }
            if (edge.vertex1())
            {
                if (draw_points) svg.writePoint(Point(edge.vertex1()->x(), edge.vertex1()->y()), false, 3, SVG::Color::RED);
            }
        }
    }
    
    if (show_coords)
    {
        for (const vd_t::vertex_type& vert : vd.vertices())
        {
            svg.writePoint(Point(vert.x(), vert.y()), show_coords, 2, SVG::Color::RED);
        }
    }
    
//     for (const vd_t::cell_type& cell : vd.cells())
//     {
//         
//     }
}


std::vector<Point> VoronoiUtils::discretizeParabola(Point generator_point, Segment generator_segment, Point start, Point end, coord_t approximate_step_size)
{
    std::vector<Point> discretized;
    
    return discretized;
}

}//namespace arachne
