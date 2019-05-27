//Copyright (c) 2018 Ultimaker B.V.

#include <optional>
#include <stack>


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


std::vector<Point> VoronoiUtils::discretizeParabola(const Point& p, const Segment& segment, Point s, Point e, coord_t approximate_step_size)
{
    std::vector<Point> discretized;
    // x is distance of point projected on the segment ab
    // xx is point projected on the segment ab
    Point a = segment.from();
    Point b = segment.to();
    Point ab = b - a;
    Point as = s - a;
    Point ae = e - a;
    coord_t ab_size = vSize(ab);
    coord_t sx = dot(as, ab) / ab_size;
    coord_t ex = dot(ae, ab) / ab_size;
    coord_t sxex = ex - sx;
    
    Point ap = p - a;
    coord_t px = dot(ap, ab) / ab_size;
    
    Point pxx = LinearAlg2D::getClosestOnLine(p, a, b);
    Point ppxx = pxx - p;
    coord_t d = vSize(ppxx);
    PointMatrix rot = PointMatrix(turn90CCW(ppxx));
    
    
    coord_t step_count = static_cast<coord_t>(static_cast<float>(std::abs(ex - sx)) / approximate_step_size + 0.5);
    
    discretized.emplace_back(s);
    for (coord_t step = 1; step < step_count; step++)
    {
        coord_t x = sx + sxex * step / step_count - px;
        coord_t y = x * x / (2 * d) + d / 2;
        Point result = rot.unapply(Point(x, y)) + pxx;
        discretized.emplace_back(result);
    }
    discretized.emplace_back(e);
    return discretized;
}

// adapted from boost::polygon::voronoi_visual_utils.cpp
void VoronoiUtils::discretize(
        const Point& point,
        const Segment& segment,
        const coord_t max_dist,
        std::vector<Point>* discretization) {
    // Apply the linear transformation to move start point of the segment to
    // the point with coordinates (0, 0) and the direction of the segment to
    // coincide the positive direction of the x-axis.
    Point segm_vec = segment.to() - segment.from();
    coord_t sqr_segment_length = vSize2(segm_vec);

    // Compute x-coordinates of the endpoints of the edge
    // in the transformed space.
    coord_t projection_start = sqr_segment_length *
            get_point_projection((*discretization)[0], segment);
    coord_t projection_end = sqr_segment_length *
            get_point_projection((*discretization)[1], segment);

    // Compute parabola parameters in the transformed space.
    // Parabola has next representation:
    // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
    Point point_vec = point - segment.from();
    coord_t rot_x = dot(segm_vec, point_vec);
    coord_t rot_y = cross(segm_vec, point_vec);

    // Save the last point.
    Point last_point = (*discretization)[1];
    discretization->pop_back();

    // Use stack to avoid recursion.
    std::stack<coord_t> point_stack;
    point_stack.push(projection_end);
    Point cur(projection_start, parabola_y(projection_start, rot_x, rot_y));

    // Adjust max_dist parameter in the transformed space.
    const coord_t max_dist_transformed = max_dist * max_dist * sqr_segment_length;
    while (!point_stack.empty()) {
        Point new_(point_stack.top(), parabola_y(point_stack.top(), rot_x, rot_y));
        Point new_vec = new_ - cur;

        // Compute coordinates of the point of the parabola that is
        // furthest from the current line segment.
        coord_t mid_x = new_vec.Y * rot_y / new_vec.X + rot_x;
        coord_t mid_y = parabola_y(mid_x, rot_x, rot_y);
        Point mid_vec = Point(mid_x, mid_y) - cur;

        // Compute maximum distance between the given parabolic arc
        // and line segment that discretize it.
        __int128 dist = cross(mid_vec, new_vec);
        dist = dist * dist / vSize2(new_vec); // TODO overflows!!!
        if (dist <= max_dist_transformed) {
            // Distance between parabola and line segment is less than max_dist.
            point_stack.pop();
            coord_t inter_x = (segm_vec.X * new_.X - segm_vec.Y * new_.Y) /
                    sqr_segment_length + segment.from().X;
            coord_t inter_y = (segm_vec.X * new_.Y + segm_vec.Y * new_.X) /
                    sqr_segment_length + segment.from().Y;
            discretization->push_back(Point(inter_x, inter_y));
            cur = new_;
        } else {
            point_stack.push(mid_x);
        }
    }

    // Update last point.
    discretization->back() = last_point;
}

// adapted from boost::polygon::voronoi_visual_utils.cpp
coord_t VoronoiUtils::parabola_y(coord_t x, coord_t a, coord_t b) {
    return ((x - a) * (x - a) + b * b) / (b + b);
}

// adapted from boost::polygon::voronoi_visual_utils.cpp
double VoronoiUtils::get_point_projection(
        const Point& point, const Segment& segment) {
    Point segment_vec = segment.to() - segment.from();
    Point point_vec = point - segment.from();
    coord_t sqr_segment_length = vSize2(segment_vec);
    coord_t vec_dot = dot(segment_vec, point_vec);
    return static_cast<double>(vec_dot) / sqr_segment_length;
}

}//namespace arachne
