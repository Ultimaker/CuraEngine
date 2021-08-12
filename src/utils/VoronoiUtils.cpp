//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <stack>
#include <optional>

#include "linearAlg2D.h"
#include "logoutput.h"
#include "VoronoiUtils.h"

namespace cura 
{

Point VoronoiUtils::p(const vd_t::vertex_type* node)
{
    const double x = node->x();
    const double y = node->y();
    return Point(x + 0.5 - (x < 0), y + 0.5 - (y < 0)); //Round to nearest integer coordinates.
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
    if(!cell.contains_point())
    {
        logWarning("Voronoi cell doesn't contain a source point!");
    }
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
    return points[cell.source_index()];
}

PolygonsPointIndex VoronoiUtils::getSourcePointIndex(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    assert(cell.contains_point());
    if(!cell.contains_point())
    {
        logWarning("Voronoi cell doesn't contain a source point!");
    }
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
    PolygonsPointIndex ret = segments[cell.source_index() - points.size()];
    return ++ret;
}

const VoronoiUtils::Segment& VoronoiUtils::getSourceSegment(const vd_t::cell_type& cell, const std::vector<Point>& points, const std::vector<Segment>& segments)
{
    assert(cell.contains_segment());
    if(!cell.contains_segment())
    {
        logWarning("Voronoi cell doesn't contain a source segment!");
    }
    return segments[cell.source_index() - points.size()];
}


std::vector<Point> VoronoiUtils::discretizeParabola(const Point& p, const Segment& segment, Point s, Point e, coord_t approximate_step_size, float transitioning_angle)
{
    std::vector<Point> discretized;
    // x is distance of point projected on the segment ab
    // xx is point projected on the segment ab
    const Point a = segment.from();
    const Point b = segment.to();
    const Point ab = b - a;
    const Point as = s - a;
    const Point ae = e - a;
    const coord_t ab_size = vSize(ab);
    const coord_t sx = dot(as, ab) / ab_size;
    const coord_t ex = dot(ae, ab) / ab_size;
    const coord_t sxex = ex - sx;
    
    const Point ap = p - a;
    const coord_t px = dot(ap, ab) / ab_size;
    
    const Point pxx = LinearAlg2D::getClosestOnLine(p, a, b);
    const Point ppxx = pxx - p;
    const coord_t d = vSize(ppxx);
    const PointMatrix rot = PointMatrix(turn90CCW(ppxx));
    
    if (d == 0)
    {
        discretized.emplace_back(s);
        discretized.emplace_back(e);
        return discretized;
    }
    
    const float marking_bound = atan(transitioning_angle * 0.5);
    coord_t msx = - marking_bound * d; // projected marking_start
    coord_t mex = marking_bound * d; // projected marking_end
    const coord_t marking_start_end_h = msx * msx / (2 * d) + d / 2;
    Point marking_start = rot.unapply(Point(msx, marking_start_end_h)) + pxx;
    Point marking_end = rot.unapply(Point(mex, marking_start_end_h)) + pxx;
    const int dir = (sx > ex) ? -1 : 1;
    if (dir < 0)
    {
        std::swap(marking_start, marking_end);
        std::swap(msx, mex);
    }
    
    bool add_marking_start = msx * dir > (sx - px) * dir && msx * dir < (ex - px) * dir;
    bool add_marking_end = mex * dir > (sx - px) * dir && mex * dir < (ex - px) * dir;

    const Point apex = rot.unapply(Point(0, d / 2)) + pxx;
    bool add_apex = (sx - px) * dir < 0 && (ex - px) * dir > 0;

    assert(!(add_marking_start && add_marking_end) || add_apex);
    if(add_marking_start && add_marking_end && !add_apex)
    {
        logWarning("Failing to discretize parabola! Must add an apex or one of the endpoints.");
    }
    
    const coord_t step_count = static_cast<coord_t>(static_cast<float>(std::abs(ex - sx)) / approximate_step_size + 0.5);
    
    discretized.emplace_back(s);
    for (coord_t step = 1; step < step_count; step++)
    {
        const coord_t x = sx + sxex * step / step_count - px;
        const coord_t y = x * x / (2 * d) + d / 2;
        
        if (add_marking_start && msx * dir < x * dir)
        {
            discretized.emplace_back(marking_start);
            add_marking_start = false;
        }
        if (add_apex && x * dir > 0)
        {
            discretized.emplace_back(apex);
            add_apex = false; // only add the apex just before the 
        }
        if (add_marking_end && mex * dir < x * dir)
        {
            discretized.emplace_back(marking_end);
            add_marking_end = false;
        }
        const Point result = rot.unapply(Point(x, y)) + pxx;
        discretized.emplace_back(result);
    }
    if (add_apex)
    {
        discretized.emplace_back(apex);
    }
    if (add_marking_end)
    {
        discretized.emplace_back(marking_end);
    }
    discretized.emplace_back(e);
    return discretized;
}

/*
The following code has been adjusted from an implementation of Boost. This is the license:

Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

// adapted from boost::polygon::voronoi_visual_utils.cpp
void VoronoiUtils::discretize(const Point& point, const Segment& segment, const coord_t max_dist, std::vector<Point>* discretization)
{
    // Apply the linear transformation to move start point of the segment to
    // the point with coordinates (0, 0) and the direction of the segment to
    // coincide the positive direction of the x-axis.
    const Point segm_vec = segment.to() - segment.from();
    const coord_t segment_length2 = vSize2(segm_vec);

    // Compute x-coordinates of the endpoints of the edge
    // in the transformed space.
    const coord_t projection_start = segment_length2 * getPointProjection((*discretization)[0], segment);
    const coord_t projection_end = segment_length2 * getPointProjection((*discretization)[1], segment);

    // Compute parabola parameters in the transformed space.
    // Parabola has next representation:
    // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
    const Point point_vec = point - segment.from();
    const coord_t rot_x = dot(segm_vec, point_vec);
    const coord_t rot_y = cross(segm_vec, point_vec);

    // Save the last point.
    const Point last_point = (*discretization)[1];
    discretization->pop_back();

    // Use stack to avoid recursion.
    std::stack<coord_t> point_stack;
    point_stack.push(projection_end);
    Point cur(projection_start, parabolaY(projection_start, rot_x, rot_y));

    // Adjust max_dist parameter in the transformed space.
    const coord_t max_dist_transformed = max_dist * max_dist * segment_length2;
    while (!point_stack.empty()) 
    {
        const Point new_(point_stack.top(), parabolaY(point_stack.top(), rot_x, rot_y));
        const Point new_vec = new_ - cur;

        // Compute coordinates of the point of the parabola that is
        // furthest from the current line segment.
        const coord_t mid_x = new_vec.Y * rot_y / new_vec.X + rot_x;
        const coord_t mid_y = parabolaY(mid_x, rot_x, rot_y);
        Point mid_vec = Point(mid_x, mid_y) - cur;

        // Compute maximum distance between the given parabolic arc
        // and line segment that discretize it.
        coord_t dist = cross(mid_vec, new_vec);
        dist = dist * dist / vSize2(new_vec); // TODO overflows!!!
        if (dist <= max_dist_transformed)
        {
            // Distance between parabola and line segment is less than max_dist.
            point_stack.pop();
            const coord_t inter_x = (segm_vec.X * new_.X - segm_vec.Y * new_.Y) / segment_length2 + segment.from().X;
            const coord_t inter_y = (segm_vec.X * new_.Y + segm_vec.Y * new_.X) / segment_length2 + segment.from().Y;
            discretization->push_back(Point(inter_x, inter_y));
            cur = new_;
        }
        else
        {
            point_stack.push(mid_x);
        }
    }

    // Update last point.
    discretization->back() = last_point;
}

// adapted from boost::polygon::voronoi_visual_utils.cpp
coord_t VoronoiUtils::parabolaY(coord_t x, coord_t a, coord_t b)
{
    return ((x - a) * (x - a) + b * b) / (b + b);
}

// adapted from boost::polygon::voronoi_visual_utils.cpp
double VoronoiUtils::getPointProjection(const Point& point, const Segment& segment)
{
    Point segment_vec = segment.to() - segment.from();
    Point point_vec = point - segment.from();
    coord_t sqr_segment_length = vSize2(segment_vec);
    coord_t vec_dot = dot(segment_vec, point_vec);
    return static_cast<double>(vec_dot) / sqr_segment_length;
}

}//namespace cura
