// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/polygonUtils.h"

#include <array>
#include <list>
#include <numbers>
#include <sstream>
#include <unordered_set>

#include <range/v3/view/enumerate.hpp>

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/SingleShape.h"
#include "infill.h"
#include "utils/SparsePointGridInclusive.h"
#include "utils/linearAlg2D.h"

#ifdef DEBUG
#include <spdlog/spdlog.h>

#include "utils/AABB.h"
#endif

namespace cura
{

const std::function<int(Point2LL)> PolygonUtils::no_penalty_function = [](Point2LL)
{
    return 0;
};

std::vector<Point2LL> PolygonUtils::spreadDotsArea(const Shape& polygons, coord_t grid_size)
{
    return spreadDotsArea(polygons, Point2LL(grid_size, grid_size));
}

std::vector<Point2LL> PolygonUtils::spreadDotsArea(const Shape& polygons, Point2LL grid_size)
{
    std::vector<VariableWidthLines> dummy_toolpaths;
    Settings dummy_settings;
    Infill infill_gen(EFillMethod::LINES, false, false, polygons, 0, grid_size.X, 0, 1, 0, 0, 0, 0, 0);
    Shape result_polygons;
    OpenLinesSet result_lines;
    infill_gen.generate(dummy_toolpaths, result_polygons, result_lines, dummy_settings, 0, SectionType::DOTS); // FIXME: @jellespijker make sure the propper layer nr is used
    std::vector<Point2LL> result;
    for (const OpenPolyline& line : result_lines)
    {
        assert(line.size() == 2);
        Point2LL a = line[0];
        Point2LL b = line[1];
        assert(a.X == b.X);
        if (a.Y > b.Y)
        {
            std::swap(a, b);
        }
        for (coord_t y = a.Y - (a.Y % grid_size.Y) - grid_size.Y; y < b.Y; y += grid_size.Y)
        {
            if (y < a.Y)
                continue;
            result.emplace_back(a.X, y);
        }
    }
    return result;
}

bool PolygonUtils::lineSegmentPolygonsIntersection(
    const Point2LL& a,
    const Point2LL& b,
    const Shape& current_outlines,
    const LocToLineGrid& outline_locator,
    Point2LL& result,
    const coord_t within_max_dist)
{
    const coord_t within_max_dist2 = within_max_dist * within_max_dist;

    Point2LL coll;
    coord_t closest_dist2 = within_max_dist2;

    const auto processOnIntersect = [&result, &closest_dist2, &a, &b, &coll](const Point2LL& p_start, const Point2LL& p_end)
    {
        if (LinearAlg2D::lineLineIntersection(a, b, p_start, p_end, coll) && LinearAlg2D::pointIsProjectedBeyondLine(coll, p_start, p_end) == 0
            && LinearAlg2D::pointIsProjectedBeyondLine(coll, a, b) == 0)
        {
            const coord_t dist2 = vSize2(b - coll);
            if (dist2 < closest_dist2)
            {
                closest_dist2 = dist2;
                result = coll;
            }
        }
    };

    const auto nearby = outline_locator.getNearby(b, within_max_dist);
    if (! nearby.empty())
    {
        for (const auto& pp_idx : nearby)
        {
            processOnIntersect(pp_idx.p(), pp_idx.next().p());
        }
        if (closest_dist2 < within_max_dist2)
        {
            return true;
        }
    }

    for (const auto& poly : current_outlines)
    {
        const size_t poly_size = poly.size();
        for (size_t i_segment_start = 0; i_segment_start < poly_size; ++i_segment_start)
        {
            const size_t i_segment_end = (i_segment_start + 1) % poly_size;
            processOnIntersect(poly[i_segment_start], poly[i_segment_end]);
        }
    }

    return closest_dist2 < within_max_dist2;
}

Point2LL PolygonUtils::getVertexInwardNormal(const Polyline& poly, unsigned int point_idx)
{
    Point2LL p1 = poly[point_idx];

    int p0_idx;
    for (p0_idx = int(point_idx) - 1; (unsigned int)p0_idx != point_idx; p0_idx = p0_idx - 1)
    { // find the last point different from p1
        if (p0_idx == -1)
        {
            p0_idx = poly.size() - 1;
        }
        if (poly[p0_idx] != p1)
        {
            break;
        }
    }
    Point2LL p0 = poly[p0_idx];

    unsigned int p2_idx;
    for (p2_idx = point_idx + 1; p2_idx != point_idx; p2_idx = p2_idx + 1)
    { // find the next point different from p1
        if (p2_idx == poly.size())
        {
            p2_idx = 0;
        }
        if (poly[p2_idx] != p1)
        {
            break;
        }
    }
    const Point2LL& p2 = poly[p2_idx];

    Point2LL off0 = turn90CCW(normal(p1 - p0, MM2INT(10.0))); // 10.0 for some precision
    Point2LL off1 = turn90CCW(normal(p2 - p1, MM2INT(10.0))); // 10.0 for some precision
    Point2LL n = off0 + off1;
    return n;
}

Point2LL PolygonUtils::getBoundaryPointWithOffset(const Polyline& poly, unsigned int point_idx, int64_t offset)
{
    return poly[point_idx] + normal(getVertexInwardNormal(poly, point_idx), -offset);
}

unsigned int PolygonUtils::moveOutside(const Shape& polygons, Point2LL& from, int distance, int64_t maxDist2)
{
    return moveInside(polygons, from, -distance, maxDist2);
}

ClosestPointPolygon PolygonUtils::moveInside2(
    const Shape& polygons,
    Point2LL& from,
    const int distance,
    const int64_t max_dist2,
    const Shape* loc_to_line_polygons,
    const LocToLineGrid* loc_to_line_grid,
    const std::function<int(Point2LL)>& penalty_function)
{
    std::optional<ClosestPointPolygon> closest_polygon_point;
    if (loc_to_line_grid)
    {
        closest_polygon_point = findClose(from, *loc_to_line_polygons, *loc_to_line_grid, penalty_function);
    }
    if (! closest_polygon_point)
    {
        closest_polygon_point = findClosest(from, polygons, penalty_function);
    }
    return _moveInside2(*closest_polygon_point, distance, from, max_dist2);
}

ClosestPointPolygon PolygonUtils::moveInside2(
    const Shape& loc_to_line_polygons,
    const Polygon& polygon,
    Point2LL& from,
    const int distance,
    const int64_t max_dist2,
    const LocToLineGrid* loc_to_line_grid,
    const std::function<int(Point2LL)>& penalty_function)
{
    std::optional<ClosestPointPolygon> closest_polygon_point;
    if (loc_to_line_grid)
    {
        closest_polygon_point = findClose(from, loc_to_line_polygons, *loc_to_line_grid, penalty_function);
    }
    if (! closest_polygon_point)
    {
        closest_polygon_point = findClosest(from, polygon, penalty_function);
    }
    return _moveInside2(*closest_polygon_point, distance, from, max_dist2);
}

ClosestPointPolygon PolygonUtils::_moveInside2(const ClosestPointPolygon& closest_polygon_point, const int distance, Point2LL& from, const int64_t max_dist2)
{
    if (! closest_polygon_point.isValid())
    {
        return ClosestPointPolygon(); // stub with invalid indices to signify we haven't found any
    }
    const Point2LL v_boundary_from = from - closest_polygon_point.location_;
    Point2LL result = moveInside(closest_polygon_point, distance);
    const Point2LL v_boundary_result = result - closest_polygon_point.location_;
    if (dot(v_boundary_result, v_boundary_from) > 0)
    { // point was already on the correct side of the polygon
        if (vSize2(v_boundary_from) > distance * distance)
        { // [from] was already on the correct side of the boudary by enough distance
            // don't change [from]
            return closest_polygon_point;
        }
        else
        {
            from = result;
            return closest_polygon_point;
        }
    }
    else
    {
        if (vSize2(v_boundary_from) > max_dist2)
        {
            return ClosestPointPolygon(closest_polygon_point.poly_); // stub with invalid indices to signify we haven't found any
        }
        else
        {
            from = result;
            return closest_polygon_point;
        }
    }
}

/*
 * Implementation assumes moving inside, but moving outside should just as well be possible.
 */
size_t PolygonUtils::moveInside(const Shape& polygons, Point2LL& from, int distance, int64_t maxDist2)
{
    Point2LL ret = from;
    int64_t bestDist2 = std::numeric_limits<int64_t>::max();
    size_t bestPoly = NO_INDEX;
    bool is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary
    for (size_t poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        const Polygon& poly = polygons[poly_idx];
        if (poly.size() < 2)
            continue;
        Point2LL p0 = poly[poly.size() - 2];
        Point2LL p1 = poly.back();
        // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
        // to avoid integer rounding edge cases
        bool projected_p_beyond_prev_segment = dot(p1 - p0, from - p0) >= vSize2(p1 - p0);
        for (const Point2LL& p2 : poly)
        {
            // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
            //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
            // X = P projected on AB
            const Point2LL& a = p1;
            const Point2LL& b = p2;
            const Point2LL& p = from;
            Point2LL ab = b - a;
            Point2LL ap = p - a;
            int64_t ab_length2 = vSize2(ab);
            if (ab_length2 <= 0) // A = B, i.e. the input polygon had two adjacent points on top of each other.
            {
                p1 = p2; // Skip only one of the points.
                continue;
            }
            int64_t dot_prod = dot(ab, ap);
            if (dot_prod <= 0) // x is projected to before ab
            {
                if (projected_p_beyond_prev_segment)
                { //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    Point2LL& x = p1;

                    int64_t dist2 = vSize2(x - p);
                    if (dist2 < bestDist2)
                    {
                        bestDist2 = dist2;
                        bestPoly = poly_idx;
                        if (distance == 0)
                        {
                            ret = x;
                        }
                        else
                        {
                            Point2LL inward_dir = turn90CCW(normal(ab, MM2INT(10.0)) + normal(p1 - p0, MM2INT(10.0))); // inward direction irrespective of sign of [distance]
                            // MM2INT(10.0) to retain precision for the eventual normalization
                            ret = x + normal(inward_dir, distance);
                            is_already_on_correct_side_of_boundary = dot(inward_dir, p - x) * distance >= 0;
                        }
                    }
                }
                else
                {
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            }
            else if (dot_prod >= ab_length2) // x is projected to beyond ab
            {
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            }
            else
            { // x is projected to a point properly on the line segment (not onto a vertex). The case which looks like | .
                projected_p_beyond_prev_segment = false;
                Point2LL x = a + ab * dot_prod / ab_length2;

                int64_t dist2 = vSize2(p - x);
                if (dist2 < bestDist2)
                {
                    bestDist2 = dist2;
                    bestPoly = poly_idx;
                    if (distance == 0)
                    {
                        ret = x;
                    }
                    else
                    {
                        Point2LL inward_dir = turn90CCW(normal(ab, distance)); // inward or outward depending on the sign of [distance]
                        ret = x + inward_dir;
                        is_already_on_correct_side_of_boundary = dot(inward_dir, p - x) >= 0;
                    }
                }
            }
            p0 = p1;
            p1 = p2;
        }
    }
    if (is_already_on_correct_side_of_boundary) // when the best point is already inside and we're moving inside, or when the best point is already outside and we're moving outside
    {
        if (bestDist2 < distance * distance)
        {
            from = ret;
        }
        else
        {
            //            from = from; // original point stays unaltered. It is already inside by enough distance
        }
        return bestPoly;
    }
    else if (bestDist2 < maxDist2)
    {
        from = ret;
        return bestPoly;
    }
    return NO_INDEX;
}

// Version that works on single PolygonRef.
unsigned int PolygonUtils::moveInside(const ClosedPolyline& polygon, Point2LL& from, int distance, int64_t maxDist2)
{
    // TODO: This is copied from the moveInside of Polygons.
    /*
    We'd like to use this function as subroutine in moveInside(Polygons...), but
    then we'd need to recompute the distance of the point to the polygon, which
    is expensive. Or we need to return the distance. We need the distance there
    to compare with the distance to other polygons.
    */
    Point2LL ret = from;
    int64_t bestDist2 = std::numeric_limits<int64_t>::max();
    bool is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary

    if (polygon.size() < 2)
    {
        return 0;
    }
    Point2LL p0 = polygon[polygon.size() - 2];
    Point2LL p1 = polygon.back();
    // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
    // to avoid integer rounding edge cases
    bool projected_p_beyond_prev_segment = dot(p1 - p0, from - p0) >= vSize2(p1 - p0);
    for (const Point2LL& p2 : polygon)
    {
        // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
        //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
        // X = P projected on AB
        const Point2LL& a = p1;
        const Point2LL& b = p2;
        const Point2LL& p = from;
        Point2LL ab = b - a;
        Point2LL ap = p - a;
        int64_t ab_length2 = vSize2(ab);
        if (ab_length2 <= 0) // A = B, i.e. the input polygon had two adjacent points on top of each other.
        {
            p1 = p2; // Skip only one of the points.
            continue;
        }
        int64_t dot_prod = dot(ab, ap);
        if (dot_prod <= 0) // x is projected to before ab
        {
            if (projected_p_beyond_prev_segment)
            { //  case which looks like:   > .
                projected_p_beyond_prev_segment = false;
                Point2LL& x = p1;

                int64_t dist2 = vSize2(x - p);
                if (dist2 < bestDist2)
                {
                    bestDist2 = dist2;
                    if (distance == 0)
                    {
                        ret = x;
                    }
                    else
                    {
                        Point2LL inward_dir = turn90CCW(normal(ab, MM2INT(10.0)) + normal(p1 - p0, MM2INT(10.0))); // inward direction irrespective of sign of [distance]
                        // MM2INT(10.0) to retain precision for the eventual normalization
                        ret = x + normal(inward_dir, distance);
                        is_already_on_correct_side_of_boundary = dot(inward_dir, p - x) * distance >= 0;
                    }
                }
            }
            else
            {
                projected_p_beyond_prev_segment = false;
                p0 = p1;
                p1 = p2;
                continue;
            }
        }
        else if (dot_prod >= ab_length2) // x is projected to beyond ab
        {
            projected_p_beyond_prev_segment = true;
            p0 = p1;
            p1 = p2;
            continue;
        }
        else
        { // x is projected to a point properly on the line segment (not onto a vertex). The case which looks like | .
            projected_p_beyond_prev_segment = false;
            Point2LL x = a + ab * dot_prod / ab_length2;

            int64_t dist2 = vSize2(p - x);
            if (dist2 < bestDist2)
            {
                bestDist2 = dist2;
                if (distance == 0)
                {
                    ret = x;
                }
                else
                {
                    Point2LL inward_dir = turn90CCW(normal(ab, distance)); // inward or outward depending on the sign of [distance]
                    ret = x + inward_dir;
                    is_already_on_correct_side_of_boundary = dot(inward_dir, p - x) >= 0;
                }
            }
        }

        p0 = p1;
        p1 = p2;
    }

    if (is_already_on_correct_side_of_boundary) // when the best point is already inside and we're moving inside, or when the best point is already outside and we're moving outside
    {
        if (bestDist2 < distance * distance)
        {
            from = ret;
        }
    }
    else if (bestDist2 < maxDist2)
    {
        from = ret;
    }
    return 0;
}

Point2LL PolygonUtils::moveOutside(const ClosestPointPolygon& cpp, const int distance)
{
    return moveInside(cpp, -distance);
}

Point2LL PolygonUtils::moveInside(const ClosestPointPolygon& cpp, const int distance)
{
    if (! cpp.isValid())
    {
        return no_point;
    }
    if (distance == 0)
    { // the point which is assumed to be on the boundary doesn't have to be moved
        return cpp.location_;
    }
    const Polyline& poly = *cpp.poly_;
    unsigned int point_idx = cpp.point_idx_;
    const Point2LL& on_boundary = cpp.location_;

    const Point2LL& p1 = poly[point_idx];
    unsigned int p2_idx;
    for (p2_idx = point_idx + 1; p2_idx != point_idx; p2_idx = p2_idx + 1)
    { // find the next point different from p1
        if (p2_idx == poly.size())
        {
            p2_idx = 0;
        }
        if (poly[p2_idx] != p1)
        {
            break;
        }
    }
    const Point2LL& p2 = poly[p2_idx];

    if (on_boundary == p1)
    {
        return getBoundaryPointWithOffset(poly, point_idx, -distance);
    }
    else if (on_boundary == p2)
    {
        return getBoundaryPointWithOffset(poly, p2_idx, -distance);
    }
    else
    {
        const Point2LL& x = on_boundary; // on_boundary is already projected on p1-p2

        Point2LL inward_dir = turn90CCW(normal(p2 - p1, distance));
        return x + inward_dir;
    }
}

ClosestPointPolygon PolygonUtils::ensureInsideOrOutside(
    const Shape& polygons,
    Point2LL& from,
    int preferred_dist_inside,
    int64_t max_dist2,
    const Shape* loc_to_line_polygons,
    const LocToLineGrid* loc_to_line_grid,
    const std::function<int(Point2LL)>& penalty_function)
{
    const ClosestPointPolygon closest_polygon_point = moveInside2(polygons, from, preferred_dist_inside, max_dist2, loc_to_line_polygons, loc_to_line_grid, penalty_function);
    return ensureInsideOrOutside(polygons, from, closest_polygon_point, preferred_dist_inside, loc_to_line_polygons, loc_to_line_grid, penalty_function);
}

ClosestPointPolygon PolygonUtils::ensureInsideOrOutside(
    const Shape& polygons,
    Point2LL& from,
    const ClosestPointPolygon& closest_polygon_point,
    int preferred_dist_inside,
    const Shape* loc_to_line_polygons,
    const LocToLineGrid* loc_to_line_grid,
    const std::function<int(Point2LL)>& penalty_function)
{
    if (! closest_polygon_point.isValid())
    {
        return ClosestPointPolygon(); // we couldn't move inside
    }
    const Polygon& closest_poly = *closest_polygon_point.poly_;
    bool is_outside_boundary = closest_poly.orientation();

    {
        bool is_inside = closest_poly.inside(from) == is_outside_boundary; // inside a hole is outside the part
        if (is_inside == (preferred_dist_inside > 0))
        { // we ended up on the right side of the polygon
            // assume we didn't overshoot another polygon in [polygons]
            return closest_polygon_point;
        }
    }

    // try once more with half the preferred distance inside
    {
        int64_t max_dist2_here = std::numeric_limits<int64_t>::max(); // we already concluded we are close enough to the closest_poly when we obtained the closest_polygon_point
        moveInside2(*loc_to_line_polygons, closest_poly, from, preferred_dist_inside / 2, max_dist2_here, loc_to_line_grid, penalty_function);
        bool is_inside = closest_poly.inside(from) == is_outside_boundary; // inside a hole is outside the part
        if (is_inside == (preferred_dist_inside > 0))
        { // we ended up on the right side of the polygon
            // assume we didn't overshoot another polygon in [polygons]
            return closest_polygon_point;
        }
    }

    // if above fails, we perform an offset and sit directly on the offsetted polygon (and keep the result from the above moveInside)
    // The offset is performed on the closest reference polygon in order to save computation time
    {
        const coord_t offset = (is_outside_boundary) ? -preferred_dist_inside : preferred_dist_inside; // perform inset on outer boundary and outset on holes
        Shape insetted
            = closest_poly.offset(offset / 2); // perform less inset, because chances are (thin parts of) the polygon will disappear, given that moveInside did an overshoot
        if (insetted.size() == 0)
        {
            return ClosestPointPolygon(); // we couldn't move inside
        }
        ClosestPointPolygon inside = findClosest(from, insetted, penalty_function);
        if (inside.isValid())
        {
            bool is_inside = polygons.inside(inside.location_);
            if (is_inside != (preferred_dist_inside > 0))
            {
                // Insetting from the reference polygon ended up outside another polygon.
                // Perform an offset on all polygons instead.
                Shape all_insetted = polygons.offset(-preferred_dist_inside);
                ClosestPointPolygon overall_inside = findClosest(from, all_insetted, penalty_function);
                bool overall_is_inside = polygons.inside(overall_inside.location_);
                if (overall_is_inside != (preferred_dist_inside > 0))
                {
                    return ClosestPointPolygon();
                }
                inside = overall_inside;
            }
            from = inside.location_;
        } // otherwise we just return the closest polygon point without modifying the from location
        return closest_polygon_point; // don't return a ClosestPoint with a reference to the above local polygons variable
    }
}

void PolygonUtils::walkToNearestSmallestConnection(ClosestPointPolygon& poly1_result, ClosestPointPolygon& poly2_result)
{
    if (! poly1_result.isValid() || ! poly2_result.isValid())
    {
        return;
    }
    const Polygon& poly1 = *poly1_result.poly_;
    const Polygon& poly2 = *poly2_result.poly_;
    size_t poly1_idx = poly1_result.poly_idx_;
    size_t poly2_idx = poly2_result.poly_idx_;
    if (poly1_result.point_idx_ == NO_INDEX || poly2_result.point_idx_ == NO_INDEX)
    {
        return;
    }

    int equilibirum_limit = MM2INT(0.1); // hard coded value
    for (int loop_counter = 0; loop_counter < equilibirum_limit; loop_counter++)
    {
        unsigned int pos1_before = poly1_result.point_idx_;
        poly1_result = findNearestClosest(poly2_result.location_, poly1, poly1_result.point_idx_);
        unsigned int pos2_before = poly2_result.point_idx_;
        poly2_result = findNearestClosest(poly1_result.location_, poly2, poly2_result.point_idx_);

        if (poly1_result.point_idx_ == pos1_before && poly2_result.point_idx_ == pos2_before)
        {
            break;
        }
    }

    // check surrounding verts in order to prevent local optima like the following:
    // o      o
    // \.....|
    //  \_.-'|
    //   \---|
    //    \-'|
    //     o o >> should find connection here
    coord_t best_distance2 = vSize2(poly1_result.p() - poly2_result.p());
    auto check_neighboring_vert
        = [&best_distance2](const Polygon& from_poly, const Polygon& to_poly, ClosestPointPolygon& from_poly_result, ClosestPointPolygon& to_poly_result, bool vertex_after)
    {
        const Point2LL after_poly2_result = to_poly[(to_poly_result.point_idx_ + vertex_after) % to_poly.size()];
        const ClosestPointPolygon poly1_after_poly2_result = findNearestClosest(after_poly2_result, from_poly, from_poly_result.point_idx_);
        const coord_t poly1_after_poly2_result_dist2 = vSize2(poly1_after_poly2_result.p() - after_poly2_result);
        if (poly1_after_poly2_result_dist2 < best_distance2)
        {
            from_poly_result = poly1_after_poly2_result;
            to_poly_result.location_ = after_poly2_result;
            best_distance2 = poly1_after_poly2_result_dist2;
        }
    };
    check_neighboring_vert(poly1, poly2, poly1_result, poly2_result, false);
    check_neighboring_vert(poly1, poly2, poly1_result, poly2_result, true);
    check_neighboring_vert(poly2, poly1, poly2_result, poly1_result, false);
    check_neighboring_vert(poly2, poly1, poly2_result, poly1_result, true);

    poly1_result.poly_idx_ = poly1_idx;
    poly2_result.poly_idx_ = poly2_idx;
}

ClosestPointPolygon PolygonUtils::findNearestClosest(const Point2LL& from, const Polygon& polygon, int start_idx)
{
    ClosestPointPolygon forth = findNearestClosest(from, polygon, start_idx, 1);
    if (! forth.isValid())
    {
        return forth; // stop computation
    }
    ClosestPointPolygon back = findNearestClosest(from, polygon, start_idx, -1);
    assert(back.isValid());
    if (vSize2(forth.location_ - from) < vSize2(back.location_ - from))
    {
        return forth;
    }
    else
    {
        return back;
    }
}

ClosestPointPolygon PolygonUtils::findNearestClosest(const Point2LL& from, const Polygon& polygon, int start_idx, int direction)
{
    if (polygon.size() == 0)
    {
        return ClosestPointPolygon(&polygon);
    }
    Point2LL aPoint = polygon[0];
    Point2LL best = aPoint;

    int64_t closestDist = vSize2(from - best);
    int bestPos = 0;

    size_t poly_size = polygon.size();
    for (size_t p = 0; p < poly_size; p++)
    {
        int p1_idx = (poly_size + direction * p + start_idx) % poly_size;
        int p2_idx = (poly_size + direction * (p + 1) + start_idx) % poly_size;
        const Point2LL& p1 = polygon[p1_idx];
        const Point2LL& p2 = polygon[p2_idx];

        Point2LL closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1, p2);
        int64_t dist = vSize2(from - closest_here);
        if (dist < closestDist)
        {
            best = closest_here;
            closestDist = dist;
            bestPos = (direction > 0) ? p1_idx : p2_idx;
        }
        else
        {
            return ClosestPointPolygon(best, bestPos, &polygon);
        }
    }

    return ClosestPointPolygon(best, bestPos, &polygon);
}

ClosestPointPolygon PolygonUtils::findClosest(const Point2LL& from, const Shape& polygons, const std::function<int(Point2LL)>& penalty_function)
{
    ClosestPointPolygon none;

    if (polygons.size() == 0)
    {
        return none;
    }
    const Polygon* any_polygon = &(polygons[0]);
    unsigned int any_poly_idx;
    for (any_poly_idx = 0; any_poly_idx < polygons.size(); any_poly_idx++)
    { // find first point in all polygons
        if (polygons[any_poly_idx].size() > 0)
        {
            any_polygon = &(polygons[any_poly_idx]);
            break;
        }
    }
    if (any_polygon->size() == 0)
    {
        return none;
    }
    ClosestPointPolygon best((*any_polygon)[0], 0, any_polygon, any_poly_idx);

    int64_t closestDist2_score = vSize2(from - best.location_) + penalty_function(best.location_);

    for (unsigned int ply = 0; ply < polygons.size(); ply++)
    {
        const Polygon& poly = polygons[ply];
        if (poly.size() == 0)
            continue;
        ClosestPointPolygon closest_here = findClosest(from, poly, penalty_function);
        if (! closest_here.isValid())
        {
            continue;
        }
        int64_t dist2_score = vSize2(from - closest_here.location_) + penalty_function(closest_here.location_);
        if (dist2_score < closestDist2_score)
        {
            best = closest_here;
            closestDist2_score = dist2_score;
            best.poly_idx_ = ply;
        }
    }

    return best;
}

ClosestPointPolygon PolygonUtils::findClosest(const Point2LL& from, const Polygon& polygon, const std::function<int(Point2LL)>& penalty_function)
{
    if (polygon.size() == 0)
    {
        return ClosestPointPolygon(&polygon);
    }
    Point2LL aPoint = polygon[0];
    Point2LL best = aPoint;

    int64_t closestDist2_score = vSize2(from - best) + penalty_function(best);
    int bestPos = 0;

    for (unsigned int p = 0; p < polygon.size(); p++)
    {
        const Point2LL& p1 = polygon[p];

        unsigned int p2_idx = p + 1;
        if (p2_idx >= polygon.size())
            p2_idx = 0;
        const Point2LL& p2 = polygon[p2_idx];

        Point2LL closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1, p2);
        int64_t dist2_score = vSize2(from - closest_here) + penalty_function(closest_here);
        if (dist2_score < closestDist2_score)
        {
            best = closest_here;
            closestDist2_score = dist2_score;
            bestPos = p;
        }
    }

    return ClosestPointPolygon(best, bestPos, &polygon);
}

PolygonsPointIndex PolygonUtils::findNearestVert(const Point2LL& from, const Shape& polys)
{
    coord_t best_dist2 = std::numeric_limits<coord_t>::max();
    PolygonsPointIndex closest_vert;
    for (unsigned int poly_idx = 0; poly_idx < polys.size(); poly_idx++)
    {
        const Polygon& poly = polys[poly_idx];
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            const coord_t dist2 = vSize2(poly[point_idx] - from);
            if (dist2 < best_dist2)
            {
                best_dist2 = dist2;
                closest_vert = PolygonsPointIndex(&polys, poly_idx, point_idx);
            }
        }
    }
    return closest_vert;
}

unsigned int PolygonUtils::findNearestVert(const Point2LL& from, const Polygon& poly)
{
    int64_t best_dist2 = std::numeric_limits<int64_t>::max();
    unsigned int closest_vert_idx = -1;
    for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
    {
        int64_t dist2 = vSize2(poly[point_idx] - from);
        if (dist2 < best_dist2)
        {
            best_dist2 = dist2;
            closest_vert_idx = point_idx;
        }
    }
    return closest_vert_idx;
}

std::unique_ptr<LocToLineGrid> PolygonUtils::createLocToLineGrid(const Shape& polygons, int square_size)
{
    unsigned int n_points = 0;
    for (const auto& poly : polygons)
    {
        n_points += poly.size();
    }

    auto ret = std::make_unique<LocToLineGrid>(square_size, n_points);

    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        const Polygon& poly = polygons[poly_idx];
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            ret->insert(PolygonsPointIndex(&polygons, poly_idx, point_idx));
        }
    }
    return ret;
}

/*
 * The current implementation can check the same line segment multiple times,
 * since the same line segment can occur in multiple cells if it it longer than
 * the cell size of the SparsePointGridInclusive.
 *
 * We could skip the duplication by keeping a vector of vectors of bools.
 */
std::optional<ClosestPointPolygon>
    PolygonUtils::findClose(Point2LL from, const Shape& polygons, const LocToLineGrid& loc_to_line, const std::function<int(Point2LL)>& penalty_function)
{
    std::vector<PolygonsPointIndex> near_lines = loc_to_line.getNearby(from, loc_to_line.getCellSize());

    Point2LL best(0, 0);

    int64_t closest_dist2_score = std::numeric_limits<int64_t>::max();
    PolygonsPointIndex best_point_poly_idx(nullptr, NO_INDEX, NO_INDEX);
    for (PolygonsPointIndex& point_poly_index : near_lines)
    {
        const Polygon& poly = polygons[point_poly_index.poly_idx_];
        const Point2LL& p1 = poly[point_poly_index.point_idx_];
        const Point2LL& p2 = poly[(point_poly_index.point_idx_ + 1) % poly.size()];

        Point2LL closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1, p2);
        int64_t dist2_score = vSize2(from - closest_here) + penalty_function(closest_here);
        if (dist2_score < closest_dist2_score)
        {
            best = closest_here;
            closest_dist2_score = dist2_score;
            best_point_poly_idx = point_poly_index;
        }
    }
    if (best_point_poly_idx.poly_idx_ == NO_INDEX)
    {
        return std::optional<ClosestPointPolygon>();
    }
    else
    {
        return std::optional<ClosestPointPolygon>(std::in_place, best, best_point_poly_idx.point_idx_, &(polygons[best_point_poly_idx.poly_idx_]), best_point_poly_idx.poly_idx_);
    }
}

std::vector<std::pair<ClosestPointPolygon, ClosestPointPolygon>>
    PolygonUtils::findClose(const Polygon& from, const Shape& destination, const LocToLineGrid& destination_loc_to_line, const std::function<int(Point2LL)>& penalty_function)
{
    std::vector<std::pair<ClosestPointPolygon, ClosestPointPolygon>> ret;
    int p0_idx = from.size() - 1;
    Point2LL p0(from[p0_idx]);
    int grid_size = destination_loc_to_line.getCellSize();
    for (unsigned int p1_idx = 0; p1_idx < from.size(); p1_idx++)
    {
        const Point2LL& p1 = from[p1_idx];
        std::optional<ClosestPointPolygon> best_here = findClose(p1, destination, destination_loc_to_line, penalty_function);
        if (best_here)
        {
            ret.push_back(std::make_pair(ClosestPointPolygon(p1, p1_idx, &from), *best_here));
        }
        Point2LL p0p1 = p1 - p0;
        int dist_to_p1 = vSize(p0p1);
        for (unsigned int middle_point_nr = 1; dist_to_p1 > grid_size * 2; ++middle_point_nr)
        {
            Point2LL x = p0 + normal(p0p1, middle_point_nr * grid_size);
            dist_to_p1 -= grid_size;

            best_here = findClose(x, destination, destination_loc_to_line, penalty_function);
            if (best_here)
            {
                ret.push_back(std::make_pair(ClosestPointPolygon(x, p0_idx, &from), *best_here));
            }
        }
        p0 = p1;
        p0_idx = p1_idx;
    }
    return ret;
}

bool PolygonUtils::getNextPointWithDistance(Point2LL from, int64_t dist, const OpenPolyline& poly, int start_idx, int poly_start_idx, GivenDistPoint& result)
{
    Point2LL prev_poly_point = poly[(start_idx + poly_start_idx) % poly.size()];

    for (unsigned int prev_idx = start_idx; prev_idx < poly.size(); prev_idx++)
    {
        int next_idx = (prev_idx + 1 + poly_start_idx) % poly.size(); // last checked segment is between last point in poly and poly[0]...
        const Point2LL& next_poly_point = poly[next_idx];
        if (! shorterThen(next_poly_point - from, dist))
        {
            /*
             *                 x    r
             *      p.---------+---+------------.n
             *                L|  /
             *                 | / dist
             *                 |/
             *                f.
             *
             * f=from
             * p=prev_poly_point
             * n=next_poly_point
             * x= f projected on pn
             * r=result point at distance [dist] from f
             */

            Point2LL pn = next_poly_point - prev_poly_point;

            if (shorterThen(pn, 100)) // when precision is limited
            {
                Point2LL middle = (next_poly_point + prev_poly_point) / 2;
                coord_t dist_to_middle = vSize(from - middle);
                if (dist_to_middle - dist < 100 && dist_to_middle - dist > -100)
                {
                    result.location = middle;
                    result.pos = prev_idx;
                    return true;
                }
                else
                {
                    prev_poly_point = next_poly_point;
                    continue;
                }
            }

            Point2LL pf = from - prev_poly_point;
            Point2LL px = dot(pf, pn) / vSize(pn) * pn / vSize(pn);
            Point2LL xf = pf - px;

            if (! shorterThen(xf, dist)) // line lies wholly further than pn
            {
                prev_poly_point = next_poly_point;
                continue;
            }

            int64_t xr_dist = std::sqrt(dist * dist - vSize2(xf)); // inverse Pythagoras

            if (vSize(pn - px) - xr_dist < 1) // r lies beyond n
            {
                prev_poly_point = next_poly_point;
                continue;
            }

            Point2LL xr = xr_dist * pn / vSize(pn);
            Point2LL pr = px + xr;

            result.location = prev_poly_point + pr;
            result.pos = prev_idx;
            return true;
        }
        prev_poly_point = next_poly_point;
    }
    return false;
}

template<class LineType>
ClosestPoint<LineType> PolygonUtils::walk(const ClosestPoint<LineType>& from, coord_t distance)
{
    const LineType& poly = *from.poly_;
    Point2LL last_vertex = from.p();
    Point2LL next_vertex;
    size_t last_point_idx = from.point_idx_;
    for (size_t point_idx = from.point_idx_ + 1;; point_idx++)
    {
        if (point_idx == poly.size())
        {
            point_idx = 0;
        }
        next_vertex = poly[point_idx];
        distance -= vSize(last_vertex - next_vertex);
        if (distance <= 0)
            break;
        last_vertex = next_vertex;
        last_point_idx = point_idx;
    }
    Point2LL result = next_vertex + normal(last_vertex - next_vertex, -distance);
    return ClosestPoint<LineType>(result, last_point_idx, &poly, from.poly_idx_);
}

std::optional<ClosestPointPolygon> PolygonUtils::getNextParallelIntersection(const ClosestPointPolygon& start, const Point2LL& line_to, const coord_t dist, const bool forward)
{
    // <--o--t-----y----< poly 1
    //       :     :
    // >---o :====>:shift
    //      \:     :
    //       s     x
    //        \    :
    //         \   :
    //          o--r----->  poly 2
    //  s=start
    //  r=result
    //  t=line_to

    const Polygon& poly = *start.poly_;
    const Point2LL s = start.p();
    const Point2LL t = line_to;

    const Point2LL st = t - s;
    const Point2LL shift = normal(turn90CCW(st), dist);

    Point2LL prev_vert = s;
    coord_t prev_projected = 0;
    for (unsigned int next_point_nr = 0; next_point_nr < poly.size(); next_point_nr++)
    {
        const unsigned int next_point_idx = forward
                                              ? (start.point_idx_ + 1 + next_point_nr) % poly.size()
                                              : (static_cast<size_t>(start.point_idx_) - next_point_nr + poly.size()) % poly.size(); // cast in order to accomodate subtracting
        const Point2LL next_vert = poly[next_point_idx];
        const Point2LL so = next_vert - s;
        const coord_t projected = dot(shift, so) / dist;
        if (std::abs(projected) > dist)
        { // segment crosses the line through xy (or the one on the other side of st)
            const Point2LL segment_vector = next_vert - prev_vert;
            const coord_t segment_length = vSize(segment_vector);
            const coord_t projected_segment_length = std::abs(projected - prev_projected);
            const int16_t sign = (projected > 0) ? 1 : -1;
            const coord_t projected_inter_segment_length
                = dist - sign * prev_projected; // add the prev_projected to dist if it is projected to the other side of the input line than where the intersection occurs.
            const coord_t inter_segment_length = segment_length * projected_inter_segment_length / projected_segment_length;
            const Point2LL intersection = prev_vert + normal(next_vert - prev_vert, inter_segment_length);

            size_t vert_before_idx = next_point_idx;
            if (forward)
            {
                vert_before_idx = (next_point_idx > 0) ? vert_before_idx - 1 : poly.size() - 1;
            }
            assert(vert_before_idx < poly.size());
            return ClosestPointPolygon(intersection, vert_before_idx, &poly);
        }

        prev_vert = next_vert;
        prev_projected = projected;
    }

    return std::optional<ClosestPointPolygon>();
}


bool PolygonUtils::polygonCollidesWithLineSegment(const Point2LL from, const Point2LL to, const LocToLineGrid& loc_to_line, PolygonsPointIndex* collision_result)
{
    bool ret = false;
    Point2LL diff = to - from;
    if (vSize2(diff) < 2)
    { // transformation matrix would fail
        return false;
    }

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point2LL transformed_from = transformation_matrix.apply(from);
    Point2LL transformed_to = transformation_matrix.apply(to);

    PolygonsPointIndex result;

    std::function<bool(const PolygonsPointIndex&)> process_elem_func
        = [transformed_from, transformed_to, &transformation_matrix, &result, &ret](const PolygonsPointIndex& line_start)
    {
        Point2LL p0 = transformation_matrix.apply(line_start.p());
        Point2LL p1 = transformation_matrix.apply(line_start.next().p());

        if (LinearAlg2D::lineSegmentsCollide(transformed_from, transformed_to, p0, p1))
        {
            result = line_start;
            ret = true;
            return false;
        }
        return true;
    };
    loc_to_line.processLine(std::make_pair(from, to), process_elem_func);

    if (collision_result)
    {
        *collision_result = result;
    }
    return ret;
}

bool PolygonUtils::polygonCollidesWithLineSegment(
    const Polygon& poly,
    const Point2LL& transformed_startPoint,
    const Point2LL& transformed_endPoint,
    PointMatrix transformation_matrix)
{
    Point2LL p0 = transformation_matrix.apply(poly.back());
    for (Point2LL p1_ : poly)
    {
        Point2LL p1 = transformation_matrix.apply(p1_);
        if (LinearAlg2D::lineSegmentsCollide(transformed_startPoint, transformed_endPoint, p0, p1))
        {
            return true;
        }
        p0 = p1;
    }
    return false;
}

bool PolygonUtils::polygonCollidesWithLineSegment(const Polygon& poly, const Point2LL& startPoint, const Point2LL& endPoint)
{
    Point2LL diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point2LL transformed_startPoint = transformation_matrix.apply(startPoint);
    Point2LL transformed_endPoint = transformation_matrix.apply(endPoint);

    return PolygonUtils::polygonCollidesWithLineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix);
}

bool PolygonUtils::polygonCollidesWithLineSegment(
    const Shape& polys,
    const Point2LL& transformed_startPoint,
    const Point2LL& transformed_endPoint,
    PointMatrix transformation_matrix)
{
    for (const Polygon& poly : polys)
    {
        if (poly.size() == 0)
        {
            continue;
        }
        if (PolygonUtils::polygonCollidesWithLineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix))
        {
            return true;
        }
    }

    return false;
}


bool PolygonUtils::polygonCollidesWithLineSegment(const Shape& polys, const Point2LL& startPoint, const Point2LL& endPoint)
{
    if (endPoint == startPoint)
    {
        return false; // Zero-length line segments never collide.
    }
    Point2LL diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point2LL transformed_startPoint = transformation_matrix.apply(startPoint);
    Point2LL transformed_endPoint = transformation_matrix.apply(endPoint);

    return polygonCollidesWithLineSegment(polys, transformed_startPoint, transformed_endPoint, transformation_matrix);
}

bool PolygonUtils::polygonsIntersect(const Polygon& poly_a, const Polygon& poly_b)
{
    // only do the full intersection when the polys' BBs overlap
    AABB bba(poly_a);
    AABB bbb(poly_b);
    return bba.hit(bbb) && poly_a.intersection(poly_b).size() > 0;
}

bool PolygonUtils::polygonOutlinesAdjacent(const Polygon& inner_poly, const Polygon& outer_poly, const coord_t max_gap)
{
    // Heuristic check if their AABBs are near first.
    AABB inner_aabb(inner_poly);
    AABB outer_aabb(outer_poly);
    inner_aabb.max_ += Point2LL(max_gap, max_gap); // Expand one of them by way of a "distance" by checking intersection with the expanded rectangle.
    inner_aabb.min_ -= Point2LL(max_gap, max_gap);
    if (! inner_aabb.hit(outer_aabb))
    {
        return false;
    }

    // Heuristic says they are near. Now check for real.
    const coord_t max_gap2 = max_gap * max_gap;
    const unsigned outer_poly_size = outer_poly.size();
    for (unsigned line_index = 0; line_index < outer_poly_size; ++line_index)
    {
        const Point2LL lp0 = outer_poly[line_index];
        const Point2LL lp1 = outer_poly[(line_index + 1) % outer_poly_size];
        for (Point2LL inner_poly_point : inner_poly)
        {
            if (LinearAlg2D::getDist2FromLineSegment(lp0, inner_poly_point, lp1) < max_gap2)
            {
                return true;
            }
        }
    }
    return false;
}

void PolygonUtils::findAdjacentPolygons(
    std::vector<unsigned>& adjacent_poly_indices,
    const Polygon& poly,
    const std::vector<const Polygon*>& possible_adjacent_polys,
    const coord_t max_gap)
{
    // given a polygon, and a vector of polygons, return a vector containing the indices of the polygons that are adjacent to the given polygon
    for (unsigned poly_idx = 0; poly_idx < possible_adjacent_polys.size(); ++poly_idx)
    {
        if (polygonOutlinesAdjacent(poly, *possible_adjacent_polys[poly_idx], max_gap) || polygonOutlinesAdjacent(*possible_adjacent_polys[poly_idx], poly, max_gap))
        {
            adjacent_poly_indices.push_back(poly_idx);
        }
    }
}

double PolygonUtils::relativeHammingDistance(const Shape& poly_a, const Shape& poly_b)
{
    const double area_a = std::abs(poly_a.area());
    const double area_b = std::abs(poly_b.area());
    const double total_area = area_a + area_b;

    // If the total area is 0.0, we'd get a division by zero. Instead, only return 0.0 if they are exactly equal.
    constexpr bool borders_allowed = true;
    if (total_area == 0.0)
    {
        for (const Polygon& polygon_a : poly_a)
        {
            for (Point2LL point : polygon_a)
            {
                if (! poly_b.inside(point, borders_allowed))
                {
                    return 1.0;
                }
            }
        }
        for (const Polygon& polygon_b : poly_b)
        {
            for (Point2LL point : polygon_b)
            {
                if (! poly_a.inside(point, borders_allowed))
                {
                    return 1.0;
                }
            }
        }
        return 0.0; // All points are inside the other polygon, regardless of where the vertices are along the edges.
    }

    const Shape symmetric_difference = poly_a.xorPolygons(poly_b);
    const double hamming_distance = symmetric_difference.area();
    return hamming_distance / total_area;
}

Polygon PolygonUtils::makeDisc(const Point2LL& mid, const coord_t radius, const size_t steps)
{
    return makeCircle<Polygon>(mid, radius, steps);
}

Point2LL PolygonUtils::makeCirclePoint(const Point2LL& mid, const coord_t radius, const AngleRadians& angle)
{
    return mid + Point2LL(std::llrint(static_cast<double>(radius) * cos(angle)), std::llrint(static_cast<double>(radius) * sin(angle)));
}

ClosedPolyline PolygonUtils::makeWheel(const Point2LL& mid, const coord_t inner_radius, const coord_t outer_radius, const size_t semi_nb_spokes, const size_t arc_angle_resolution)
{
    ClosedPolyline wheel;

    const size_t nb_spokes = semi_nb_spokes * 2;
    const double spoke_angle_step = TAU / static_cast<double>(nb_spokes);
    const double arc_angle_step = spoke_angle_step / static_cast<double>(arc_angle_resolution);

    for (size_t spoke = 0; spoke < nb_spokes; ++spoke)
    {
        const double spoke_angle = static_cast<double>(spoke) * spoke_angle_step;
        const coord_t radius = spoke % 2 == 0 ? inner_radius : outer_radius;

        for (size_t arc_part = 0; arc_part <= arc_angle_resolution; ++arc_part)
        {
            const double angle = spoke_angle + static_cast<double>(arc_part) * arc_angle_step;
            wheel.push_back(makeCirclePoint(mid, radius, angle));
        }
    }

    return wheel;
}

Shape PolygonUtils::connect(const Shape& input)
{
    Shape ret;
    std::vector<SingleShape> parts = input.splitIntoParts(true);
    for (SingleShape& part : parts)
    {
        Polygon& outline = part.outerPolygon();
        for (size_t hole_idx = 1; hole_idx < part.size(); hole_idx++)
        {
            Polygon& hole = part[hole_idx];
            Point2LL hole_point = hole[0];
            hole.push_back(hole_point);
            // find where the scanline passes the Y
            size_t best_segment_to_idx = 0;
            coord_t best_dist = std::numeric_limits<coord_t>::max();
            Point2LL best_intersection_point = outline.back();

            Point2LL prev = outline.back();
            for (size_t point_idx = 0; point_idx < outline.size(); point_idx++)
            {
                Point2LL here = outline[point_idx];
                if (here.Y > hole_point.Y && prev.Y <= hole_point.Y && here.Y != prev.Y)
                {
                    Point2LL intersection_point = prev + (here - prev) * (hole_point.Y - prev.Y) / (here.Y - prev.Y);
                    coord_t dist = hole_point.X - intersection_point.X;
                    if (dist > 0 && dist < best_dist)
                    {
                        best_dist = dist;
                        best_segment_to_idx = point_idx;
                        best_intersection_point = intersection_point;
                    }
                }
                prev = here;
            }
            outline.insert(outline.begin() + best_segment_to_idx, 2, best_intersection_point);
            outline.insert(outline.begin() + best_segment_to_idx + 1, hole.begin(), hole.end());
        }
        ret.push_back(outline);
    }
    return ret;
}

/* Note: Also tries to solve for near-self intersections, when epsilon >= 1
 */
void PolygonUtils::fixSelfIntersections(const coord_t epsilon, Shape& polygon)
{
    if (epsilon < 1)
    {
        polygon.simplify();
        return;
    }

    const coord_t half_epsilon = std::max(10LL, (epsilon + 1) / 2);

    // Points too close to line segments should be moved a little away from those line segments, but less than epsilon,
    //   so at least half-epsilon distance between points can still be guaranteed.
    constexpr coord_t grid_size = 2000;
    auto query_grid = PolygonUtils::createLocToLineGrid(polygon, grid_size);

    const coord_t move_dist = half_epsilon - 2;
    const coord_t half_epsilon_sqrd = half_epsilon * half_epsilon;

    const size_t n = polygon.size();
    for (size_t poly_idx = 0; poly_idx < n; poly_idx++)
    {
        const size_t pathlen = polygon[poly_idx].size();
        for (size_t point_idx = 0; point_idx < pathlen; ++point_idx)
        {
            Point2LL& pt = polygon[poly_idx][point_idx];
            for (const auto& line : query_grid->getNearby(pt, epsilon * 2))
            {
                const size_t line_next_idx = (line.point_idx_ + 1) % polygon[line.poly_idx_].size();
                if (poly_idx == line.poly_idx_ && (point_idx == line.point_idx_ || point_idx == line_next_idx))
                {
                    continue;
                }

                const Point2LL& a = polygon[line.poly_idx_][line.point_idx_];
                const Point2LL& b = polygon[line.poly_idx_][line_next_idx];

                if (half_epsilon_sqrd >= vSize2(pt - LinearAlg2D::getClosestOnLineSegment(pt, a, b)))
                {
                    const Point2LL& other = polygon[poly_idx][(point_idx + 1) % pathlen];
                    const Point2LL vec = LinearAlg2D::pointIsLeftOfLine(other, a, b) > 0 ? b - a : a - b;
                    const coord_t len = std::max(vSize(vec), 1LL);
                    pt.X += (-vec.Y * move_dist) / len;
                    pt.Y += (vec.X * move_dist) / len;
                }
            }
        }
    }

    polygon.simplify();
}

Shape PolygonUtils::unionManySmall(const Shape& polygon)
{
    if (polygon.size() < 8)
    {
        return polygon.unionPolygons();
    }

    Shape a, b;
    a.reserve(polygon.size() / 2);
    b.reserve(a.size() + 1);
    for (const auto& [i, path] : polygon | ranges::views::enumerate)
    {
        (i % 2 == 0 ? b : a).push_back(path);
    }
    return unionManySmall(a).unionPolygons(unionManySmall(b));
}

Shape PolygonUtils::clipPolygonWithAABB(const Shape& src, const AABB& aabb)
{
    Shape out;
    out.reserve(src.size());
    for (const auto path : src)
    {
        Polygon poly;

        const size_t cnt = path.size();
        if (cnt < 3)
        {
            return Shape();
        }

        enum class Side
        {
            Left = 1,
            Right = 2,
            Top = 4,
            Bottom = 8
        };

        auto sides = [aabb](const Point2LL& p)
        {
            return int(p.X < aabb.min_.X) * int(Side::Left) + int(p.X > aabb.max_.X) * int(Side::Right) + int(p.Y < aabb.min_.Y) * int(Side::Bottom)
                 + int(p.Y > aabb.max_.Y) * int(Side::Top);
        };

        int sides_prev = sides(path.back());
        int sides_this = sides(path.front());
        const size_t last = cnt - 1;

        for (size_t i = 0; i < last; ++i)
        {
            int sides_next = sides(path[i + 1]);
            if ( // This point is inside. Take it.
                sides_this == 0 ||
                // Either this point is outside and previous or next is inside, or
                // the edge possibly cuts corner of the bounding box.
                (sides_prev & sides_this & sides_next) == 0)
            {
                poly.push_back(path[i]);
                sides_prev = sides_this;
            }
            else
            {
                // All the three points (this, prev, next) are outside at the same side.
                // Ignore this point.
            }
            sides_this = sides_next;
        }

        // Never produce just a single point output polygon.
        if (! poly.empty())
        {
            int sides_next = sides(poly.front());
            if (sides_this == 0 || // The last point is inside. Take it.
                (sides_prev & sides_this & sides_next) == 0) // Either this point is outside and previous or next is inside, or the edge possibly cuts corner of the bounding box.

                poly.push_back(path.back());
        }

        if (! poly.empty())
        {
            out.push_back(poly);
        }
    }
    return out;
}

std::tuple<ClosedLinesSet, coord_t>
    PolygonUtils::generateCirculatOutset(const Point2LL& center, const coord_t inner_radius, const coord_t outer_radius, coord_t line_width, const size_t circle_definition)
{
    ClosedLinesSet outset;
    const coord_t semi_line_width = line_width / 2;
    coord_t radius = inner_radius + semi_line_width;

    while (radius + semi_line_width <= outer_radius)
    {
        outset.push_back(makeCircle(center, radius, circle_definition));
        radius += line_width;
    }

    return { outset, radius - semi_line_width };
}

ClosedLinesSet PolygonUtils::generateCircularInset(const Point2LL& center, const coord_t outer_radius, const coord_t line_width, const size_t circle_definition)
{
    ClosedLinesSet inset;
    const coord_t semi_line_width = line_width / 2;
    coord_t radius = outer_radius - semi_line_width;

    while (radius - semi_line_width >= line_width)
    {
        inset.push_back(makeCircle(center, radius, circle_definition));
        radius -= line_width;
    }

    return inset;
}

template ClosestPoint<Polygon> PolygonUtils::walk(const ClosestPoint<Polygon>& from, coord_t distance);
template ClosestPoint<OpenPolyline> PolygonUtils::walk(const ClosestPoint<OpenPolyline>& from, coord_t distance);

} // namespace cura
