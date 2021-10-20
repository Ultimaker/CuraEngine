//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <array>
#include <list>
#include <sstream>
#include <unordered_set>

#include "linearAlg2D.h"
#include "polygonUtils.h"
#include "SparsePointGridInclusive.h"
#include "../utils/logoutput.h"
#include "../infill.h"

#ifdef DEBUG
#include "AABB.h"
#include "SVG.h"
#endif

namespace cura
{

const std::function<int(Point)> PolygonUtils::no_penalty_function = [](Point){ return 0; };

int64_t PolygonUtils::segmentLength(PolygonsPointIndex start, PolygonsPointIndex end)
{
    assert(start.poly_idx == end.poly_idx);
    int64_t segment_length = 0;
    Point prev_vert = start.p();
    ConstPolygonRef poly = (*start.polygons)[start.poly_idx];
    for (unsigned int point_idx = 1; point_idx <= poly.size(); point_idx++)
    {
        unsigned int vert_idx = (start.point_idx + point_idx) % poly.size();
        Point vert = poly[vert_idx];
        segment_length += vSize(vert - prev_vert);

        if (vert_idx == end.point_idx)
        { // break at the end of the loop, so that [end] and [start] may be the same
            return segment_length;
        }
        prev_vert = vert;
    }
    assert(false && "The segment end should have been encountered!");
    return segment_length;
}

void PolygonUtils::spreadDots(PolygonsPointIndex start, PolygonsPointIndex end, unsigned int n_dots, std::vector<ClosestPolygonPoint>& result)
{
    assert(start.poly_idx == end.poly_idx);
    int64_t segment_length = segmentLength(start, end);

    ConstPolygonRef poly = (*start.polygons)[start.poly_idx];
    unsigned int n_dots_in_between = n_dots;
    if (start == end)
    {
        result.emplace_back(start.p(), start.point_idx, poly);
        n_dots_in_between--; // generate one less below, because we already pushed a point to the result
    }

    int64_t wipe_point_dist = segment_length / (n_dots_in_between + 1); // distance between two wipe points; keep a distance at both sides of the segment

    int64_t dist_past_vert_to_insert_point = wipe_point_dist;
    unsigned int n_points_generated = 0;
    PolygonsPointIndex vert = start;
    while (true)
    {
        Point p0 = vert.p();
        Point p1 = vert.next().p();
        Point p0p1 = p1 - p0;
        int64_t p0p1_length = vSize(p0p1);

        for ( ; dist_past_vert_to_insert_point < p0p1_length && n_points_generated < n_dots_in_between; dist_past_vert_to_insert_point += wipe_point_dist)
        {
            result.emplace_back(p0 + normal(p0p1, dist_past_vert_to_insert_point), vert.point_idx, poly);
            n_points_generated++;
        }
        dist_past_vert_to_insert_point -= p0p1_length;

        ++vert;
        if (vert == end)
        { // break at end of loop to allow for [start] and [end] being the same, meaning the full polygon
            break;
        }
    }
    assert(result.size() == n_dots && "we didn't generate as many wipe locations as we asked for.");
}

std::vector<Point> PolygonUtils::spreadDotsArea(const Polygons& polygons, coord_t grid_size)
{
    VariableWidthPaths dummy_toolpaths;
    Settings dummy_settings;
    Infill infill_gen(EFillMethod::LINES, false, false, polygons, 0, grid_size, 0, 1, 0, 0, 0, 0, 0);
    Polygons result_polygons;
    Polygons result_lines;
    infill_gen.generate(dummy_toolpaths, result_polygons, result_lines, dummy_settings);
    std::vector<Point> result;
    for (PolygonRef line : result_lines)
    {
        assert(line.size() == 2);
        Point a = line[0];
        Point b = line[1];
        assert(a.X == b.X);
        if (a.Y > b.Y)
        {
            std::swap(a, b);
        }
        for (coord_t y = a.Y - (a.Y % grid_size) - grid_size; y < b.Y; y += grid_size)
        {
            if (y < a.Y) continue;
            result.emplace_back(a.X, y);
        }
    }
    return result;
}

bool PolygonUtils::lineSegmentPolygonsIntersection(const Point& a, const Point& b, const Polygons& current_outlines, const LocToLineGrid& outline_locator, Point& result)
{
    Point coll;
    coord_t closest_dist2 = std::numeric_limits<coord_t>::max();

    const auto processOnIntersect =
        [&result, &closest_dist2, &a, &b, &coll](const Point& p_start, const Point& p_end)
    {
        if
        (
            LinearAlg2D::lineLineIntersection(a, b, p_start, p_end, coll) &&
            LinearAlg2D::pointIsProjectedBeyondLine(coll, p_start, p_end) == 0 &&
            LinearAlg2D::pointIsProjectedBeyondLine(coll, a, b) == 0
        )
        {
            const coord_t dist2 = vSize2(b - coll);
            if (dist2 < closest_dist2)
            {
                closest_dist2 = dist2;
                result = coll;
            }
        }
    };

    const auto nearby = outline_locator.getNearby(b, outline_locator.getCellSize() * 2);
    if (! nearby.empty())
    {
        for (const auto& pp_idx : nearby)
        {
            processOnIntersect(pp_idx.p(), pp_idx.next().p());
        }
        if (closest_dist2 < std::numeric_limits<coord_t>::max())
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

    return closest_dist2 < std::numeric_limits<coord_t>::max();
}

Point PolygonUtils::getVertexInwardNormal(ConstPolygonRef poly, unsigned int point_idx)
{
    Point p1 = poly[point_idx];

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
    Point p0 = poly[p0_idx];

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
    const Point& p2 = poly[p2_idx];

    Point off0 = turn90CCW(normal(p1 - p0, MM2INT(10.0))); // 10.0 for some precision
    Point off1 = turn90CCW(normal(p2 - p1, MM2INT(10.0))); // 10.0 for some precision
    Point n = off0 + off1;
    return n;
}

Point PolygonUtils::getBoundaryPointWithOffset(ConstPolygonRef poly, unsigned int point_idx, int64_t offset)
{
    return poly[point_idx] + normal(getVertexInwardNormal(poly, point_idx), -offset);
}

Point PolygonUtils::moveInsideDiagonally(ClosestPolygonPoint point_on_boundary, int64_t inset)
{
    if (!point_on_boundary.isValid())
    {
        return no_point;
    }
    ConstPolygonRef poly = **point_on_boundary.poly;
    Point p0 = poly[point_on_boundary.point_idx];
    Point p1 = poly[(point_on_boundary.point_idx + 1) % poly.size()];
    if (vSize2(p0 - point_on_boundary.location) < vSize2(p1 - point_on_boundary.location))
    {
        return point_on_boundary.location + normal(getVertexInwardNormal(poly, point_on_boundary.point_idx), inset);
    }
    else
    {
        return point_on_boundary.location + normal(getVertexInwardNormal(poly, (point_on_boundary.point_idx + 1) % poly.size()), inset);
    }
}

unsigned int PolygonUtils::moveOutside(const Polygons& polygons, Point& from, int distance, int64_t maxDist2)
{
    return moveInside(polygons, from, -distance, maxDist2);
}

ClosestPolygonPoint PolygonUtils::moveInside2(const Polygons& polygons, Point& from, const int distance, const int64_t max_dist2, const Polygons* loc_to_line_polygons, const LocToLineGrid* loc_to_line_grid, const std::function<int(Point)>& penalty_function)
{
    std::optional<ClosestPolygonPoint> closest_polygon_point;
    if (loc_to_line_grid)
    {
        closest_polygon_point = findClose(from, *loc_to_line_polygons, *loc_to_line_grid, penalty_function);
    }
    if (!closest_polygon_point)
    {
        closest_polygon_point = findClosest(from, polygons, penalty_function);
    }
    return _moveInside2(*closest_polygon_point, distance, from, max_dist2);
}

ClosestPolygonPoint PolygonUtils::moveInside2(const Polygons& loc_to_line_polygons, ConstPolygonRef polygon, Point& from, const int distance, const int64_t max_dist2, const LocToLineGrid* loc_to_line_grid, const std::function<int(Point)>& penalty_function)
{
    std::optional<ClosestPolygonPoint> closest_polygon_point;
    if (loc_to_line_grid)
    {
        closest_polygon_point = findClose(from, loc_to_line_polygons, *loc_to_line_grid, penalty_function);
    }
    if (!closest_polygon_point)
    {
        closest_polygon_point = findClosest(from, polygon, penalty_function);
    }
    return _moveInside2(*closest_polygon_point, distance, from, max_dist2);
}

ClosestPolygonPoint PolygonUtils::_moveInside2(const ClosestPolygonPoint& closest_polygon_point, const int distance, Point& from, const int64_t max_dist2)
{
    if (!closest_polygon_point.isValid())
    {
        return ClosestPolygonPoint(); // stub with invalid indices to signify we haven't found any
    }
    const Point v_boundary_from = from - closest_polygon_point.location;
    Point result = moveInside(closest_polygon_point, distance);
    const Point v_boundary_result = result - closest_polygon_point.location;
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
            return ClosestPolygonPoint(*closest_polygon_point.poly); // stub with invalid indices to signify we haven't found any
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
unsigned int PolygonUtils::moveInside(const Polygons& polygons, Point& from, int distance, int64_t maxDist2)
{
    Point ret = from;
    int64_t bestDist2 = std::numeric_limits<int64_t>::max();
    unsigned int bestPoly = NO_INDEX;
    bool is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        ConstPolygonRef poly = polygons[poly_idx];
        if (poly.size() < 2)
            continue;
        Point p0 = poly[poly.size()-2];
        Point p1 = poly.back();
        // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
        // to avoid integer rounding edge cases
        bool projected_p_beyond_prev_segment = dot(p1 - p0, from - p0) >= vSize2(p1 - p0);
        for(const Point& p2 : poly)
        {
            // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
            //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
            // X = P projected on AB
            const Point& a = p1;
            const Point& b = p2;
            const Point& p = from;
            Point ab = b - a;
            Point ap = p - a;
            int64_t ab_length2 = vSize2(ab);
            if(ab_length2 <= 0) //A = B, i.e. the input polygon had two adjacent points on top of each other.
            {
                p1 = p2; //Skip only one of the points.
                continue;
            }
            int64_t dot_prod = dot(ab, ap);
            if (dot_prod <= 0) // x is projected to before ab
            {
                if (projected_p_beyond_prev_segment)
                { //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    Point& x = p1;

                    int64_t dist2 = vSize2(x - p);
                    if (dist2 < bestDist2)
                    {
                        bestDist2 = dist2;
                        bestPoly = poly_idx;
                        if (distance == 0) { ret = x; }
                        else
                        {
                            Point inward_dir = turn90CCW(normal(ab, MM2INT(10.0)) + normal(p1 - p0, MM2INT(10.0))); // inward direction irrespective of sign of [distance]
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
                Point x = a + ab * dot_prod / ab_length2;

                int64_t dist2 = vSize2(p - x);
                if (dist2 < bestDist2)
                {
                    bestDist2 = dist2;
                    bestPoly = poly_idx;
                    if (distance == 0) { ret = x; }
                    else
                    {
                        Point inward_dir = turn90CCW(normal(ab, distance)); // inward or outward depending on the sign of [distance]
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

//Version that works on single PolygonRef.
unsigned int PolygonUtils::moveInside(const ConstPolygonRef polygon, Point& from, int distance, int64_t maxDist2)
{
    //TODO: This is copied from the moveInside of Polygons.
    /*
    We'd like to use this function as subroutine in moveInside(Polygons...), but
    then we'd need to recompute the distance of the point to the polygon, which
    is expensive. Or we need to return the distance. We need the distance there
    to compare with the distance to other polygons.
    */
    Point ret = from;
    int64_t bestDist2 = std::numeric_limits<int64_t>::max();
    bool is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary

    if (polygon.size() < 2)
    {
        return 0;
    }
    Point p0 = polygon[polygon.size() - 2];
    Point p1 = polygon.back();
    // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
    // to avoid integer rounding edge cases
    bool projected_p_beyond_prev_segment = dot(p1 - p0, from - p0) >= vSize2(p1 - p0);
    for(const Point& p2 : polygon)
    {
        // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
        //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
        // X = P projected on AB
        const Point& a = p1;
        const Point& b = p2;
        const Point& p = from;
        Point ab = b - a;
        Point ap = p - a;
        int64_t ab_length2 = vSize2(ab);
        if(ab_length2 <= 0) //A = B, i.e. the input polygon had two adjacent points on top of each other.
        {
            p1 = p2; //Skip only one of the points.
            continue;
        }
        int64_t dot_prod = dot(ab, ap);
        if (dot_prod <= 0) // x is projected to before ab
        {
            if (projected_p_beyond_prev_segment)
            { //  case which looks like:   > .
                projected_p_beyond_prev_segment = false;
                Point& x = p1;

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
                        Point inward_dir = turn90CCW(normal(ab, MM2INT(10.0)) + normal(p1 - p0, MM2INT(10.0))); // inward direction irrespective of sign of [distance]
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
            Point x = a + ab * dot_prod / ab_length2;

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
                    Point inward_dir = turn90CCW(normal(ab, distance)); // inward or outward depending on the sign of [distance]
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

Point PolygonUtils::moveOutside(const ClosestPolygonPoint& cpp, const int distance)
{
    return moveInside(cpp, -distance);
}

Point PolygonUtils::moveInside(const ClosestPolygonPoint& cpp, const int distance)
{
    if (!cpp.isValid())
    {
        return no_point;
    }
    if (distance == 0)
    { // the point which is assumed to be on the boundary doesn't have to be moved
        return cpp.location;
    }
    ConstPolygonRef poly = *cpp.poly;
    unsigned int point_idx = cpp.point_idx;
    const Point& on_boundary = cpp.location;

    const Point& p1 = poly[point_idx];
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
    const Point& p2 = poly[p2_idx];

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
        const Point& x = on_boundary; // on_boundary is already projected on p1-p2

        Point inward_dir = turn90CCW(normal(p2 - p1, distance));
        return x + inward_dir;
    }
}

ClosestPolygonPoint PolygonUtils::ensureInsideOrOutside(const Polygons& polygons, Point& from, int preferred_dist_inside, int64_t max_dist2, const Polygons* loc_to_line_polygons, const LocToLineGrid* loc_to_line_grid, const std::function<int(Point)>& penalty_function)
{
    const ClosestPolygonPoint closest_polygon_point = moveInside2(polygons, from, preferred_dist_inside, max_dist2, loc_to_line_polygons, loc_to_line_grid, penalty_function);
    return ensureInsideOrOutside(polygons, from, closest_polygon_point, preferred_dist_inside, loc_to_line_polygons, loc_to_line_grid, penalty_function);
}

ClosestPolygonPoint PolygonUtils::ensureInsideOrOutside(const Polygons& polygons, Point& from, const ClosestPolygonPoint& closest_polygon_point, int preferred_dist_inside, const Polygons* loc_to_line_polygons, const LocToLineGrid* loc_to_line_grid, const std::function<int(Point)>& penalty_function)
{
    if (!closest_polygon_point.isValid())
    {
        return ClosestPolygonPoint(); // we couldn't move inside
    }
    ConstPolygonRef closest_poly = *closest_polygon_point.poly;
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
    int64_t max_dist2_here = std::numeric_limits<int64_t>::max(); // we already concluded we are close enough to the closest_poly when we obtained the closest_polygon_point
    moveInside2(*loc_to_line_polygons, closest_poly, from, preferred_dist_inside / 2, max_dist2_here, loc_to_line_grid, penalty_function);
    bool is_inside = closest_poly.inside(from) == is_outside_boundary; // inside a hole is outside the part
    if (is_inside == (preferred_dist_inside > 0))
    { // we ended up on the right side of the polygon
        // assume we didn't overshoot another polygon in [polygons]
        return closest_polygon_point;
    }
    // if above fails, we perform an offset and sit directly on the offsetted polygon (and keep the result from the above moveInside)
    // The offset is performed on the closest reference polygon in order to save computation time
    else
    {
        const coord_t offset = (is_outside_boundary) ? -preferred_dist_inside : preferred_dist_inside; // perform inset on outer boundary and outset on holes
        Polygons insetted = closest_poly.offset(offset / 2); // perform less inset, because chances are (thin parts of) the polygon will disappear, given that moveInside did an overshoot
        if (insetted.size() == 0)
        {
            return ClosestPolygonPoint(); // we couldn't move inside
        }
        ClosestPolygonPoint inside = findClosest(from, insetted, penalty_function);
        if (inside.isValid())
        {
            bool is_inside = polygons.inside(inside.location);
            if (is_inside != (preferred_dist_inside > 0))
            {
                // Insetting from the reference polygon ended up outside another polygon.
                // Perform an offset on all polygons instead.
                Polygons all_insetted = polygons.offset(-preferred_dist_inside);
                ClosestPolygonPoint overall_inside = findClosest(from, all_insetted, penalty_function);
                bool overall_is_inside = polygons.inside(overall_inside.location);
                if (overall_is_inside != (preferred_dist_inside > 0))
                {
#ifdef DEBUG
                    try
                    {
                        int offset_performed = offset / 2;
                        AABB aabb(polygons);
                        aabb.expand(std::abs(preferred_dist_inside) * 2);
                        SVG svg("debug.html", aabb);
                        svg.writeComment("Original polygon in black");
                        svg.writePolygons(polygons, SVG::Color::BLACK);
                        for (auto poly : polygons)
                        {
                            for (auto point : poly)
                            {
                                svg.writePoint(point, true, 2);
                            }
                        }
                        std::stringstream ss;
                        svg.writeComment("Reference polygon in yellow");
                        svg.writePolygon(closest_poly, SVG::Color::YELLOW);
                        ss << "Offsetted polygon in blue with offset " << offset_performed;
                        svg.writeComment(ss.str());
                        svg.writePolygons(insetted, SVG::Color::BLUE);
                        for (auto poly : insetted)
                        {
                            for (auto point : poly)
                            {
                                svg.writePoint(point, true, 2);
                            }
                        }
                        svg.writeComment("From location");
                        svg.writePoint(from, true, 5, SVG::Color::GREEN);
                        svg.writeComment("Location computed to be inside the black polygon");
                        svg.writePoint(inside.location, true, 5, SVG::Color::RED);
                    }
                    catch(...)
                    {
                    }
                    logError("Clipper::offset failed. See generated debug.html!\n\tBlack is original\n\tBlue is offsetted polygon\n");
#endif
                    return ClosestPolygonPoint();
                }
                inside = overall_inside;
            }
            from = inside.location;
        } // otherwise we just return the closest polygon point without modifying the from location
        return closest_polygon_point; // don't return a ClosestPolygonPoint with a reference to the above local polygons variable
    }
}



std::pair<ClosestPolygonPoint, ClosestPolygonPoint> PolygonUtils::findConnection(ConstPolygonRef poly1, Polygons& polys2, coord_t min_connection_length, coord_t max_connection_length, std::function<bool (std::pair<ClosestPolygonPoint, ClosestPolygonPoint>)> precondition)
{
    ClosestPolygonPoint invalid;
    std::pair<ClosestPolygonPoint, ClosestPolygonPoint> ret = std::make_pair(invalid, invalid);
    if (poly1.empty() || polys2.empty())
    {
        return ret;
    }

    const coord_t min_connection_dist2 = min_connection_length * min_connection_length;
    const coord_t max_connection_dist2 = max_connection_length * max_connection_length;

    auto grid = PolygonUtils::createLocToLineGrid(polys2, max_connection_length);


    std::unordered_set<std::pair<size_t, PolygonsPointIndex>> checked_segment_pairs; // pairs of index into segment start on poly1 and PolygonsPointIndex to segment start on polys2

    for (size_t point_idx = 0; point_idx < poly1.size(); point_idx++)
    {
        std::function<bool (const PolygonsPointIndex&)> process_elem_func =
            [&, point_idx](const PolygonsPointIndex& line_from)
            {
                std::pair<size_t, PolygonsPointIndex> segment_pair = std::make_pair(point_idx, line_from);
                if (checked_segment_pairs.count(segment_pair) > 0)
                { // these two line segments were already checked
                    return true; // continue looking for connections
                }

                Point a1 = poly1[point_idx];
                Point a2 = poly1[(point_idx + 1) % poly1.size()];
                Point b1 = line_from.p();
                Point b2 = line_from.next().p();
                std::pair<Point, Point> connection = LinearAlg2D::getClosestConnection(a1, a2, b1, b2);
                coord_t dist2 = vSize2(connection.first - connection.second);
                ret = std::make_pair(
                    ClosestPolygonPoint(connection.first, point_idx, poly1),
                    ClosestPolygonPoint(connection.second, line_from.point_idx, polys2[line_from.poly_idx], line_from.poly_idx));
                if (min_connection_dist2 < dist2 && dist2 < max_connection_dist2
                    && precondition(ret))
                {
                    return false; // stop the search; break the for-loop
                }

                checked_segment_pairs.emplace(point_idx, line_from);
                return true; // continue looking for connections
            };

        std::pair<Point, Point> line = std::make_pair(poly1[point_idx], poly1[(point_idx + 1) % poly1.size()]);
        Point normal_vector = normal(turn90CCW(line.second - line.first), max_connection_length);
        std::pair<Point, Point> line2 = std::make_pair(line.first + normal_vector, line.second + normal_vector); // for neighborhood around the line
        std::pair<Point, Point> line3 = std::make_pair(line.first - normal_vector, line.second - normal_vector); // for neighborhood around the line

        bool continue_;
        continue_ = grid->processLine(line, process_elem_func);
        if (!continue_) break;
        continue_ = grid->processLine(line2, process_elem_func);
        if (!continue_) break;
        continue_ = grid->processLine(line3, process_elem_func);
        if (!continue_) break;
    }
    ret.first.poly_idx = 0;
    return ret;
}

void PolygonUtils::findSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result)
{
    if (!poly1_result.poly || !poly2_result.poly)
    {
        return;
    }
    ConstPolygonRef poly1 = *poly1_result.poly;
    ConstPolygonRef poly2 = *poly2_result.poly;
    if (poly1.size() == 0 || poly2.size() == 0)
    {
        return;
    }

    Point center1 = poly1[0];
    ClosestPolygonPoint intermediate_poly2_result = findClosest(center1, poly2);
    ClosestPolygonPoint intermediate_poly1_result = findClosest(intermediate_poly2_result.p(), poly1);

    poly2_result = findClosest(intermediate_poly1_result.p(), poly2);
    poly1_result = findClosest(poly2_result.p(), poly1);
    walkToNearestSmallestConnection(poly1_result, poly2_result);
}

void PolygonUtils::walkToNearestSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result)
{
    if (!poly1_result.isValid() || !poly2_result.isValid())
    {
        return;
    }
    ConstPolygonRef poly1 = *poly1_result.poly;
    ConstPolygonRef poly2 = *poly2_result.poly;
    size_t poly1_idx = poly1_result.poly_idx;
    size_t poly2_idx = poly2_result.poly_idx;
    if (poly1_result.point_idx == NO_INDEX || poly2_result.point_idx == NO_INDEX)
    {
        return;
    }

    int equilibirum_limit = MM2INT(0.1); // hard coded value
    for (int loop_counter = 0; loop_counter < equilibirum_limit; loop_counter++)
    {
        unsigned int pos1_before = poly1_result.point_idx;
        poly1_result = findNearestClosest(poly2_result.location, poly1, poly1_result.point_idx);
        unsigned int pos2_before = poly2_result.point_idx;
        poly2_result = findNearestClosest(poly1_result.location, poly2, poly2_result.point_idx);

        if (poly1_result.point_idx == pos1_before && poly2_result.point_idx == pos2_before)
        {
            break;
        }
    }

    // check surrounding verts in order to prevent local optima like the following:
    //o      o
    // \.....|
    //  \_.-'|
    //   \---|
    //    \-'|
    //     o o >> should find connection here
    coord_t best_distance2 = vSize2(poly1_result.p() - poly2_result.p());
    auto check_neighboring_vert = [&best_distance2](ConstPolygonRef from_poly, ConstPolygonRef to_poly, ClosestPolygonPoint& from_poly_result, ClosestPolygonPoint& to_poly_result, bool vertex_after)
        {
            const Point after_poly2_result = to_poly[(to_poly_result.point_idx + vertex_after) % to_poly.size()];
            const ClosestPolygonPoint poly1_after_poly2_result = findNearestClosest(after_poly2_result, from_poly, from_poly_result.point_idx);
            const coord_t poly1_after_poly2_result_dist2 = vSize2(poly1_after_poly2_result.p() - after_poly2_result);
            if (poly1_after_poly2_result_dist2 < best_distance2)
            {
                from_poly_result = poly1_after_poly2_result;
                to_poly_result.location = after_poly2_result;
                best_distance2 = poly1_after_poly2_result_dist2;
            }
        };
    check_neighboring_vert(poly1, poly2, poly1_result, poly2_result, false);
    check_neighboring_vert(poly1, poly2, poly1_result, poly2_result, true);
    check_neighboring_vert(poly2, poly1, poly2_result, poly1_result, false);
    check_neighboring_vert(poly2, poly1, poly2_result, poly1_result, true);

    poly1_result.poly_idx = poly1_idx;
    poly2_result.poly_idx = poly2_idx;
}

ClosestPolygonPoint PolygonUtils::findNearestClosest(Point from, ConstPolygonRef polygon, int start_idx)
{
    ClosestPolygonPoint forth = findNearestClosest(from, polygon, start_idx, 1);
    if (!forth.isValid())
    {
        return forth; // stop computation
    }
    ClosestPolygonPoint back = findNearestClosest(from, polygon, start_idx, -1);
    assert(back.isValid());
    if (vSize2(forth.location - from) < vSize2(back.location - from))
    {
        return forth;
    }
    else
    {
        return back;
    }
}

ClosestPolygonPoint PolygonUtils::findNearestClosest(Point from, ConstPolygonRef polygon, int start_idx, int direction)
{
    if (polygon.size() == 0)
    {
        return ClosestPolygonPoint(polygon);
    }
    Point aPoint = polygon[0];
    Point best = aPoint;

    int64_t closestDist = vSize2(from - best);
    int bestPos = 0;

    size_t poly_size = polygon.size();
    for (size_t p = 0; p < poly_size; p++)
    {
        int p1_idx = (poly_size + direction * p + start_idx) % poly_size;
        int p2_idx = (poly_size + direction * (p + 1) + start_idx) % poly_size;
        const Point& p1 = polygon[p1_idx];
        const Point& p2 = polygon[p2_idx];

        Point closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist = vSize2(from - closest_here);
        if (dist < closestDist)
        {
            best = closest_here;
            closestDist = dist;
            bestPos = (direction > 0) ? p1_idx : p2_idx;
        }
        else
        {
            return ClosestPolygonPoint(best, bestPos, polygon);
        }
    }

    return ClosestPolygonPoint(best, bestPos, polygon);
}

ClosestPolygonPoint PolygonUtils::findClosest(Point from, const Polygons& polygons, const std::function<int(Point)>& penalty_function)
{
    ClosestPolygonPoint none;

    if (polygons.size() == 0)
    {
        return none;
    }
    ConstPolygonPointer any_polygon = polygons[0];
    unsigned int any_poly_idx;
    for (any_poly_idx = 0; any_poly_idx < polygons.size(); any_poly_idx++)
    { // find first point in all polygons
        if (polygons[any_poly_idx].size() > 0)
        {
            any_polygon = polygons[any_poly_idx];
            break;
        }
    }
    if (any_polygon->size() == 0)
    {
        return none;
    }
    ClosestPolygonPoint best((*any_polygon)[0], 0, *any_polygon, any_poly_idx);

    int64_t closestDist2_score = vSize2(from - best.location) + penalty_function(best.location);

    for (unsigned int ply = 0; ply < polygons.size(); ply++)
    {
        ConstPolygonRef poly = polygons[ply];
        if (poly.size() == 0) continue;
        ClosestPolygonPoint closest_here = findClosest(from, poly, penalty_function);
        if (!closest_here.isValid())
        {
            continue;
        }
        int64_t dist2_score = vSize2(from - closest_here.location) + penalty_function(closest_here.location);
        if (dist2_score < closestDist2_score)
        {
            best = closest_here;
            closestDist2_score = dist2_score;
            best.poly_idx = ply;
        }
    }

    return best;
}

ClosestPolygonPoint PolygonUtils::findClosest(Point from, ConstPolygonRef polygon, const std::function<int(Point)>& penalty_function)
{
    if (polygon.size() == 0)
    {
        return ClosestPolygonPoint(polygon);
    }
    Point aPoint = polygon[0];
    Point best = aPoint;

    int64_t closestDist2_score = vSize2(from - best) + penalty_function(best);
    int bestPos = 0;

    for (unsigned int p = 0; p<polygon.size(); p++)
    {
        const Point& p1 = polygon[p];

        unsigned int p2_idx = p+1;
        if (p2_idx >= polygon.size()) p2_idx = 0;
        const Point& p2 = polygon[p2_idx];

        Point closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist2_score = vSize2(from - closest_here) + penalty_function(closest_here);
        if (dist2_score < closestDist2_score)
        {
            best = closest_here;
            closestDist2_score = dist2_score;
            bestPos = p;
        }
    }

    return ClosestPolygonPoint(best, bestPos, polygon);
}

PolygonsPointIndex PolygonUtils::findNearestVert(const Point from, const Polygons& polys)
{
    int64_t best_dist2 = std::numeric_limits<int64_t>::max();
    PolygonsPointIndex closest_vert;
    for (unsigned int poly_idx = 0; poly_idx < polys.size(); poly_idx++)
    {
        ConstPolygonRef poly = polys[poly_idx];
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            int64_t dist2 = vSize2(poly[point_idx] - from);
            if (dist2 < best_dist2)
            {
                best_dist2 = dist2;
                closest_vert = PolygonsPointIndex(&polys, poly_idx, point_idx);
            }
        }
    }
    return closest_vert;
}

unsigned int PolygonUtils::findNearestVert(const Point from, ConstPolygonRef poly)
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

std::unique_ptr<LocToLineGrid> PolygonUtils::createLocToLineGrid(const Polygons& polygons, int square_size)
{
    unsigned int n_points = 0;
    for (const auto& poly : polygons)
    {
        n_points += poly.size();
    }

    auto ret = std::make_unique<LocToLineGrid>(square_size, n_points);

    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        ConstPolygonRef poly = polygons[poly_idx];
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
std::optional<ClosestPolygonPoint> PolygonUtils::findClose(
    Point from, const Polygons& polygons,
    const LocToLineGrid& loc_to_line,
    const std::function<int(Point)>& penalty_function)
{
    std::vector<PolygonsPointIndex> near_lines =
        loc_to_line.getNearby(from, loc_to_line.getCellSize());

    Point best(0, 0);

    int64_t closest_dist2_score = std::numeric_limits<int64_t>::max();
    PolygonsPointIndex best_point_poly_idx(nullptr, NO_INDEX, NO_INDEX);
    for (PolygonsPointIndex& point_poly_index : near_lines)
    {
        ConstPolygonRef poly = polygons[point_poly_index.poly_idx];
        const Point& p1 = poly[point_poly_index.point_idx];
        const Point& p2 = poly[(point_poly_index.point_idx + 1) % poly.size()];

        Point closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist2_score = vSize2(from - closest_here) + penalty_function(closest_here);
        if (dist2_score < closest_dist2_score)
        {
            best = closest_here;
            closest_dist2_score = dist2_score;
            best_point_poly_idx = point_poly_index;
        }
    }
    if (best_point_poly_idx.poly_idx == NO_INDEX)
    {
        return std::optional<ClosestPolygonPoint>();
    }
    else
    {
        return std::optional<ClosestPolygonPoint>(std::in_place, best, best_point_poly_idx.point_idx, polygons[best_point_poly_idx.poly_idx], best_point_poly_idx.poly_idx);
    }
}

std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> PolygonUtils::findClose(
    ConstPolygonRef from, const Polygons& destination,
    const LocToLineGrid& destination_loc_to_line,
    const std::function<int(Point)>& penalty_function)
{
    std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> ret;
    int p0_idx = from.size() - 1;
    Point p0(from[p0_idx]);
    int grid_size = destination_loc_to_line.getCellSize();
    for (unsigned int p1_idx = 0; p1_idx < from.size(); p1_idx++)
    {
        const Point& p1 = from[p1_idx];
        std::optional<ClosestPolygonPoint> best_here = findClose(p1, destination, destination_loc_to_line, penalty_function);
        if (best_here)
        {
            ret.push_back(std::make_pair(ClosestPolygonPoint(p1, p1_idx, from), *best_here));
        }
        Point p0p1 = p1 - p0;
        int dist_to_p1 = vSize(p0p1);
        for (unsigned int middle_point_nr = 1; dist_to_p1 > grid_size * 2; ++middle_point_nr)
        {
            Point x = p0 + normal(p0p1, middle_point_nr * grid_size);
            dist_to_p1 -= grid_size;

            std::optional<ClosestPolygonPoint> best_here = findClose(x, destination, destination_loc_to_line, penalty_function);
            if (best_here)
            {
                ret.push_back(std::make_pair(ClosestPolygonPoint(x, p0_idx, from), *best_here));
            }
        }
        p0 = p1;
        p0_idx = p1_idx;
    }
    return ret;
}

bool PolygonUtils::getNextPointWithDistance(Point from, int64_t dist, ConstPolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result)
{

    Point prev_poly_point = poly[(start_idx + poly_start_idx) % poly.size()];

    for (unsigned int prev_idx = start_idx; prev_idx < poly.size(); prev_idx++)
    {
        int next_idx = (prev_idx + 1 + poly_start_idx) % poly.size(); // last checked segment is between last point in poly and poly[0]...
        const Point& next_poly_point = poly[next_idx];
        if ( !shorterThen(next_poly_point - from, dist) )
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

            Point pn = next_poly_point - prev_poly_point;

            if (shorterThen(pn, 100)) // when precision is limited
            {
                Point middle = (next_poly_point + prev_poly_point) / 2;
                coord_t dist_to_middle = vSize(from - middle);
                if (dist_to_middle - dist < 100 && dist_to_middle - dist > -100)
                {
                    result.location = middle;
                    result.pos = prev_idx;
                    return true;
                } else
                {
                    prev_poly_point = next_poly_point;
                    continue;
                }
            }

            Point pf = from - prev_poly_point;
            Point px = dot(pf, pn) / vSize(pn) * pn / vSize(pn);
            Point xf = pf - px;

            if (!shorterThen(xf, dist)) // line lies wholly further than pn
            {
                prev_poly_point = next_poly_point;
                continue;

            }

            int64_t xr_dist = std::sqrt(dist*dist - vSize2(xf)); // inverse Pythagoras

            if (vSize(pn - px) - xr_dist < 1) // r lies beyond n
            {
                prev_poly_point = next_poly_point;
                continue;
            }

            Point xr = xr_dist * pn / vSize(pn);
            Point pr = px + xr;

            result.location = prev_poly_point + pr;
            result.pos = prev_idx;
            return true;
        }
        prev_poly_point = next_poly_point;
    }
    return false;
}

ClosestPolygonPoint PolygonUtils::walk(const ClosestPolygonPoint& from, coord_t distance)
{
    ConstPolygonRef poly = *from.poly;
    Point last_vertex = from.p();
    Point next_vertex;
    size_t last_point_idx = from.point_idx;
    for (size_t point_idx = from.point_idx + 1; ; point_idx++)
    {
        if (point_idx == poly.size())
        {
            point_idx = 0;
        }
        next_vertex = poly[point_idx];
        distance -= vSize(last_vertex - next_vertex);
        if (distance <= 0) break;
        last_vertex = next_vertex;
        last_point_idx = point_idx;
    }
    Point result = next_vertex + normal(last_vertex - next_vertex, -distance);
    return ClosestPolygonPoint(result, last_point_idx, poly, from.poly_idx);
}

std::optional<ClosestPolygonPoint> PolygonUtils::getNextParallelIntersection(const ClosestPolygonPoint& start, const Point& line_to, const coord_t dist, const bool forward)
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

    ConstPolygonRef poly = *start.poly;
    const Point s = start.p();
    const Point t = line_to;

    const Point st = t - s;
    const Point shift = normal(turn90CCW(st), dist);

    Point prev_vert = s;
    coord_t prev_projected = 0;
    for (unsigned int next_point_nr = 0; next_point_nr < poly.size(); next_point_nr++)
    {
        const unsigned int next_point_idx =
            forward ?
                (start.point_idx + 1 + next_point_nr) % poly.size()
                : (static_cast<size_t>(start.point_idx) - next_point_nr + poly.size()) % poly.size(); // cast in order to accomodate subtracting
        const Point next_vert = poly[next_point_idx];
        const Point so = next_vert - s;
        const coord_t projected = dot(shift, so) / dist;
        if (std::abs(projected) > dist)
        { // segment crosses the line through xy (or the one on the other side of st)
            const Point segment_vector = next_vert - prev_vert;
            const coord_t segment_length = vSize(segment_vector);
            const coord_t projected_segment_length = std::abs(projected - prev_projected);
            const int16_t sign = (projected > 0) ? 1 : -1;
            const coord_t projected_inter_segment_length = dist - sign * prev_projected; // add the prev_projected to dist if it is projected to the other side of the input line than where the intersection occurs.
            const coord_t inter_segment_length = segment_length * projected_inter_segment_length / projected_segment_length;
            const Point intersection = prev_vert + normal(next_vert - prev_vert, inter_segment_length);

            size_t vert_before_idx = next_point_idx;
            if (forward)
            {
                vert_before_idx = (next_point_idx > 0) ? vert_before_idx - 1 : poly.size() - 1;
            }
            assert(vert_before_idx < poly.size());
            return ClosestPolygonPoint(intersection, vert_before_idx, poly);
        }

        prev_vert = next_vert;
        prev_projected = projected;
    }

    return std::optional<ClosestPolygonPoint>();
}


bool PolygonUtils::polygonCollidesWithLineSegment(const Point from, const Point to, const LocToLineGrid& loc_to_line, PolygonsPointIndex* collision_result)
{
    bool ret = false;
    Point diff = to - from;
    if (vSize2(diff) < 2)
    { // transformation matrix would fail
        return false;
    }

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_from = transformation_matrix.apply(from);
    Point transformed_to = transformation_matrix.apply(to);

    PolygonsPointIndex result;

    std::function<bool (const PolygonsPointIndex&)> process_elem_func =
        [transformed_from, transformed_to, &transformation_matrix, &result, &ret]
        (const PolygonsPointIndex& line_start)
        {
            Point p0 = transformation_matrix.apply(line_start.p());
            Point p1 = transformation_matrix.apply(line_start.next().p());

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

bool PolygonUtils::polygonCollidesWithLineSegment(ConstPolygonRef poly, const Point& transformed_startPoint, const Point& transformed_endPoint, PointMatrix transformation_matrix)
{
    Point p0 = transformation_matrix.apply(poly.back());
    for(Point p1_ : poly)
    {
        Point p1 = transformation_matrix.apply(p1_);
        if (LinearAlg2D::lineSegmentsCollide(transformed_startPoint, transformed_endPoint, p0, p1))
        {
            return true;
        }
        p0 = p1;
    }
    return false;
}

bool PolygonUtils::polygonCollidesWithLineSegment(ConstPolygonRef poly, const Point& startPoint, const Point& endPoint)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return PolygonUtils::polygonCollidesWithLineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix);
}

bool PolygonUtils::polygonCollidesWithLineSegment(const Polygons& polys, const Point& transformed_startPoint, const Point& transformed_endPoint, PointMatrix transformation_matrix)
{
    for (ConstPolygonRef poly : polys)
    {
        if (poly.size() == 0) { continue; }
        if (PolygonUtils::polygonCollidesWithLineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix))
        {
            return true;
        }
    }

    return false;
}


bool PolygonUtils::polygonCollidesWithLineSegment(const Polygons& polys, const Point& startPoint, const Point& endPoint)
{
    if(endPoint == startPoint)
    {
        return false; //Zero-length line segments never collide.
    }
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return polygonCollidesWithLineSegment(polys, transformed_startPoint, transformed_endPoint, transformation_matrix);
}

bool PolygonUtils::polygonsIntersect(const ConstPolygonRef& poly_a, const ConstPolygonRef& poly_b)
{
    // only do the full intersection when the polys' BBs overlap
    AABB bba(poly_a);
    AABB bbb(poly_b);
    return bba.hit(bbb) && poly_a.intersection(poly_b).size() > 0;
}

bool PolygonUtils::polygonOutlinesAdjacent(const ConstPolygonRef inner_poly, const ConstPolygonRef outer_poly, const coord_t max_gap)
{
    //Heuristic check if their AABBs are near first.
    AABB inner_aabb(inner_poly);
    AABB outer_aabb(outer_poly);
    inner_aabb.max += Point(max_gap, max_gap); //Expand one of them by way of a "distance" by checking intersection with the expanded rectangle.
    inner_aabb.min -= Point(max_gap, max_gap);
    if (!inner_aabb.hit(outer_aabb))
    {
        return false;
    }

    //Heuristic says they are near. Now check for real.
    const coord_t max_gap2 = max_gap * max_gap;
    const unsigned outer_poly_size = outer_poly.size();
    for (unsigned line_index = 0; line_index < outer_poly_size; ++line_index)
    {
        const Point lp0 = outer_poly[line_index];
        const Point lp1 = outer_poly[(line_index + 1) % outer_poly_size];
        for (Point inner_poly_point : inner_poly)
        {
            if (LinearAlg2D::getDist2FromLineSegment(lp0, inner_poly_point, lp1) < max_gap2)
            {
                return true;
            }
        }
    }
    return false;
}

void PolygonUtils::findAdjacentPolygons(std::vector<unsigned>& adjacent_poly_indices, const ConstPolygonRef& poly, const std::vector<ConstPolygonPointer>& possible_adjacent_polys, const coord_t max_gap)
{
    // given a polygon, and a vector of polygons, return a vector containing the indices of the polygons that are adjacent to the given polygon
    for (unsigned poly_idx = 0; poly_idx < possible_adjacent_polys.size(); ++poly_idx)
    {
        if (polygonOutlinesAdjacent(poly, *possible_adjacent_polys[poly_idx], max_gap) ||
            polygonOutlinesAdjacent(*possible_adjacent_polys[poly_idx], poly, max_gap))
        {
            adjacent_poly_indices.push_back(poly_idx);
        }
    }
}

double PolygonUtils::relativeHammingDistance(const Polygons& poly_a, const Polygons& poly_b)
{
    const double area_a = std::abs(poly_a.area());
    const double area_b = std::abs(poly_b.area());
    const double total_area = area_a + area_b;

    //If the total area is 0.0, we'd get a division by zero. Instead, only return 0.0 if they are exactly equal.
    constexpr bool borders_allowed = true;
    if(total_area == 0.0)
    {
        for(const ConstPolygonRef& polygon_a : poly_a)
        {
            for(Point point : polygon_a)
            {
                if(!poly_b.inside(point, borders_allowed))
                {
                    return 1.0;
                }
            }
        }
        for(const ConstPolygonRef& polygon_b : poly_b)
        {
            for(Point point : polygon_b)
            {
                if(!poly_a.inside(point, borders_allowed))
                {
                    return 1.0;
                }
            }
        }
        return 0.0; //All points are inside the other polygon, regardless of where the vertices are along the edges.
    }

    const Polygons symmetric_difference = poly_a.xorPolygons(poly_b);
    const double hamming_distance = symmetric_difference.area();
    return hamming_distance / total_area;
}

Polygon PolygonUtils::makeCircle(const Point mid, const coord_t radius, const AngleRadians a_step)
{
    Polygon circle;
    for (float a = 0; a < 2 * M_PI; a += a_step)
    {
        circle.emplace_back(mid + Point(radius * cos(a), radius * sin(a)));
    }
    return circle;
}


Polygons PolygonUtils::connect(const Polygons& input)
{
    Polygons ret;
    std::vector<PolygonsPart> parts = input.splitIntoParts(true);
    for (PolygonsPart& part : parts)
    {
        PolygonRef outline = part.outerPolygon();
        for (size_t hole_idx = 1; hole_idx < part.size(); hole_idx++)
        {
            PolygonRef hole = part[hole_idx];
            Point hole_point = hole[0];
            hole.add(hole_point);
            // find where the scanline passes the Y
            size_t best_segment_to_idx = 0;
            coord_t best_dist = std::numeric_limits<coord_t>::max();
            Point best_intersection_point = outline.back();

            Point prev = outline.back();
            for (size_t point_idx = 0; point_idx < outline.size(); point_idx++)
            {
                Point here = outline[point_idx];
                if (here.Y > hole_point.Y && prev.Y <= hole_point.Y && here.Y != prev.Y)
                {
                    Point intersection_point = prev + (here - prev) * (hole_point.Y - prev.Y) / (here.Y - prev.Y);
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
            (*outline).insert(outline.begin() + best_segment_to_idx, 2, best_intersection_point);
            (*outline).insert(outline.begin() + best_segment_to_idx + 1, hole.begin(), hole.end());
        }
        ret.add(outline);
    }
    return ret;
}

/* Note: Also tries to solve for near-self intersections, when epsilon >= 1
 */
void PolygonUtils::fixSelfIntersections(const coord_t epsilon, Polygons& thiss)
{
    if (epsilon < 1)
    {
        ClipperLib::SimplifyPolygons(thiss.paths);
        return;
    }

    const coord_t half_epsilon = (epsilon + 1) / 2;

    // Points too close to line segments should be moved a little away from those line segments, but less than epsilon,
    //   so at least half-epsilon distance between points can still be guaranteed.
    constexpr coord_t grid_size = 2000;
    auto query_grid = PolygonUtils::createLocToLineGrid(thiss, grid_size);

    const coord_t move_dist = std::max(2LL, half_epsilon - 2);
    const coord_t half_epsilon_sqrd = half_epsilon * half_epsilon;

    const size_t n = thiss.size();
    for (size_t poly_idx = 0; poly_idx < n; poly_idx++)
    {
        const size_t pathlen = thiss[poly_idx].size();
        for (size_t point_idx = 0; point_idx < pathlen; ++point_idx)
        {
            Point& pt = thiss[poly_idx][point_idx];
            for (const auto& line : query_grid->getNearby(pt, epsilon))
            {
                const size_t line_next_idx = (line.point_idx + 1) % thiss[line.poly_idx].size();
                if (poly_idx == line.poly_idx && (point_idx == line.point_idx || point_idx == line_next_idx))
                {
                    continue;
                }

                const Point& a = thiss[line.poly_idx][line.point_idx];
                const Point& b = thiss[line.poly_idx][line_next_idx];

                if (half_epsilon_sqrd >= vSize2(pt - LinearAlg2D::getClosestOnLineSegment(pt, a, b)))
                {
                    const Point& other = thiss[poly_idx][(point_idx + 1) % pathlen];
                    const Point vec = LinearAlg2D::pointIsLeftOfLine(other, a, b) > 0 ? b - a : a - b;
                    const coord_t len = vSize(vec);
                    pt.X += (-vec.Y * move_dist) / len;
                    pt.Y += ( vec.X * move_dist) / len;
                }
            }
        }
    }

    ClipperLib::SimplifyPolygons(thiss.paths);
}

}//namespace cura
