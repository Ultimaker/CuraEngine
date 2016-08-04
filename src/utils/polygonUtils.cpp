/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "polygonUtils.h"

#include <list>
#include <sstream>

#include "linearAlg2D.h"
#include "SparsePointGridInclusive.h"

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
    ConstPolygonRef poly = *point_on_boundary.poly;
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
        bool projected_p_beyond_prev_segment = dot(p1 - p0, from - p0) >= vSize2(p1 - p0);
        for(const Point& p2 : poly)
        {   
            // X = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            // X = P projected on AB
            const Point& a = p1;
            const Point& b = p2;
            const Point& p = from;
            Point ab = b - a;
            Point ap = p - a;
            int64_t ab_length = vSize(ab);
            if(ab_length <= 0) //A = B, i.e. the input polygon had two adjacent points on top of each other.
            {
                p1 = p2; //Skip only one of the points.
                continue;
            }
            int64_t ax_length = dot(ab, ap) / ab_length;
            if (ax_length <= 0) // x is projected to before ab
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
            else if (ax_length >= ab_length) // x is projected to beyond ab
            {
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            }
            else 
            { // x is projected to a point properly on the line segment (not onto a vertex). The case which looks like | .
                projected_p_beyond_prev_segment = false;
                Point x = a + ab * ax_length / ab_length;
                
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
    ClosestPolygonPoint closest_polygon_point = moveInside2(polygons, from, preferred_dist_inside, max_dist2, loc_to_line_polygons, loc_to_line_grid, penalty_function);
    return ensureInsideOrOutside(polygons, from, closest_polygon_point, preferred_dist_inside, max_dist2, loc_to_line_polygons, loc_to_line_grid, penalty_function);
}

ClosestPolygonPoint PolygonUtils::ensureInsideOrOutside(const Polygons& polygons, Point& from, ClosestPolygonPoint& closest_polygon_point, int preferred_dist_inside, int64_t max_dist2, const Polygons* loc_to_line_polygons, const LocToLineGrid* loc_to_line_grid, const std::function<int(Point)>& penalty_function)
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
    int64_t max_dist2_here = std::numeric_limits<int64_t>::max(); // we already concluded we are close enough to the closest_poly
    moveInside2(*loc_to_line_polygons, closest_poly, from, preferred_dist_inside / 2, max_dist2_here, loc_to_line_grid, penalty_function);
    bool is_inside = closest_poly.inside(from) == is_outside_boundary; // inside a hole is outside the part
    if (is_inside == (preferred_dist_inside > 0))
    { // we ended up on the right side of the polygon
        // assume we didn't overshoot another polygon in [polygons]
        return closest_polygon_point;
    }
    // if above fails, we perform an offset and sit directly on the offsetted polygon (and keep the result from the above moveInside)
    else
    {
        int offset = (is_outside_boundary)? -preferred_dist_inside : preferred_dist_inside; // perform inset on outer boundary and outset on holes
        Polygons insetted = closest_poly.offset(offset / 2); // perform less inset, because chances are (thin parts of) the polygon will disappear, given that moveInside did an overshoot
        if (insetted.size() == 0)
        {
            return ClosestPolygonPoint(); // we couldn't move inside
        }
        ClosestPolygonPoint inside = findClosest(from, insetted, penalty_function);
        if (inside.isValid())
        {
            bool is_inside = polygons.inside(inside.location) == is_outside_boundary; // inside a hole is outside the part
            if (is_inside != (preferred_dist_inside > 0))
            {
                /*
                 * somehow the insetted polygon is not inside of [closest_poly]
                 * Clipper seems to fuck up sometimes.
                 */
#ifdef DEBUG
                try
                {
                    int offset_performed = offset / 2;
                    AABB aabb(insetted);
                    aabb.expand(std::abs(preferred_dist_inside) * 2);
                    SVG svg("debug.html", aabb);
                    svg.writeComment("Original polygon in black");
                    svg.writePolygon(closest_poly, SVG::Color::BLACK);
                    for (auto point : closest_poly)
                    {
                        svg.writePoint(point, true, 2);
                    }
                    std::stringstream ss;
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
            from = inside.location;
        } // otherwise we just return the closest polygon point without modifying the from location
        return closest_polygon_point; // don't return a ClosestPolygonPoint with a reference to the above local polygons variable
    }
}


void PolygonUtils::findSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result, int sample_size)
{
    if (!poly1_result.isValid() || !poly2_result.isValid())
    {
        return;
    }
    ConstPolygonRef poly1 = *poly1_result.poly;
    ConstPolygonRef poly2 = *poly2_result.poly;
    if (poly1.size() == 0 || poly2.size() == 0)
    {
        return;
    }
    
    int bestDist2 = -1;
    
    int step1 = std::max<unsigned int>(2, poly1.size() / sample_size);
    int step2 = std::max<unsigned int>(2, poly2.size() / sample_size);
    for (unsigned int i = 0; i < poly1.size(); i += step1)
    {
        for (unsigned int j = 0; j < poly2.size(); j += step2)
        {   
            int dist2 = vSize2(poly1[i] - poly2[j]);
            if (bestDist2 == -1 || dist2 < bestDist2)
            {   
                bestDist2 = dist2;
                poly1_result.point_idx = i;
                poly2_result.point_idx = j;
            }
        }
    }
    
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
    if (poly1_result.point_idx < 0 || poly2_result.point_idx < 0)
    {
        return;
    }
    
    int equilibirum_limit = 100; // hard coded value
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

    for (unsigned int p = 0; p<polygon.size(); p++)
    {
        int p1_idx = (polygon.size() + direction*p + start_idx) % polygon.size();
        int p2_idx = (polygon.size() + direction*(p+1) + start_idx) % polygon.size();
        const Point& p1 = polygon[p1_idx];
        const Point& p2 = polygon[p2_idx];

        Point closest_here = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist = vSize2(from - closest_here);
        if (dist < closestDist)
        {
            best = closest_here;
            closestDist = dist;
            bestPos = p1_idx;
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
    ConstPolygonRef any_polygon = polygons[0];
    unsigned int any_poly_idx;
    for (any_poly_idx = 0; any_poly_idx < polygons.size(); any_poly_idx++)
    { // find first point in all polygons
        if (polygons[any_poly_idx].size() > 0)
        {
            any_polygon = polygons[any_poly_idx];
            break;
        }
    }
    if (any_polygon.size() == 0)
    {
        return none;
    }
    ClosestPolygonPoint best(any_polygon[0], 0, any_polygon, any_poly_idx);

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
//
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

unsigned int PolygonUtils::findNearestVert(const Point from, const PolygonRef poly)
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


LocToLineGrid* PolygonUtils::createLocToLineGrid(const Polygons& polygons, int square_size)
{
    unsigned int n_points = 0;
    for (const auto& poly : polygons)
    {
        n_points += poly.size();
    }

    LocToLineGrid* ret = new LocToLineGrid(square_size, n_points);

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
 * The current implemetnation can check the same line segment multiple times, 
 * since the same line segment can occur in multiple cells if it it longer than the cell size of the SparsePointGridInclusive.
 * 
 * We could skip the duplication by keeping a vector of vectors of bools.
 *
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
        bool bs_arg = true; // doesn't mean anything. Just to make clear we call the variable arguments of the constructor.
        return std::optional<ClosestPolygonPoint>(bs_arg, best, best_point_poly_idx.point_idx, polygons[best_point_poly_idx.poly_idx], best_point_poly_idx.poly_idx);
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
                int64_t dist_to_middle = vSize(from - middle);
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

bool PolygonUtils::polygonCollidesWithLineSegment(PolygonRef poly, const Point& startPoint, const Point& endPoint)
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
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return polygonCollidesWithLineSegment(polys, transformed_startPoint, transformed_endPoint, transformation_matrix);
}


}//namespace cura
