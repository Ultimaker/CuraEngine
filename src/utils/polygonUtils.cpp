/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "polygonUtils.h"

#include <list>

#include "linearAlg2D.h"
#include "SparseGrid.h"
#include "../debug.h"

#ifdef DEBUG
#include "AABB.h"
#include "SVG.h"
#endif

namespace cura 
{

const std::function<int(Point)> PolygonUtils::no_penalty_function = [](Point){ return 0; };

Point PolygonUtils::getBoundaryPointWithOffset(PolygonRef poly, unsigned int point_idx, int64_t offset)
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
    Point& p2 = poly[p2_idx];

    Point off0 = turn90CCW(normal(p1 - p0, MM2INT(10.0))); // 10.0 for some precision
    Point off1 = turn90CCW(normal(p2 - p1, MM2INT(10.0))); // 10.0 for some precision
    Point n = normal(off0 + off1, -offset);

    return p1 + n;
}

unsigned int PolygonUtils::moveOutside(const Polygons& polygons, Point& from, int distance, int64_t maxDist2)
{
    return moveInside(polygons, from, -distance, maxDist2);
}

ClosestPolygonPoint PolygonUtils::moveInside2(const Polygons& polygons, Point& from, const int distance, const int64_t max_dist2, const std::function<int(Point)>& penalty_function)
{
    const ClosestPolygonPoint closest_polygon_point = findClosest(from, polygons, penalty_function);
    return _moveInside2(closest_polygon_point, distance, from, max_dist2);
}

ClosestPolygonPoint PolygonUtils::moveInside2(const PolygonRef polygon, Point& from, const int distance, const int64_t max_dist2, const std::function<int(Point)>& penalty_function)
{
    const ClosestPolygonPoint closest_polygon_point = findClosest(from, polygon, penalty_function);
    return _moveInside2(closest_polygon_point, distance, from, max_dist2);
}

ClosestPolygonPoint PolygonUtils::_moveInside2(const ClosestPolygonPoint& closest_polygon_point, const int distance, Point& from, const int64_t max_dist2)
{
    if (closest_polygon_point.point_idx == NO_INDEX)
    {
        return ClosestPolygonPoint(closest_polygon_point.poly); // stub with invalid indices to signify we haven't found any
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
            return ClosestPolygonPoint(closest_polygon_point.poly); // stub with invalid indices to signify we haven't found any
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
        const PolygonRef poly = polygons[poly_idx];
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
    if (distance == 0)
    { // the point which is assumed to be on the boundary doesn't have to be moved
        return cpp.location;
    }
    const PolygonRef poly = cpp.poly;
    unsigned int point_idx = cpp.point_idx;
    const Point& on_boundary = cpp.location;

    Point& p1 = poly[point_idx];
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
    Point& p2 = poly[p2_idx];

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

ClosestPolygonPoint PolygonUtils::ensureInsideOrOutside(const Polygons& polygons, Point& from, int preferred_dist_inside, int64_t max_dist2, const std::function<int(Point)>& penalty_function)
{
    ClosestPolygonPoint closest_polygon_point = moveInside2(polygons, from, preferred_dist_inside, max_dist2, penalty_function);
    if (closest_polygon_point.point_idx == NO_INDEX)
    {
        return ClosestPolygonPoint(polygons[0]); // we couldn't move inside
    }
    PolygonRef closest_poly = closest_polygon_point.poly;
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
    moveInside2(closest_poly, from, preferred_dist_inside / 2, max_dist2_here, penalty_function);
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
            return ClosestPolygonPoint(polygons[0]); // we couldn't move inside
        }
        ClosestPolygonPoint inside = findClosest(from, insetted, penalty_function);
        if (inside.point_idx != NO_INDEX)
        {
            bool is_inside = polygons.inside(inside.location) == is_outside_boundary; // inside a hole is outside the part
            if (is_inside != (preferred_dist_inside > 0))
            {
                /*
                 * somehow the insetted polygon is not inside of [closest_poly]
                 * Clipper seems to fuck up sometimes.
                 */
#ifdef DEBUG
                {
                    AABB aabb(insetted);
                    aabb.expand(std::abs(preferred_dist_inside) * 2);
                    SVG svg("debug.html", aabb);
                    svg.writePolygon(closest_poly, SVG::Color::BLACK);
                    for (auto point : closest_poly)
                    {
                        svg.writePoint(point, true, 2);
                    }
                    svg.writePolygons(insetted, SVG::Color::BLUE);
                    for (auto poly : insetted)
                    {
                        for (auto point : poly)
                        {
                            svg.writePoint(point, true, 2);
                        }
                    }
                    svg.writePoint(from, false, 5, SVG::Color::GREEN);
                    svg.writePoint(inside.location, false, 5, SVG::Color::RED);
                }
                logError("ERROR! ERROR!\n\tClipper::offset failed. See generated debug.html!\n\tBlack is original\n\tBlue is offsetted polygon\n");
#endif
                return ClosestPolygonPoint(polygons[0]);
            }
            from = inside.location;
        } // otherwise we just return the closest polygon point without modifying the from location
        return closest_polygon_point; // don't return a ClosestPolygonPoint with a reference to the above local polygons variable
    }
}


void PolygonUtils::findSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result, int sample_size)
{
    PolygonRef poly1 = poly1_result.poly;
    PolygonRef poly2 = poly2_result.poly;
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
    PolygonRef poly1 = poly1_result.poly;
    PolygonRef poly2 = poly2_result.poly;
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

ClosestPolygonPoint PolygonUtils::findNearestClosest(Point from, PolygonRef polygon, int start_idx)
{
    ClosestPolygonPoint forth = findNearestClosest(from, polygon, start_idx, 1);
    if (forth.point_idx == NO_INDEX)
    {
        return forth; // stop computation
    }
    ClosestPolygonPoint back = findNearestClosest(from, polygon, start_idx, -1);
    assert(back.point_idx != NO_INDEX);
    if (vSize2(forth.location - from) < vSize2(back.location - from))
    {
        return forth;
    }
    else
    {
        return back;
    }
}

ClosestPolygonPoint PolygonUtils::findNearestClosest(Point from, PolygonRef polygon, int start_idx, int direction)
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
        Point& p1 = polygon[p1_idx];
        Point& p2 = polygon[p2_idx];

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
    ClosestPolygonPoint none(from, -1, polygons[0], -1);
    
    if (polygons.size() == 0) return none;
    PolygonRef aPolygon = polygons[0];
    if (aPolygon.size() == 0) return none;
    Point aPoint = aPolygon[0];

    ClosestPolygonPoint best(aPoint, 0, aPolygon, 0);

    int64_t closestDist2_score = vSize2(from - best.location) + penalty_function(best.location);
    
    for (unsigned int ply = 0; ply < polygons.size(); ply++)
    {
        const PolygonRef poly = polygons[ply];
        if (poly.size() == 0) continue;
        ClosestPolygonPoint closest_here = findClosest(from, poly, penalty_function);
        if (closest_here.point_idx == NO_INDEX)
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

ClosestPolygonPoint PolygonUtils::findClosest(Point from, const PolygonRef polygon, const std::function<int(Point)>& penalty_function)
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
        Point& p1 = polygon[p];

        unsigned int p2_idx = p+1;
        if (p2_idx >= polygon.size()) p2_idx = 0;
        Point& p2 = polygon[p2_idx];

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

SparseGrid<PolygonsPointIndex>* PolygonUtils::createLocToLineGrid(const Polygons& polygons, int square_size)
{
    unsigned int n_points = 0;
    for (const auto& poly : polygons)
    {
        n_points += poly.size();
    }

    SparseGrid<PolygonsPointIndex>* ret = new SparseGrid<PolygonsPointIndex>(square_size, n_points);

    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        const PolygonRef poly = polygons[poly_idx];
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point& p1 = poly[point_idx];
            Point& p2 = poly[(point_idx + 1) % poly.size()];
            
            ret->insert(p1, PolygonsPointIndex(poly_idx, point_idx));
            Point vec = p2 - p1;
            int64_t vec_length = vSize(vec);
            for (int64_t dist_along_line = square_size; dist_along_line < vec_length; dist_along_line += square_size)
            {
                Point point_along_line = p1 + vec * dist_along_line / vec_length;
                
                ret->insert(point_along_line, PolygonsPointIndex(poly_idx, point_idx));
            }
        }
        
    }
    
    
    
    
    
    return ret;
}

/*
 * The current implemetnation can check the same line segment multiple times, 
 * since the same line segment can occur in multiple cells if it it longer than the cell size of the SparseGrid.
 * 
 * We could skip the duplication by keeping a vector of vectors of bools.
 *
 */
std::optional<ClosestPolygonPoint> PolygonUtils::findClose(
    Point from, const Polygons& polygons,
    const SparseGrid<PolygonsPointIndex>& loc_to_line,
    const std::function<int(Point)>& penalty_function)
{
    std::vector<PolygonsPointIndex> near_lines =
        loc_to_line.getNearbyVals(from, loc_to_line.getCellSize());

    Point best(0, 0);

    int64_t closest_dist2_score = std::numeric_limits<int64_t>::max();
    PolygonsPointIndex best_point_poly_idx(NO_INDEX, NO_INDEX);
    for (PolygonsPointIndex& point_poly_index : near_lines)
    {
        const PolygonRef poly = polygons[point_poly_index.poly_idx];
        Point& p1 = poly[point_poly_index.point_idx];
        Point& p2 = poly[(point_poly_index.point_idx + 1) % poly.size()];

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
        bool bs_arg; // doesn't mean anything. Just to make clear we call the variable arguments of the constructor.
        return std::optional<ClosestPolygonPoint>(bs_arg, best, best_point_poly_idx.point_idx, polygons[best_point_poly_idx.poly_idx], best_point_poly_idx.poly_idx);
    }
}


std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> PolygonUtils::findClose(
    const PolygonRef from, const Polygons& destination,
    const SparseGrid< PolygonsPointIndex >& destination_loc_to_line,
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





bool PolygonUtils::getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result)
{
    
    Point prev_poly_point = poly[(start_idx + poly_start_idx) % poly.size()];
    
    for (unsigned int prev_idx = start_idx; prev_idx < poly.size(); prev_idx++) 
    {
        int next_idx = (prev_idx + 1 + poly_start_idx) % poly.size(); // last checked segment is between last point in poly and poly[0]...
        Point& next_poly_point = poly[next_idx];
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



bool PolygonUtils::polygonCollidesWithlineSegment(const PolygonRef poly, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix)
{
    Point p0 = transformation_matrix.apply(poly.back());
    for(Point p1_ : poly)
    {
        Point p1 = transformation_matrix.apply(p1_);
        if ((p0.Y >= transformed_startPoint.Y && p1.Y <= transformed_startPoint.Y) || (p1.Y >= transformed_startPoint.Y && p0.Y <= transformed_startPoint.Y))
        {
            int64_t x;
            if(p1.Y == p0.Y)
            {
                x = p0.X;
            }
            else
            {
                x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
            }
            
            if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                return true;
        }
        p0 = p1;
    }
    return false;
}

bool PolygonUtils::polygonCollidesWithlineSegment(const PolygonRef poly, Point& startPoint, Point& endPoint)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return PolygonUtils::polygonCollidesWithlineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix);
}

bool PolygonUtils::polygonCollidesWithlineSegment(const Polygons& polys, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix)
{
    for (const PolygonRef poly : const_cast<Polygons&>(polys))
    {
        if (poly.size() == 0) { continue; }
        if (PolygonUtils::polygonCollidesWithlineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix))
        {
            return true;
        }
    }
    
    return false;
}


bool PolygonUtils::polygonCollidesWithlineSegment(const Polygons& polys, Point& startPoint, Point& endPoint)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return polygonCollidesWithlineSegment(polys, transformed_startPoint, transformed_endPoint, transformation_matrix);
}


}//namespace cura
