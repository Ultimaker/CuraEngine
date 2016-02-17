/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "polygonUtils.h"

#include <list>

#include "linearAlg2D.h"
#include "BucketGrid2D.h"
#include "../debug.h"


namespace cura 
{

int64_t offset_safe_allowance = 20; // make all offset safe operations a bit less safe to allow for small variations in walls which are supposed to be exactly x perimeters thick
int64_t in_between_min_dist_half = 10;

void PolygonUtils::offsetExtrusionWidth(const Polygons& poly, bool inward, int extrusionWidth, Polygons& result, Polygons* in_between, bool removeOverlappingPerimeters)
{
    int direction = (inward)? -1 : 1;
    int distance = (inward)? -extrusionWidth : extrusionWidth;
    if (!removeOverlappingPerimeters)
    {
        result = poly.offset(distance);
        return;
    } 
    else
    {
        result = poly.offset(distance*3/2 - direction*offset_safe_allowance).offset(-distance/2 + direction*offset_safe_allowance); // overshoot by half the extrusionWidth
        if (in_between) // if a pointer for in_between is given
            in_between->add(poly.offset(distance/2 + direction*in_between_min_dist_half).difference(result.offset(-distance/2 - direction*in_between_min_dist_half)));
    }
}

void PolygonUtils::offsetSafe(const Polygons& poly, int distance, int offset_first_boundary, int extrusion_width, Polygons& result, Polygons* in_between, bool removeOverlappingPerimeters)
{
    int direction = (distance > 0)? 1 : -1;
    if (!removeOverlappingPerimeters)
    {
        result = poly.offset(distance);
        return;
    } 
    else
    {
        result = poly.offset(distance + direction*extrusion_width / 2 - direction*offset_safe_allowance).offset(-direction*extrusion_width/2 + direction*offset_safe_allowance); // overshoot by half the extrusionWidth
        if (in_between) // if a pointer for in_between is given
            in_between->add(poly.offset(offset_first_boundary + direction*in_between_min_dist_half).difference(result.offset(-direction * extrusion_width/2 - direction*in_between_min_dist_half)));
    }
}


void PolygonUtils::offsetSafe(const Polygons& poly, int distance, int extrusionWidth, Polygons& result, bool removeOverlappingPerimeters)
{
    int direction = (distance > 0)? 1 : -1;
    if (!removeOverlappingPerimeters)
    {
        result = poly.offset(distance);
        return;
    } 
    else
    {
        result = poly.offset(distance + direction*extrusionWidth/2 - direction*offset_safe_allowance).offset(-direction * extrusionWidth/2 + direction*offset_safe_allowance);
    }
}

void PolygonUtils::removeOverlapping(const Polygons& poly, int extrusionWidth, Polygons& result)
{
    result = poly.offset(extrusionWidth/2).offset(-extrusionWidth).offset(extrusionWidth/2);
}

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

    Point off0 = crossZ(normal(p1 - p0, MM2INT(10.0))); // 10.0 for some precision
    Point off1 = crossZ(normal(p2 - p1, MM2INT(10.0))); // 10.0 for some precision
    Point n = normal(off0 + off1, -offset);

    return p1 + n;
}

/*
 * Implementation assumes moving inside, but moving outside should just as well be possible.
 */
unsigned int PolygonUtils::moveInside(Polygons& polygons, Point& from, int distance, int64_t maxDist2)
{
    Point ret = from;
    int64_t bestDist2 = std::numeric_limits<int64_t>::max();
    unsigned int bestPoly = NO_INDEX;
    bool is_inside = false;
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        PolygonRef poly = polygons[poly_idx];
        if (poly.size() < 2)
            continue;
        Point p0 = poly[poly.size()-2];
        Point p1 = poly.back();
        bool projected_p_beyond_prev_segment = dot(p1 - p0, from - p0) > vSize2(p1 - p0);
        for(Point& p2 : poly)
        {   
            // X = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            // X = P projected on AB
            Point& a = p1;
            Point& b = p2;
            Point& p = from;
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
                            Point inward_dir = crossZ(normal(ab, MM2INT(10.0)) + normal(p1 - p0, MM2INT(10.0))); // MM2INT(10.0) to retain precision for the eventual normalization 
                            ret = x + normal(inward_dir, distance);
                            is_inside = dot(inward_dir, p - x) >= 0;
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
            {
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
                        Point inward_dir = crossZ(normal(ab, distance));
                        ret = x + inward_dir; 
                        is_inside = dot(inward_dir, p - x) >= 0;
                    }
                }
            }
            
            
            p0 = p1;
            p1 = p2;
        }
    }
    if (is_inside)
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

Point PolygonUtils::moveInside(const ClosestPolygonPoint& cpp, const int distance)
{
    if (distance == 0)
    { // the point which is assumed to be on the boundary doesn't have to be moved
        return cpp.location;
    }
    const PolygonRef poly = cpp.poly;
    unsigned int point_idx = cpp.pos;
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
        
        Point inward_dir = crossZ(normal(p2 - p1, distance));
        return x + inward_dir; 
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
                poly1_result.pos = i;
                poly2_result.pos = j;
            }
        }
    }
    
    walkToNearestSmallestConnection(poly1_result, poly2_result);    
}

void PolygonUtils::walkToNearestSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result)
{
    PolygonRef poly1 = poly1_result.poly;
    PolygonRef poly2 = poly2_result.poly;
    if (poly1_result.pos < 0 || poly2_result.pos < 0)
    {
        return;
    }
    
    int equilibirum_limit = 100; // hard coded value
    for (int loop_counter = 0; loop_counter < equilibirum_limit; loop_counter++)
    {
        int pos1_before = poly1_result.pos;
        poly1_result = findNearestClosest(poly2_result.location, poly1, poly1_result.pos);
        int pos2_before = poly2_result.pos;
        poly2_result = findNearestClosest(poly1_result.location, poly2, poly2_result.pos);
       
        if (poly1_result.pos == pos1_before && poly2_result.pos == pos2_before)
        {
            break;
        }
    }
}

ClosestPolygonPoint PolygonUtils::findNearestClosest(Point from, PolygonRef polygon, int start_idx)
{
    ClosestPolygonPoint forth = findNearestClosest(from, polygon, start_idx, 1);
    ClosestPolygonPoint back = findNearestClosest(from, polygon, start_idx, -1);
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

        Point closestHere = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist = vSize2(from - closestHere);
        if (dist < closestDist)
        {
            best = closestHere;
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

ClosestPolygonPoint PolygonUtils::findClosest(Point from, Polygons& polygons)
{

    Polygon emptyPoly;
    ClosestPolygonPoint none(from, -1, emptyPoly);
    
    if (polygons.size() == 0) return none;
    PolygonRef aPolygon = polygons[0];
    if (aPolygon.size() == 0) return none;
    Point aPoint = aPolygon[0];

    ClosestPolygonPoint best(aPoint, 0, aPolygon);

    int64_t closestDist = vSize2(from - best.location);
    
    for (unsigned int ply = 0; ply < polygons.size(); ply++)
    {
        PolygonRef poly = polygons[ply];
        if (poly.size() == 0) continue;
        ClosestPolygonPoint closestHere = findClosest(from, poly);
        int64_t dist = vSize2(from - closestHere.location);
        if (dist < closestDist)
        {
            best = closestHere;
            closestDist = dist;
        }

    }

    return best;
}

ClosestPolygonPoint PolygonUtils::findClosest(Point from, PolygonRef polygon)
{
    if (polygon.size() == 0)
    {
        return ClosestPolygonPoint(polygon);
    }
    Point aPoint = polygon[0];
    Point best = aPoint;

    int64_t closestDist = vSize2(from - best);
    int bestPos = 0;
//
    for (unsigned int p = 0; p<polygon.size(); p++)
    {
        Point& p1 = polygon[p];

        unsigned int p2_idx = p+1;
        if (p2_idx >= polygon.size()) p2_idx = 0;
        Point& p2 = polygon[p2_idx];

        Point closestHere = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist = vSize2(from - closestHere);
        if (dist < closestDist)
        {
            best = closestHere;
            closestDist = dist;
            bestPos = p;
        }
    }

    return ClosestPolygonPoint(best, bestPos, polygon);
}

BucketGrid2D<PolygonsPointIndex>* PolygonUtils::createLocToLineGrid(const Polygons& polygons, int square_size)
{
    unsigned int n_points = 0;
    for (const PolygonRef poly : const_cast<Polygons&>(polygons))
    {
        n_points += poly.size();
    }

    BucketGrid2D<PolygonsPointIndex>* ret = new BucketGrid2D<PolygonsPointIndex>(square_size, n_points);

    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        const PolygonRef poly = const_cast<Polygons&>(polygons)[poly_idx];
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
 * since the same line segment can occur in multiple cells if it it longer than the cell size of the BucketGrid.
 * 
 * We could skip the duplication by keeping a vector of vectors of bools.
 *
 */
ClosestPolygonPoint* PolygonUtils::findClose(Point from, const Polygons& polygons, BucketGrid2D<PolygonsPointIndex> loc_to_line)
{
    std::vector<PolygonsPointIndex> near_lines;
    loc_to_line.findNearbyObjects(from, near_lines);

    const Point arbitrary_point = const_cast<Polygons&>(polygons)[0][0];
    Point best = arbitrary_point;

    int64_t closest_dist2 = vSize2(from - best);
    PolygonsPointIndex best_point_poly_idx(NO_INDEX, NO_INDEX);
    for (PolygonsPointIndex& point_poly_index : near_lines)
    {
        const PolygonRef poly = const_cast<Polygons&>(polygons)[point_poly_index.poly_idx];
        Point& p1 = poly[point_poly_index.point_idx];
        Point& p2 = poly[(point_poly_index.point_idx + 1) % poly.size()];

        Point closestHere = LinearAlg2D::getClosestOnLineSegment(from, p1 ,p2);
        int64_t dist = vSize2(from - closestHere);
        if (dist < closest_dist2)
        {
            best = closestHere;
            closest_dist2 = dist;
            best_point_poly_idx = point_poly_index;
        }
    }
    if (best_point_poly_idx.poly_idx == NO_INDEX)
    {
        return nullptr;
    }
    else
    {
        return new ClosestPolygonPoint(best, best_point_poly_idx.point_idx, const_cast<Polygons&>(polygons)[best_point_poly_idx.poly_idx]);
    }
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



bool PolygonUtils::polygonCollidesWithlineSegment(PolygonRef poly, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix)
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

bool PolygonUtils::polygonCollidesWithlineSegment(PolygonRef poly, Point& startPoint, Point& endPoint)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return PolygonUtils::polygonCollidesWithlineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix);
}

bool PolygonUtils::polygonCollidesWithlineSegment(Polygons& polys, Point& transformed_startPoint, Point& transformed_endPoint, PointMatrix transformation_matrix)
{
    for(PolygonRef poly : polys)
    {
        if (poly.size() == 0) { continue; }
        if (PolygonUtils::polygonCollidesWithlineSegment(poly, transformed_startPoint, transformed_endPoint, transformation_matrix))
        {
            return true;
        }
    }
    
    return false;
}


bool PolygonUtils::polygonCollidesWithlineSegment(Polygons& polys, Point& startPoint, Point& endPoint)
{
    Point diff = endPoint - startPoint;

    PointMatrix transformation_matrix = PointMatrix(diff);
    Point transformed_startPoint = transformation_matrix.apply(startPoint);
    Point transformed_endPoint = transformation_matrix.apply(endPoint);

    return polygonCollidesWithlineSegment(polys, transformed_startPoint, transformed_endPoint, transformation_matrix);
}


}//namespace cura
