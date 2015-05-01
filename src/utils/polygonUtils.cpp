/** Copyright (C) 2015 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "polygonUtils.h"

#include <list>

#include "../debug.h"
namespace cura 
{

void offsetExtrusionWidth(Polygons& poly, bool inward, int extrusionWidth, Polygons& result, Polygons* in_between, bool avoidOverlappingPerimeters)
{
    int distance = (inward)? -extrusionWidth : extrusionWidth;
    if (!avoidOverlappingPerimeters)
    {
        result = poly.offset(distance);
        return;
    } 
    else
    {
        result = poly.offset(distance*3/2).offset(-distance/2); // overshoot by half the extrusionWidth
        if (in_between) // if a pointer for in_between is given
            in_between->add(poly.offset(distance/2).difference(result.offset(-distance/2)));
    }
}


void offsetSafe(Polygons& poly, int distance, int extrusionWidth, Polygons& result, bool avoidOverlappingPerimeters)
{
    int direction = (distance > 0)? 1 : -1;
    if (!avoidOverlappingPerimeters)
    {
        result = poly.offset(distance);
        return;
    } 
    else
    {
        result = poly.offset(distance + direction*extrusionWidth/2).offset(-direction * extrusionWidth/2);
    }
}

void removeOverlapping(Polygons& poly, int extrusionWidth, Polygons& result)
{
    result = poly.offset(extrusionWidth/2).offset(-extrusionWidth).offset(extrusionWidth/2);
}




void findSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result, int sample_size)
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

void walkToNearestSmallestConnection(ClosestPolygonPoint& poly1_result, ClosestPolygonPoint& poly2_result)
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

ClosestPolygonPoint findNearestClosest(Point from, PolygonRef polygon, int start_idx)
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

ClosestPolygonPoint findNearestClosest(Point from, PolygonRef polygon, int start_idx, int direction)
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

        Point closestHere = getClosestOnLine(from, p1 ,p2);
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

ClosestPolygonPoint findClosest(Point from, Polygons& polygons)
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

ClosestPolygonPoint findClosest(Point from, PolygonRef polygon)
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

        Point closestHere = getClosestOnLine(from, p1 ,p2);
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


Point getClosestOnLine(Point from, Point p0, Point p1)
{
    Point direction = p1 - p0;
    Point toFrom = from-p0;
    int64_t projected_x = dot(toFrom, direction) ;

    int64_t x_p0 = 0;
    int64_t x_p1 = vSize2(direction);

    if (projected_x <= x_p0)
    {
        return p0;
    }
    if (projected_x >= x_p1)
    {
        return p1;
    }
    else
    {
        if (vSize2(direction) == 0)
        {
            std::cout << "warning! too small segment" << std::endl;
            return p0;
        }
        Point ret = p0 + projected_x / vSize(direction) * direction  / vSize(direction);
        return ret ;
    }

}














bool getNextPointWithDistance(Point from, int64_t dist, const PolygonRef poly, int start_idx, int poly_start_idx, GivenDistPoint& result)
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



}//namespace cura
