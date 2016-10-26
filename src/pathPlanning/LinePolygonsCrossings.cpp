/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "LinePolygonsCrossings.h"

#include <algorithm>

#include "../utils/polygonUtils.h"
#include "../sliceDataStorage.h"
#include "../utils/SVG.h"

namespace cura {


bool LinePolygonsCrossings::calcScanlineCrossings(bool fail_on_unavoidable_obstacles)
{
    
    min_crossing_idx = NO_INDEX;
    max_crossing_idx = NO_INDEX;

    for(unsigned int poly_idx = 0; poly_idx < boundary.size(); poly_idx++)
    {
        PolyCrossings minMax(poly_idx); 
        PolygonRef poly = boundary[poly_idx];
        Point p0 = transformation_matrix.apply(poly[poly.size() - 1]);
        for(unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = transformation_matrix.apply(poly[point_idx]);
            if ((p0.Y >= transformed_startPoint.Y && p1.Y <= transformed_startPoint.Y) || (p1.Y >= transformed_startPoint.Y && p0.Y <= transformed_startPoint.Y))
            { // if line segment crosses the line through the transformed start and end point (aka scanline)
                if (p1.Y == p0.Y) //Line segment is parallel with the scanline. That means that both endpoints lie on the scanline, so they will have intersected with the adjacent line.
                {
                    p0 = p1;
                    continue;
                }
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y); // intersection point between line segment and the scanline
                
                if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                {
                    if (!((p1.Y == transformed_startPoint.Y && p1.Y < p0.Y) || (p0.Y == transformed_startPoint.Y && p0.Y < p1.Y)))
                    { // perform edge case only for line segments on and below the scanline, not for line segments on and above.
                        // \/ will be no crossings and /\ two, but most importantly | will be one crossing.
                        minMax.n_crossings++;
                    }
                    if(x < minMax.min.x) //For the leftmost intersection, move x left to stay outside of the border.
                                         //Note: The actual distance from the intersection to the border is almost always less than dist_to_move_boundary_point_outside, since it only moves along the direction of the scanline.
                    {
                        minMax.min.x = x;
                        minMax.min.point_idx = point_idx;
                    }
                    if(x > minMax.max.x) //For the rightmost intersection, move x right to stay outside of the border.
                    {
                        minMax.max.x = x;
                        minMax.max.point_idx = point_idx;
                    }
                }
            }
            p0 = p1;
        }

        if (fail_on_unavoidable_obstacles && minMax.n_crossings % 2 == 1)
        { // if start area and end area are not the same
            return false;
        }
        else if (minMax.min.point_idx != NO_INDEX) // then always also max.point_idx != NO_INDEX
        { // if this polygon crossed the scanline
            if (min_crossing_idx == NO_INDEX || minMax.min.x < crossings[min_crossing_idx].min.x) { min_crossing_idx = crossings.size(); }
            if (max_crossing_idx == NO_INDEX || minMax.max.x > crossings[max_crossing_idx].max.x) { max_crossing_idx = crossings.size(); }
            crossings.push_back(minMax);
        }
    }
    return true;
}


bool LinePolygonsCrossings::lineSegmentCollidesWithBoundary()
{
    Point diff = endPoint - startPoint;

    transformation_matrix = PointMatrix(diff);
    transformed_startPoint = transformation_matrix.apply(startPoint);
    transformed_endPoint = transformation_matrix.apply(endPoint);

    for(PolygonRef poly : boundary)
    {
        Point p0 = transformation_matrix.apply(poly.back());
        for(Point p1_ : poly)
        {
            Point p1 = transformation_matrix.apply(p1_);
            // when the boundary just touches the line don't disambiguate between the boundary moving on to actually cross the line
            // and the boundary bouncing back, resulting in not a real collision - to keep the algorithm simple.
            //
            // disregard overlapping line segments; probably the next or previous line segment is not overlapping, but will give a collision
            // when the boundary line segment fully overlaps with the line segment this edge case is not viewed as a collision
            if (p1.Y != p0.Y && ((p0.Y >= transformed_startPoint.Y && p1.Y <= transformed_startPoint.Y) || (p1.Y >= transformed_startPoint.Y && p0.Y <= transformed_startPoint.Y)))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);

                if (x > transformed_startPoint.X && x < transformed_endPoint.X)
                {
                    return true;
                }
            }
            p0 = p1;
        }
    }
    
    return false;
}


bool LinePolygonsCrossings::getCombingPath(CombPath& combPath, int64_t max_comb_distance_ignored, bool fail_on_unavoidable_obstacles)
{
    if (shorterThen(endPoint - startPoint, max_comb_distance_ignored) || !lineSegmentCollidesWithBoundary())
    {
        //We're not crossing any boundaries. So skip the comb generation.
        combPath.push_back(startPoint); 
        combPath.push_back(endPoint);
        return true;
    }
    
    bool success = calcScanlineCrossings(fail_on_unavoidable_obstacles);
    if (!success)
    {
        return false;
    }

    CombPath basicPath;
    getBasicCombingPath(basicPath);
    optimizePath(basicPath, combPath);
//     combPath = basicPath; // uncomment to disable comb path optimization
    return true;
}


void LinePolygonsCrossings::getBasicCombingPath(CombPath& combPath) 
{
    for (PolyCrossings* crossing = getNextPolygonAlongScanline(transformed_startPoint.X)
        ; crossing != nullptr
        ; crossing = getNextPolygonAlongScanline(crossing->max.x))
    {
        getBasicCombingPath(*crossing, combPath);
    }
    combPath.push_back(endPoint);
}

void LinePolygonsCrossings::getBasicCombingPath(PolyCrossings& polyCrossings, CombPath& combPath) 
{
    PolygonRef poly = boundary[polyCrossings.poly_idx];
    combPath.push_back(transformation_matrix.unapply(Point(polyCrossings.min.x - dist_to_move_boundary_point_outside, transformed_startPoint.Y)));
    if ( ( polyCrossings.max.point_idx - polyCrossings.min.point_idx + poly.size() ) % poly.size() 
        < poly.size() / 2 )
    { // follow the path in the same direction as the winding order of the boundary polygon
        for(unsigned int point_idx = polyCrossings.min.point_idx
            ; point_idx != polyCrossings.max.point_idx
            ; point_idx = (point_idx < poly.size() - 1) ? (point_idx + 1) : (0))
        {
            combPath.push_back(PolygonUtils::getBoundaryPointWithOffset(poly, point_idx, dist_to_move_boundary_point_outside));
        }
    }
    else
    { // follow the path in the opposite direction of the winding order of the boundary polygon
        unsigned int min_idx = (polyCrossings.min.point_idx == 0)? poly.size() - 1: polyCrossings.min.point_idx - 1;
        unsigned int max_idx = (polyCrossings.max.point_idx == 0)? poly.size() - 1: polyCrossings.max.point_idx - 1;

        for(unsigned int point_idx = min_idx; point_idx != max_idx; point_idx = (point_idx > 0) ? (point_idx - 1) : (poly.size() - 1))
        {
            combPath.push_back(PolygonUtils::getBoundaryPointWithOffset(poly, point_idx, dist_to_move_boundary_point_outside));
        }
    }
    combPath.push_back(transformation_matrix.unapply(Point(polyCrossings.max.x + dist_to_move_boundary_point_outside, transformed_startPoint.Y))); 
}



LinePolygonsCrossings::PolyCrossings* LinePolygonsCrossings::getNextPolygonAlongScanline(int64_t x)
{
    PolyCrossings* ret = nullptr;
    for(PolyCrossings& crossing : crossings)
    {
        if (crossing.min.x > x && (ret == nullptr || crossing.min.x < ret->min.x) )
        {
            ret = &crossing;
        }
    }
    return ret;
}

bool LinePolygonsCrossings::optimizePath(CombPath& comb_path, CombPath& optimized_comb_path) 
{
    optimized_comb_path.push_back(startPoint);
    for(unsigned int point_idx = 1; point_idx<comb_path.size(); point_idx++)
    {
        if(comb_path[point_idx] == comb_path[point_idx - 1]) //Two points are the same. Skip the second.
        {
            continue;
        }
        Point& current_point = optimized_comb_path.back();
        if (PolygonUtils::polygonCollidesWithlineSegment(current_point, comb_path[point_idx], loc_to_line_grid))
        {
            if (PolygonUtils::polygonCollidesWithlineSegment(current_point, comb_path[point_idx - 1], loc_to_line_grid))
            {
                comb_path.cross_boundary = true;
            }
            optimized_comb_path.push_back(comb_path[point_idx - 1]);
        }
        else 
        {
            // : dont add the newest point
            
            // TODO: add the below extra optimization? (+/- 7% extra computation time, +/- 2% faster print for Dual_extrusion_support_generation.stl)
            while (optimized_comb_path.size() > 1)
            {
                if (PolygonUtils::polygonCollidesWithlineSegment(optimized_comb_path[optimized_comb_path.size() - 2], comb_path[point_idx], loc_to_line_grid))
                {
                    break;
                }
                else 
                {
                    optimized_comb_path.pop_back();
                }
            }
        }
    }
    optimized_comb_path.push_back(comb_path.back());
    return true;
}

}//namespace cura
