//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LinePolygonsCrossings.h"

#include <algorithm>

#include "../sliceDataStorage.h"
#include "../utils/SVG.h"

namespace cura {

LinePolygonsCrossings::Crossing::Crossing(const size_t poly_idx, const coord_t x, const size_t point_idx)
: poly_idx(poly_idx)
, x(x)
, point_idx(point_idx)
{
}

bool LinePolygonsCrossings::calcScanlineCrossings(bool fail_on_unavoidable_obstacles)
{
    for(unsigned int poly_idx = 0; poly_idx < boundary.size(); poly_idx++)
    {
        ConstPolygonRef poly = boundary[poly_idx];
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
                const coord_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y); // intersection point between line segment and the scanline
                
                if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                {
                    if (!((p1.Y == transformed_startPoint.Y && p1.Y < p0.Y) || (p0.Y == transformed_startPoint.Y && p0.Y < p1.Y)))
                    { // perform edge case only for line segments on and below the scanline, not for line segments on and above.
                        // \/ will be no crossings and /\ two, but most importantly | will be one crossing.
                        crossings.emplace_back(poly_idx, x, point_idx);
                    }
                }
            }
            p0 = p1;
        }

        if (fail_on_unavoidable_obstacles && crossings.size() % 2 == 1)
        { // if start area and end area are not the same
            return false;
        }
    }
    // order crossings by increasing x
    std::sort(crossings.begin(), crossings.end(), [](const Crossing& a, const Crossing& b) -> bool { return a.x < b.x; });
    return true;
}


bool LinePolygonsCrossings::lineSegmentCollidesWithBoundary()
{
    Point diff = endPoint - startPoint;

    transformation_matrix = PointMatrix(diff);
    transformed_startPoint = transformation_matrix.apply(startPoint);
    transformed_endPoint = transformation_matrix.apply(endPoint);

    for(ConstPolygonRef poly : boundary)
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


bool LinePolygonsCrossings::generateCombingPath(CombPath& combPath, int64_t max_comb_distance_ignored, bool fail_on_unavoidable_obstacles)
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
    generateBasicCombingPath(basicPath);
    optimizePath(basicPath, combPath);
//    combPath = basicPath; // uncomment to disable comb path optimization
    return true;
}


void LinePolygonsCrossings::generateBasicCombingPath(CombPath& combPath)
{
    // crossings are ordered by increasing x
    for (unsigned i = 0; i < crossings.size(); ++i)
    {
        // find the next crossing that belongs to the same polygon
        for (unsigned j = i + 1; j < crossings.size(); ++j)
        {
            if (crossings[i].poly_idx == crossings[j].poly_idx)
            {
                // comb between the two crossings
                generateBasicCombingPath(crossings[i], crossings[j], combPath);
                // update outer loop variable to skip any crossings on other polygons
                i = j;
                break;
            }
        }
    }
    combPath.push_back(endPoint);
}

void LinePolygonsCrossings::generateBasicCombingPath(const Crossing& min, const Crossing& max, CombPath& combPath)
{
    // minimise the path length by measuring the length of both paths around the polygon so we can determine the shorter path

    ConstPolygonRef poly = boundary[min.poly_idx];
    combPath.push_back(transformation_matrix.unapply(Point(min.x - std::abs(dist_to_move_boundary_point_outside), transformed_startPoint.Y)));

    // follow the path in the same direction as the winding order of the boundary polygon
    std::vector<Point> fwd_points;
    Point prev = combPath.back();
    coord_t fwd_len = 0;
    for (unsigned int point_idx = min.point_idx
        ; point_idx != max.point_idx
        ; point_idx = (point_idx < poly.size() - 1) ? (point_idx + 1) : (0))
    {
        const Point p = PolygonUtils::getBoundaryPointWithOffset(poly, point_idx, dist_to_move_boundary_point_outside);
        fwd_points.push_back(p);
        fwd_len += vSize(p - prev);
        prev = p;
    }

    const Point last = transformation_matrix.unapply(Point(max.x + std::abs(dist_to_move_boundary_point_outside), transformed_startPoint.Y));

    if (fwd_points.size() > 0)
    {
        fwd_len += vSize(last - fwd_points.back());
    }

    // follow the path in the opposite direction of the winding order of the boundary polygon
    std::vector<Point> rev_points;
    prev = combPath.back();
    coord_t rev_len = 0;
    unsigned int min_idx = (min.point_idx == 0)? poly.size() - 1 : min.point_idx - 1;
    unsigned int max_idx = (max.point_idx == 0)? poly.size() - 1 : max.point_idx - 1;
    for (unsigned int point_idx = min_idx
        ; point_idx != max_idx
        ; point_idx = (point_idx > 0) ? (point_idx - 1) : (poly.size() - 1))
    {
        const Point p = PolygonUtils::getBoundaryPointWithOffset(poly, point_idx, dist_to_move_boundary_point_outside);
        rev_points.push_back(p);
        rev_len += vSize(p - prev);
        prev = p;
        if (rev_len > fwd_len)
        {
            // this path is already longer than the forward path so there's no point in carrying on
            break;
        }
    }

    if (rev_points.size() > 0)
    {
        rev_len += vSize(last - rev_points.back());
    }

    // use the points from the shortest path
    for (auto& p : (fwd_len < rev_len) ? fwd_points : rev_points)
    {
        combPath.push_back(p);
    }
    combPath.push_back(last);
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
        if (PolygonUtils::polygonCollidesWithLineSegment(current_point, comb_path[point_idx], loc_to_line_grid))
        {
            if (PolygonUtils::polygonCollidesWithLineSegment(current_point, comb_path[point_idx - 1], loc_to_line_grid))
            {
                comb_path.cross_boundary = true;
            }
            // before we add this point, can we discard some of the previous points and still avoid clashing with the combing boundary?
            const int max_short_circuit_len = 1 << 3; // test distances of 8, 4, 2, 1
            for (unsigned n = std::min(max_short_circuit_len, (int)optimized_comb_path.size()); n > 0; n >>= 1)
            {
                if (optimized_comb_path.size() > n && !PolygonUtils::polygonCollidesWithLineSegment(optimized_comb_path[optimized_comb_path.size() - n - 1], comb_path[point_idx - 1], loc_to_line_grid))
                {
                    // we can remove n points from the path without it clashing with the combing boundary
                    for (unsigned i = 0; i < n; ++i)
                    {
                        optimized_comb_path.pop_back();
                    }
                    break;
                }
            }
            optimized_comb_path.push_back(comb_path[point_idx - 1]);

            if (point_idx == 1)
            {
                // we have just inserted the first point in the path that touches the boundary
                // and as the first line in the comb is pointing from the start point towards the
                // end point, we can maybe save some travel by replacing this first point with another
                // that also lies on the second path of the comb, like this:
                //
                //         1
                //        /|
                //       / |
                //      /  |           ===>  ----1
                //     S   |                 S   |
                //         |                     |
                //         2----3 ...            2----3 ...
                //

                Point p = optimized_comb_path.back();
                for (float frac : { 0.9, 0.9, 0.7, 0.5 })
                {
                    // slide p towards the second point in the comb path
                    p = comb_path[1] + (p - comb_path[1]) * frac;
                    if (!PolygonUtils::polygonCollidesWithLineSegment(startPoint, p, loc_to_line_grid))
                    {
                        // using the new corner doesn't cause a conflict
                        optimized_comb_path.back() = p;
                    }
                    else
                    {
                        // quit loop and keep what we know to be good
                        break;
                    }
                }
            }
        }
        else 
        {
            // : dont add the newest point
            
            // TODO: add the below extra optimization? (+/- 7% extra computation time, +/- 2% faster print for Dual_extrusion_support_generation.stl)
            while (optimized_comb_path.size() > 1)
            {
                if (PolygonUtils::polygonCollidesWithLineSegment(optimized_comb_path[optimized_comb_path.size() - 2], comb_path[point_idx], loc_to_line_grid))
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

    if (optimized_comb_path.size() > 1)
    {
        const unsigned n = optimized_comb_path.size();
        // the penultimate corner may be deleted if the resulting path doesn't conflict with the boundary
        if (!PolygonUtils::polygonCollidesWithLineSegment(optimized_comb_path[n - 2], comb_path.back(), loc_to_line_grid))
        {
            optimized_comb_path.pop_back();
        }
        else {
            // that wasn't possible so try and move the penultimate corner without conficting with the boundary
            // in exactly the same way as we did at the start of the path
            for (float frac : { 0.9, 0.9, 0.7, 0.5 })
            {
                // make a new point between the penultimate corner and the corner before that
                Point p = optimized_comb_path[n - 2] + (optimized_comb_path[n - 1] - optimized_comb_path[n - 2]) * frac;
                if (!PolygonUtils::polygonCollidesWithLineSegment(p, comb_path.back(), loc_to_line_grid))
                {
                    // using the new corner doesn't cause a conflict
                    optimized_comb_path[n - 1] = p;
                }
                else
                {
                    // quit loop and keep what we know to be good
                    break;
                }
            }
        }
    }

    optimized_comb_path.push_back(comb_path.back());
    return true;
}

}//namespace cura
