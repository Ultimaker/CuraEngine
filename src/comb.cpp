/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "comb.h"

#include <algorithm>

#include "utils/polygonUtils.h"
#include "sliceDataStorage.h"
#include "utils/SVG.h"

namespace cura {


// boundary_outside is only computed when it's needed!
Polygons& Comb::getBoundaryOutside()
{
    if (!boundary_outside)
    {
        boundary_outside = new Polygons();
        *boundary_outside = storage.getLayerOutlines(layer_nr, false).offset(offset_from_outlines_outside); 
    }
    return *boundary_outside;
}

BucketGrid2D<PolygonsPointIndex>& Comb::getOutsideLocToLine()
{
    Polygons& outside = getBoundaryOutside();
    if (!outside_loc_to_line)
    {
        outside_loc_to_line = PolygonUtils::createLocToLineGrid(outside, offset_from_outlines_outside * 3 / 2);
    }
    return *outside_loc_to_line;
}

  
Comb::Comb(SliceDataStorage& storage, int layer_nr, Polygons& comb_boundary_inside, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance)
: storage(storage)
, layer_nr(layer_nr)
, offset_from_outlines(comb_boundary_offset) // between second wall and infill / other walls
, max_moveInside_distance2(offset_from_outlines * 2 * offset_from_outlines * 2)
, offset_from_outlines_outside(travel_avoid_distance)
, max_crossing_dist2(offset_from_outlines_outside * offset_from_outlines_outside * 3) // so max_crossing_dist = offset_from_outlines_outside * sqrt(3), which is a bit more than sqrt(2) which is necesary for 90* corners
, avoid_other_parts(travel_avoid_other_parts)
// , boundary_inside( boundary.offset(-offset_from_outlines) ) // TODO: make inside boundary configurable?
, boundary_inside( comb_boundary_inside )
, boundary_outside(nullptr)
, outside_loc_to_line(nullptr)
, partsView_inside( boundary_inside.splitIntoPartsView() ) // !! changes the order of boundary_inside !!
{
}

Comb::~Comb()
{
    if (boundary_outside)
    {
        delete boundary_outside;
    }
    if (outside_loc_to_line)
    {
        delete outside_loc_to_line;
    }
}

bool Comb::calc(Point startPoint, Point endPoint, CombPaths& combPaths, bool _startInside, bool _endInside, int64_t max_comb_distance_ignored, bool via_outside_makes_combing_fail, bool over_inavoidable_obstacles_makes_combing_fail)
{
    if (shorterThen(endPoint - startPoint, max_comb_distance_ignored))
    {
        return true;
    }

    //Move start and end point inside the comb boundary
    unsigned int start_inside_poly = NO_INDEX;
    if (_startInside) 
    {
        start_inside_poly = PolygonUtils::moveInside(boundary_inside, startPoint, offset_extra_start_end, max_moveInside_distance2);
        if (!boundary_inside.inside(start_inside_poly) || start_inside_poly == NO_INDEX)
        {
            if (start_inside_poly != NO_INDEX)
            { // if not yet inside because of overshoot, try again
                start_inside_poly = PolygonUtils::moveInside(boundary_inside, startPoint, offset_extra_start_end, max_moveInside_distance2);
            }
            if (start_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
            {
                _startInside = false;
            }
        }
    }
    const bool startInside = _startInside;

    unsigned int end_inside_poly = NO_INDEX;
    if (_endInside)
    {
        end_inside_poly = PolygonUtils::moveInside(boundary_inside, endPoint, offset_extra_start_end, max_moveInside_distance2);
        if (!boundary_inside.inside(endPoint) || end_inside_poly == NO_INDEX)
        {
            if (end_inside_poly != NO_INDEX)
            { // if not yet inside because of overshoot, try again
                end_inside_poly = PolygonUtils::moveInside(boundary_inside, endPoint, offset_extra_start_end, max_moveInside_distance2);
            }
            if (end_inside_poly == NO_INDEX)    //If we fail to move the point inside the comb boundary we need to retract.
            {
                _endInside = false;
            }
        }
    }
    const bool endInside = _endInside;

    unsigned int start_part_boundary_poly_idx;
    unsigned int end_part_boundary_poly_idx;
    unsigned int start_part_idx =   (start_inside_poly == NO_INDEX)?    NO_INDEX : partsView_inside.getPartContaining(start_inside_poly, &start_part_boundary_poly_idx);
    unsigned int end_part_idx =     (end_inside_poly == NO_INDEX)?      NO_INDEX : partsView_inside.getPartContaining(end_inside_poly, &end_part_boundary_poly_idx);
    
    if (startInside && endInside && start_part_idx == end_part_idx)
    { // normal combing within part
        PolygonsPart part = partsView_inside.assemblePart(start_part_idx);
        combPaths.emplace_back();
        return LinePolygonsCrossings::comb(part, startPoint, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
    }
    else 
    { // comb inside part to edge (if needed) >> move through air avoiding other parts >> comb inside end part upto the endpoint (if needed) 
        //  INSIDE  |          in_between            |            OUTSIDE     |              in_between         |     INSIDE
        //        ^crossing_1_in     ^crossing_1_mid  ^crossing_1_out        ^crossing_2_out    ^crossing_2_mid   ^crossing_2_in
        //
        // when startPoint is inside crossing_1_in is of interest
        // when it is in between inside and outside it is equal to crossing_1_mid

        if (via_outside_makes_combing_fail)
        {
            return false;
        }

        Point crossing_1_in_or_mid; // the point inside the starting polygon if startPoint is inside or the startPoint itself if it is not inside
        Point crossing_1_out;
        Point crossing_2_in_or_mid; // the point inside the ending polygon if endPoint is inside or the endPoint itself if it is not inside
        Point crossing_2_out;

        PolygonsPart start_part; // will be initialized below when startInside holds
        PolygonsPart end_part; // will be initialized below when endInside holds

        PolygonRef start_part_crossing_poly(boundary_inside[start_part_boundary_poly_idx]); // will refer to the polygon in start_part which should be crossed (mostly the boundary poly, but sometimes a hole)
        PolygonRef end_part_crossing_poly(boundary_inside[end_part_boundary_poly_idx]);     // will refer to the polygon in end_part   which should be crossed (mostly the boundary poly, but sometimes a hole)

        { // find crossing over the in-between area between inside and outside
            if (startInside)
            {
                // find the point on the start inside-polygon closest to the endpoint, but also kind of close to the start point
                std::function<int(Point)> close_towards_start_penalty_function([startPoint](Point candidate){ return vSize2((candidate - startPoint) / 10); });
                start_part = partsView_inside.assemblePart(start_part_idx);
                ClosestPolygonPoint crossing_1_in_cp = PolygonUtils::findClosest(endPoint, start_part, close_towards_start_penalty_function);
                start_part_crossing_poly = crossing_1_in_cp.poly;
                crossing_1_in_or_mid = PolygonUtils::moveInside(crossing_1_in_cp, offset_dist_to_get_from_on_the_polygon_to_outside); // in-case
            }
            else 
            {
                crossing_1_in_or_mid = startPoint; // mid-case
            }

            if (endInside)
            {
                // find the point on the end inside-polygon closest to the start of the crossing between the start inside-polygon, but also kind of close to the end point
                std::function<int(Point)> close_towards_end_crossing_penalty_function([endPoint](Point candidate){ return vSize2((candidate - endPoint) / 10); });
                end_part = partsView_inside.assemblePart(end_part_idx);
                ClosestPolygonPoint crossing_2_in_cp = PolygonUtils::findClosest(crossing_1_in_or_mid, end_part, close_towards_end_crossing_penalty_function);
                end_part_crossing_poly = crossing_2_in_cp.poly;
                crossing_2_in_or_mid = PolygonUtils::moveInside(crossing_2_in_cp, offset_dist_to_get_from_on_the_polygon_to_outside); // in-case
            }
            else 
            {
                crossing_2_in_or_mid = endPoint; // mid-case
            }
        }
        
        bool avoid_other_parts_now = avoid_other_parts;
        if (avoid_other_parts_now && vSize2(crossing_1_in_or_mid - crossing_2_in_or_mid) < offset_from_outlines_outside * offset_from_outlines_outside * 4)
        { // parts are next to eachother, i.e. the direct crossing will always be smaller than two crossings via outside
            avoid_other_parts_now = false;
        }
        
        if (avoid_other_parts_now)
        { // compute the crossing points when moving through air
            Polygons& outside = getBoundaryOutside(); // comb through all air, since generally the outside consists of a single part
            
            
            crossing_1_out = crossing_1_in_or_mid;
            if (startInside || outside.inside(crossing_1_in_or_mid, true)) // start in_between
            { // move outside
                Point preferred_crossing_1_out = crossing_1_in_or_mid + normal(crossing_2_in_or_mid - crossing_1_in_or_mid, offset_from_outlines + offset_from_outlines_outside);
                std::function<int(Point)> close_towards_crossing_2_in_penalty_function([preferred_crossing_1_out](Point candidate){ return vSize2((candidate - preferred_crossing_1_out) / 2); });
                ClosestPolygonPoint* crossing_1_out_cpp = PolygonUtils::findClose(crossing_1_in_or_mid, outside, getOutsideLocToLine(), close_towards_crossing_2_in_penalty_function);
                if (crossing_1_out_cpp)
                {
                    crossing_1_out = PolygonUtils::moveOutside(*crossing_1_out_cpp, offset_dist_to_get_from_on_the_polygon_to_outside);
                }
                else 
                {
                    PolygonUtils::moveOutside(outside, crossing_1_out, offset_dist_to_get_from_on_the_polygon_to_outside);
                }
            }
            int64_t in_out_dist2_1 = vSize2(crossing_1_out - crossing_1_in_or_mid); 
            if (startInside && in_out_dist2_1 > max_crossing_dist2) // moveInside moved too far
            { // if move is to far over in_between
                // find crossing closer by
                std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> best = findBestCrossing(*start_part_crossing_poly, startPoint, endPoint);
                if (best)
                {
                    crossing_1_in_or_mid = PolygonUtils::moveInside(best->first, offset_dist_to_get_from_on_the_polygon_to_outside);
                    crossing_1_out = PolygonUtils::moveOutside(best->second, offset_dist_to_get_from_on_the_polygon_to_outside);
                }
                if (over_inavoidable_obstacles_makes_combing_fail && vSize2(crossing_1_out - crossing_1_in_or_mid) > max_crossing_dist2) // moveInside moved still too far
                {
                    return false;
                }
            }


            crossing_2_out = crossing_2_in_or_mid;
            if (endInside || outside.inside(crossing_2_in_or_mid, true))
            { // move outside
                Point preferred_crossing_2_out = crossing_2_in_or_mid + normal(crossing_1_out - crossing_2_in_or_mid, offset_from_outlines + offset_from_outlines_outside);
                std::function<int(Point)> close_towards_crossing_1_out_penalty_function([preferred_crossing_2_out](Point candidate){ return vSize2((candidate - preferred_crossing_2_out) / 2); });
                ClosestPolygonPoint* crossing_2_out_cpp = PolygonUtils::findClose(crossing_2_in_or_mid, outside, getOutsideLocToLine(), close_towards_crossing_1_out_penalty_function);
                if (crossing_2_out_cpp)
                {
                    crossing_2_out = PolygonUtils::moveOutside(*crossing_2_out_cpp, offset_dist_to_get_from_on_the_polygon_to_outside);
                }
                else 
                {
                    PolygonUtils::moveOutside(outside, crossing_2_out, offset_dist_to_get_from_on_the_polygon_to_outside);
                }
            }
            int64_t in_out_dist2_2 = vSize2(crossing_2_out - crossing_2_in_or_mid); 
            if (endInside && in_out_dist2_2 > max_crossing_dist2) // moveInside moved too far
            { // if move is to far over in_between
                // find crossing closer by
                std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> best = findBestCrossing(*end_part_crossing_poly, endPoint, crossing_1_out);
                if (best)
                {
                    crossing_2_in_or_mid = PolygonUtils::moveInside(best->first, offset_dist_to_get_from_on_the_polygon_to_outside);
                    crossing_2_out = PolygonUtils::moveOutside(best->second, offset_dist_to_get_from_on_the_polygon_to_outside);
                }
                if (over_inavoidable_obstacles_makes_combing_fail && vSize2(crossing_2_out - crossing_2_in_or_mid) > max_crossing_dist2) // moveInside moved still too far
                {
                    return false;
                }
            }
        }

        if (startInside)
        {
            // start to boundary
            assert(start_part.size() > 0 && "The part we start inside when combing should have been computed already!");
            combPaths.emplace_back();
            bool combing_succeeded = LinePolygonsCrossings::comb(start_part, startPoint, crossing_1_in_or_mid, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
            assert(combing_succeeded && "Couldn't comb between start point and computed crossing from the start part!");
            if (!combing_succeeded)
            {
                return false;
            }
        }
        
        // throught air from boundary to boundary
        if (avoid_other_parts_now)
        {
            combPaths.emplace_back();
            combPaths.throughAir = true;
            if ( vSize(crossing_1_in_or_mid - crossing_2_in_or_mid) < vSize(crossing_1_in_or_mid - crossing_1_out) + vSize(crossing_2_in_or_mid - crossing_2_out) )
            { // via outside is moving more over the in-between zone
                combPaths.back().push_back(crossing_1_in_or_mid);
                combPaths.back().push_back(crossing_2_in_or_mid);
            }
            else
            {
                bool combing_succeeded = LinePolygonsCrossings::comb(getBoundaryOutside(), crossing_1_out, crossing_2_out, combPaths.back(), offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
                if (!combing_succeeded)
                {
                    return false;
                }
            }
        }
        else 
        { // directly through air (not avoiding other parts)
            combPaths.emplace_back();
            combPaths.throughAir = true;
            combPaths.back().cross_boundary = true; // TODO: calculate whether we cross a boundary!
            combPaths.back().push_back(crossing_1_in_or_mid);
            combPaths.back().push_back(crossing_2_in_or_mid);
        }
        
        if (endInside)
        {
            // boundary to end
            assert(end_part.size() > 0 && "The part we end up inside when combing should have been computed already!");
            combPaths.emplace_back();
            bool combing_succeeded = LinePolygonsCrossings::comb(end_part, crossing_2_in_or_mid, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
            assert(combing_succeeded && "Couldn't comb between end point and computed crossing to the end part!");
            if (!combing_succeeded)
            {
                return false;
            }
        }
        
        return true;
    }
}

std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> Comb::findBestCrossing(PolygonRef from, Point estimated_start, Point estimated_end)
{
    ClosestPolygonPoint* best_in = nullptr;
    ClosestPolygonPoint* best_out = nullptr;
    int64_t best_detour_dist = std::numeric_limits<int64_t>::max();
    std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> crossing_out_candidates = PolygonUtils::findClose(from, getBoundaryOutside(), getOutsideLocToLine());
    for (std::pair<ClosestPolygonPoint, ClosestPolygonPoint>& crossing_candidate : crossing_out_candidates)
    {
        int64_t crossing_dist2 = vSize2(crossing_candidate.first.location - crossing_candidate.second.location);
        if (crossing_dist2 > max_crossing_dist2)
        {
            continue;
        }
        
        int64_t dist_to_start = vSize(crossing_candidate.second.location - estimated_start); // use outside location, so that the crossing direction is taken into account
        int64_t dist_to_end = vSize(crossing_candidate.second.location - estimated_end);
        int64_t detour_dist = dist_to_start + dist_to_end;
        if (detour_dist < best_detour_dist)
        {
            best_in = &crossing_candidate.first;
            best_out = &crossing_candidate.second;
            best_detour_dist = detour_dist;
        }
    }
    if (best_detour_dist == std::numeric_limits<int64_t>::max())
    {
        return std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>>();
    }
    return std::make_shared<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>>(*best_in, *best_out);
}


bool LinePolygonsCrossings::calcScanlineCrossings(bool over_inavoidable_obstacles_makes_combing_fail)
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
            if((p0.Y >= transformed_startPoint.Y && p1.Y <= transformed_startPoint.Y) || (p1.Y >= transformed_startPoint.Y && p0.Y <= transformed_startPoint.Y))
            { // if line segment crosses the line through the transformed start and end point (aka scanline)
                if(p1.Y == p0.Y) //Line segment is parallel with the scanline. That means that both endpoints lie on the scanline, so they will have intersected with the adjacent line.
                {
                    if (p0.X >= transformed_startPoint.X && p0.X <= transformed_endPoint.X && 
                        p1.X >= transformed_startPoint.X && p1.X <= transformed_endPoint.X &&
                        transformation_matrix.apply(poly[(point_idx + 1) % poly.size()]).Y != p1.Y)
                    { // if whole segment lies between start and end (and is the last such line segment in a row)
                        minMax.n_crossings--; // Don't count both the previous and consecutive segment both as a line crossing
                    }
                    p0 = p1;
                    continue;
                }
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y); // intersection point between line segment and the scanline
                
                if (x >= transformed_startPoint.X && x <= transformed_endPoint.X)
                {
                    minMax.n_crossings++;
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

        if (over_inavoidable_obstacles_makes_combing_fail && minMax.n_crossings % 2 == 1)
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
            if ((p0.Y > transformed_startPoint.Y && p1.Y < transformed_startPoint.Y) || (p1.Y > transformed_startPoint.Y && p0.Y < transformed_startPoint.Y))
            {
                int64_t x = p0.X + (p1.X - p0.X) * (transformed_startPoint.Y - p0.Y) / (p1.Y - p0.Y);
                
                if (x > transformed_startPoint.X && x < transformed_endPoint.X)
                    return true;
            }
            p0 = p1;
        }
    }
    
    return false;
}


bool LinePolygonsCrossings::getCombingPath(CombPath& combPath, int64_t max_comb_distance_ignored, bool over_inavoidable_obstacles_makes_combing_fail)
{
    if (shorterThen(endPoint - startPoint, max_comb_distance_ignored) || !lineSegmentCollidesWithBoundary())
    {
        //We're not crossing any boundaries. So skip the comb generation.
        combPath.push_back(startPoint); 
        combPath.push_back(endPoint);
        return true;
    }
    
    bool success = calcScanlineCrossings(over_inavoidable_obstacles_makes_combing_fail);
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
        if (PolygonUtils::polygonCollidesWithlineSegment(boundary, current_point, comb_path[point_idx]))
        {
            if (PolygonUtils::polygonCollidesWithlineSegment(boundary, current_point, comb_path[point_idx - 1]))
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
                if (PolygonUtils::polygonCollidesWithlineSegment(boundary, optimized_comb_path[optimized_comb_path.size() - 2], comb_path[point_idx]))
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
