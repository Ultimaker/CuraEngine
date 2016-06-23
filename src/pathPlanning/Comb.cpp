/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "Comb.h"

#include <algorithm>

#include "../utils/polygonUtils.h"
#include "../sliceDataStorage.h"
#include "../utils/SVG.h"

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
        outside_loc_to_line = PolygonUtils::createLocToLineGrid(outside, offset_from_inside_to_outside * 3 / 2);
    }
    return *outside_loc_to_line;
}

  
Comb::Comb(SliceDataStorage& storage, int layer_nr, Polygons& comb_boundary_inside, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance)
: storage(storage)
, layer_nr(layer_nr)
, offset_from_outlines(comb_boundary_offset) // between second wall and infill / other walls
, max_moveInside_distance2(offset_from_outlines * 2 * offset_from_outlines * 2)
, offset_from_outlines_outside(travel_avoid_distance)
, offset_from_inside_to_outside(offset_from_outlines + offset_from_outlines_outside)
, max_crossing_dist2(offset_from_inside_to_outside * offset_from_inside_to_outside * 3) // so max_crossing_dist = offset_from_inside_to_outside * sqrt(3), which is a bit more than sqrt(2) which is necesary for 90* corners
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
    const bool startInside = moveInside(_startInside, startPoint, start_inside_poly);

    unsigned int end_inside_poly = NO_INDEX;
    const bool endInside = moveInside(_endInside, endPoint, end_inside_poly);

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

        Crossing start_crossing(startPoint, startInside, start_part_idx, start_part_boundary_poly_idx, boundary_inside);
        Crossing end_crossing(endPoint, endInside, end_part_idx, end_part_boundary_poly_idx, boundary_inside);

        { // find crossing over the in-between area between inside and outside
            start_crossing.findCrossingInOrMid(partsView_inside, endPoint);
            end_crossing.findCrossingInOrMid(partsView_inside, start_crossing.in_or_mid);
        }

        bool avoid_other_parts_now = avoid_other_parts;
        if (avoid_other_parts_now && vSize2(start_crossing.in_or_mid - end_crossing.in_or_mid) < offset_from_inside_to_outside * offset_from_inside_to_outside * 4)
        { // parts are next to eachother, i.e. the direct crossing will always be smaller than two crossings via outside
            avoid_other_parts_now = false;
        }

        if (avoid_other_parts_now)
        { // compute the crossing points when moving through air
            Polygons& outside = getBoundaryOutside(); // comb through all air, since generally the outside consists of a single part

            bool success = start_crossing.findOutside(outside, end_crossing.in_or_mid, over_inavoidable_obstacles_makes_combing_fail, *this);
            if (!success)
            {
                return false;
            }

            success = end_crossing.findOutside(outside, start_crossing.out, over_inavoidable_obstacles_makes_combing_fail, *this);
            if (!success)
            {
                return false;
            }
        }

        // generate the actual comb paths
        if (startInside)
        {
            // start to boundary
            assert(start_crossing.dest_part.size() > 0 && "The part we start inside when combing should have been computed already!");
            combPaths.emplace_back();
            bool combing_succeeded = LinePolygonsCrossings::comb(start_crossing.dest_part, startPoint, start_crossing.in_or_mid, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
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
            if ( vSize(start_crossing.in_or_mid - end_crossing.in_or_mid) < vSize(start_crossing.in_or_mid - start_crossing.out) + vSize(end_crossing.in_or_mid - end_crossing.out) )
            { // via outside is moving more over the in-between zone
                combPaths.back().push_back(start_crossing.in_or_mid);
                combPaths.back().push_back(end_crossing.in_or_mid);
            }
            else
            {
                bool combing_succeeded = LinePolygonsCrossings::comb(getBoundaryOutside(), start_crossing.out, end_crossing.out, combPaths.back(), offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
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
            combPaths.back().push_back(start_crossing.in_or_mid);
            combPaths.back().push_back(end_crossing.in_or_mid);
        }
        
        if (endInside)
        {
            // boundary to end
            assert(end_crossing.dest_part.size() > 0 && "The part we end up inside when combing should have been computed already!");
            combPaths.emplace_back();
            bool combing_succeeded = LinePolygonsCrossings::comb(end_crossing.dest_part, end_crossing.in_or_mid, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, over_inavoidable_obstacles_makes_combing_fail);
            assert(combing_succeeded && "Couldn't comb between end point and computed crossing to the end part!");
            if (!combing_succeeded)
            {
                return false;
            }
        }
        
        return true;
    }
}

Comb::Crossing::Crossing(const Point& dest_point, const bool dest_is_inside, const unsigned int dest_part_idx, const unsigned int dest_part_boundary_crossing_poly_idx, const Polygons& boundary_inside)
: dest_is_inside(dest_is_inside)
, dest_crossing_poly(boundary_inside[dest_part_boundary_crossing_poly_idx]) // initialize with most obvious poly, cause mostly a combing move will move outside the part, rather than inside a hole in the part
, dest_point(dest_point)
, dest_part_idx(dest_part_idx)
{

}

bool Comb::moveInside(bool is_inside, Point& dest_point, unsigned int& inside_poly)
{
    if (is_inside)
    {
        ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(boundary_inside, dest_point, offset_extra_start_end, max_moveInside_distance2);
        if (cpp.point_idx == NO_INDEX)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return is_inside;
}

void Comb::Crossing::findCrossingInOrMid(const PartsView& partsView_inside, const Point close_to)
{
    if (dest_is_inside)
    { // in-case
        // find the point on the start inside-polygon closest to the endpoint, but also kind of close to the start point
        Point _dest_point(dest_point); // copy to local variable for lambda capture
        std::function<int(Point)> close_towards_start_penalty_function([_dest_point](Point candidate){ return vSize2((candidate - _dest_point) / 10); });
        dest_part = partsView_inside.assemblePart(dest_part_idx);
        ClosestPolygonPoint crossing_1_in_cp = PolygonUtils::findClosest(close_to, dest_part, close_towards_start_penalty_function);
        dest_crossing_poly = crossing_1_in_cp.poly;
        int offset_to_get_off_boundary = offset_dist_to_get_from_on_the_polygon_to_outside;
        in_or_mid = PolygonUtils::moveInside(crossing_1_in_cp, offset_to_get_off_boundary);
        while (!dest_part.inside(in_or_mid) && offset_to_get_off_boundary != 0)
        { // on very small segments, try again with smaller offset
            offset_to_get_off_boundary /= 2;
            in_or_mid = PolygonUtils::moveInside(crossing_1_in_cp, offset_to_get_off_boundary);
        }
    }
    else 
    {
        in_or_mid = dest_point; // mid-case
    }
};

bool Comb::Crossing::findOutside(const Polygons& outside, const Point close_to, const bool over_inavoidable_obstacles_makes_combing_fail, Comb& comber)
{
    out = in_or_mid;
    if (dest_is_inside || outside.inside(in_or_mid, true)) // start in_between
    { // move outside
        Point preferred_crossing_1_out = in_or_mid + normal(close_to - in_or_mid, comber.offset_from_inside_to_outside);
        std::function<int(Point)> close_to_penalty_function([preferred_crossing_1_out](Point candidate){ return vSize2((candidate - preferred_crossing_1_out) / 2); });
        ClosestPolygonPoint* crossing_1_out_cpp = PolygonUtils::findClose(in_or_mid, outside, comber.getOutsideLocToLine(), close_to_penalty_function);
        if (crossing_1_out_cpp)
        {
            out = PolygonUtils::moveOutside(*crossing_1_out_cpp, comber.offset_dist_to_get_from_on_the_polygon_to_outside);
        }
        else 
        {
            PolygonUtils::moveOutside(outside, out, comber.offset_dist_to_get_from_on_the_polygon_to_outside);
        }
    }
    int64_t in_out_dist2_1 = vSize2(out - in_or_mid); 
    if (dest_is_inside && in_out_dist2_1 > comber.max_crossing_dist2) // moveInside moved too far
    { // if move is to far over in_between
        // find crossing closer by
        std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> best = findBestCrossing(outside, dest_crossing_poly, dest_point, close_to, comber);
        if (best)
        {
            in_or_mid = PolygonUtils::moveInside(best->first, comber.offset_dist_to_get_from_on_the_polygon_to_outside);
            out = PolygonUtils::moveOutside(best->second, comber.offset_dist_to_get_from_on_the_polygon_to_outside);
        }
        if (over_inavoidable_obstacles_makes_combing_fail && vSize2(out - in_or_mid) > comber.max_crossing_dist2) // moveInside moved still too far
        {
            return false;
        }
    }
    return true;
}


std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> Comb::Crossing::findBestCrossing(const Polygons& outside, const PolygonRef from, const Point estimated_start, const Point estimated_end, Comb& comber)
{
    ClosestPolygonPoint* best_in = nullptr;
    ClosestPolygonPoint* best_out = nullptr;
    int64_t best_detour_score = std::numeric_limits<int64_t>::max();
    int64_t best_crossing_dist2;
    std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> crossing_out_candidates = PolygonUtils::findClose(from, outside, comber.getOutsideLocToLine());
    bool seen_close_enough_connection = false;
    for (std::pair<ClosestPolygonPoint, ClosestPolygonPoint>& crossing_candidate : crossing_out_candidates)
    {
        int64_t crossing_dist2 = vSize2(crossing_candidate.first.location - crossing_candidate.second.location);
        if (crossing_dist2 > comber.max_crossing_dist2 * 2)
        { // preliminary filtering
            continue;
        }
        
        int64_t dist_to_start = vSize(crossing_candidate.second.location - estimated_start); // use outside location, so that the crossing direction is taken into account
        int64_t dist_to_end = vSize(crossing_candidate.second.location - estimated_end);
        int64_t detour_dist = dist_to_start + dist_to_end;
        int64_t detour_score = crossing_dist2 + detour_dist * detour_dist / 1000; // prefer a closest connection over a detour
        if ((!seen_close_enough_connection && detour_score < best_detour_score) // keep the best as long as we havent seen one close enough (so that we may walk along the polygon to find a closer connection from it in the code below)
            || (!seen_close_enough_connection && crossing_dist2 <= comber.max_crossing_dist2) // make the one which is close enough the best as soon as we see one close enough
            || (seen_close_enough_connection && crossing_dist2 <= comber.max_crossing_dist2 && detour_score < best_detour_score)) // update to keep the best crossing which is close enough already
        {
            if (!seen_close_enough_connection && crossing_dist2 <= comber.max_crossing_dist2)
            {
                seen_close_enough_connection = true;
            }
            best_in = &crossing_candidate.first;
            best_out = &crossing_candidate.second;
            best_detour_score = detour_score;
            best_crossing_dist2 = crossing_dist2;
        }
    }
    if (best_detour_score == std::numeric_limits<int64_t>::max())
    {
        return std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>>();
    }
    if (best_crossing_dist2 > comber.max_crossing_dist2)
    { // find closer point on line segments, rather than moving between vertices of the polygons only
        PolygonUtils::walkToNearestSmallestConnection(*best_in, *best_out);
        best_crossing_dist2 = vSize2(best_in->location - best_out->location);
        if (best_crossing_dist2 > comber.max_crossing_dist2)
        {
            return std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>>();
        }
    }
    return std::make_shared<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>>(*best_in, *best_out);
}

}//namespace cura
