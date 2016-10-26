/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "Comb.h"

#include <algorithm>
#include <functional> // function

#include "../utils/polygonUtils.h"
#include "../sliceDataStorage.h"
#include "../utils/SVG.h"

namespace cura {

SparseLineGrid<PolygonsPointIndex, PolygonsPointIndexSegmentLocator>& Comb::getOutsideLocToLine()
{
    return *outside_loc_to_line;
}

Polygons& Comb::getBoundaryOutside()
{
    return *boundary_outside;
}
  
Comb::Comb(SliceDataStorage& storage, int layer_nr, Polygons& comb_boundary_inside, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance)
: storage(storage)
, layer_nr(layer_nr)
, offset_from_outlines(comb_boundary_offset) // between second wall and infill / other walls
, max_moveInside_distance2(offset_from_outlines * 2 * offset_from_outlines * 2)
, offset_from_outlines_outside(travel_avoid_distance)
, offset_from_inside_to_outside(offset_from_outlines + offset_from_outlines_outside)
, max_crossing_dist2(offset_from_inside_to_outside * offset_from_inside_to_outside * 2) // so max_crossing_dist = offset_from_inside_to_outside * sqrt(2) =approx 1.5 to allow for slightly diagonal crossings and slightly inaccurate crossing computation
, avoid_other_parts(travel_avoid_other_parts)
// , boundary_inside( boundary.offset(-offset_from_outlines) ) // TODO: make inside boundary configurable?
, boundary_inside( comb_boundary_inside )
, inside_loc_to_line(PolygonUtils::createLocToLineGrid(boundary_inside, comb_boundary_offset))
, boundary_outside(
        [&storage, layer_nr, travel_avoid_distance]()
        {
            return storage.getLayerOutlines(layer_nr, false).offset(travel_avoid_distance);
        }
    )
, outside_loc_to_line(
        [](Comb* comber, const int64_t offset_from_inside_to_outside)
        {
            return PolygonUtils::createLocToLineGrid(comber->getBoundaryOutside(), offset_from_inside_to_outside * 3 / 2);
        }
        , this
        , offset_from_inside_to_outside
    )
, partsView_inside( boundary_inside.splitIntoPartsView() ) // !! changes the order of boundary_inside !!
{
}

Comb::~Comb()
{
    if (inside_loc_to_line)
    {
        delete inside_loc_to_line;
    }
}

bool Comb::calc(Point startPoint, Point endPoint, CombPaths& combPaths, bool _startInside, bool _endInside, int64_t max_comb_distance_ignored, bool via_outside_makes_combing_fail, bool fail_on_unavoidable_obstacles)
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
        return LinePolygonsCrossings::comb(part, startPoint, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
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

        bool skip_avoid_other_parts_path = false;
        if (skip_avoid_other_parts_path && vSize2(start_crossing.in_or_mid - end_crossing.in_or_mid) < offset_from_inside_to_outside * offset_from_inside_to_outside * 4)
        { // parts are next to eachother, i.e. the direct crossing will always be smaller than two crossings via outside
            skip_avoid_other_parts_path = true;
        }

        if (avoid_other_parts && !skip_avoid_other_parts_path)
        { // compute the crossing points when moving through air
            // comb through all air, since generally the outside consists of a single part

            bool success = start_crossing.findOutside(*boundary_outside, end_crossing.in_or_mid, fail_on_unavoidable_obstacles, *this);
            if (!success)
            {
                return false;
            }

            success = end_crossing.findOutside(*boundary_outside, start_crossing.out, fail_on_unavoidable_obstacles, *this);
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
            bool combing_succeeded = LinePolygonsCrossings::comb(start_crossing.dest_part, startPoint, start_crossing.in_or_mid, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
            if (!combing_succeeded)
            { // Couldn't comb between start point and computed crossing from the start part! Happens for very thin parts when the offset_to_get_off_boundary moves points to outside the polygon
                return false;
            }
        }
        
        // throught air from boundary to boundary
        if (avoid_other_parts && !skip_avoid_other_parts_path)
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
                bool combing_succeeded = LinePolygonsCrossings::comb(*boundary_outside, start_crossing.out, end_crossing.out, combPaths.back(), offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
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
            combPaths.back().cross_boundary = true; // note: we don't actually know whether this is cross boundary, but it might very well be
            combPaths.back().push_back(start_crossing.in_or_mid);
            combPaths.back().push_back(end_crossing.in_or_mid);
        }
        if (skip_avoid_other_parts_path)
        {
            if (startInside == endInside && start_part_idx == end_part_idx)
            {
                if (startInside)
                { // both start and end are inside
                    combPaths.back().cross_boundary = PolygonUtils::polygonCollidesWithlineSegment(startPoint, endPoint, *inside_loc_to_line);
                }
                else
                { // both start and end are outside
                    combPaths.back().cross_boundary = PolygonUtils::polygonCollidesWithlineSegment(startPoint, endPoint, *outside_loc_to_line);
                }
            }
            else
            {
                combPaths.back().cross_boundary = true;
            }
        }
        
        if (endInside)
        {
            // boundary to end
            assert(end_crossing.dest_part.size() > 0 && "The part we end up inside when combing should have been computed already!");
            combPaths.emplace_back();
            
            bool combing_succeeded = LinePolygonsCrossings::comb(end_crossing.dest_part, end_crossing.in_or_mid, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
            if (!combing_succeeded)
            { // Couldn't comb between end point and computed crossing to the end part! Happens for very thin parts when the offset_to_get_off_boundary moves points to outside the polygon
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
            inside_poly = cpp.poly_idx;
            return true;
        }
    }
    return false;
}

void Comb::Crossing::findCrossingInOrMid(const PartsView& partsView_inside, const Point close_to)
{
    if (dest_is_inside)
    { // in-case
        // find the point on the start inside-polygon closest to the endpoint, but also kind of close to the start point
        Point _dest_point(dest_point); // copy to local variable for lambda capture
        std::function<int(Point)> close_towards_start_penalty_function([_dest_point](Point candidate){ return vSize2((candidate - _dest_point) / 10); });
        dest_part = partsView_inside.assemblePart(dest_part_idx);
        Point result(close_to);
        int64_t max_dist2 = std::numeric_limits<int64_t>::max();
        ClosestPolygonPoint crossing_1_in_cp = PolygonUtils::ensureInsideOrOutside(dest_part, result, offset_dist_to_get_from_on_the_polygon_to_outside, max_dist2, close_towards_start_penalty_function);
        if (crossing_1_in_cp.point_idx != NO_INDEX)
        {
            dest_crossing_poly = crossing_1_in_cp.poly;
            in_or_mid = result;
        }
        else
        { // part is too small to be ensuring a point inside with the given distance
            in_or_mid = dest_point; // just use the startPoint or endPoint itself
        }
    }
    else
    {
        in_or_mid = dest_point; // mid-case
    }
};

bool Comb::Crossing::findOutside(const Polygons& outside, const Point close_to, const bool fail_on_unavoidable_obstacles, Comb& comber)
{
    out = in_or_mid;
    if (dest_is_inside || outside.inside(in_or_mid, true)) // start in_between
    { // move outside
        Point preferred_crossing_1_out = in_or_mid + normal(close_to - in_or_mid, comber.offset_from_inside_to_outside);
        std::function<int(Point)> close_to_penalty_function([preferred_crossing_1_out](Point candidate){ return vSize2((candidate - preferred_crossing_1_out) / 2); });
        std::optional<ClosestPolygonPoint> crossing_1_out_cpp = PolygonUtils::findClose(in_or_mid, outside, comber.getOutsideLocToLine(), close_to_penalty_function);
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
    { // if move is too far over in_between
        // find crossing closer by
        std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> best = findBestCrossing(outside, dest_crossing_poly, dest_point, close_to, comber);
        if (best)
        {
            in_or_mid = PolygonUtils::moveInside(best->first, comber.offset_dist_to_get_from_on_the_polygon_to_outside);
            out = PolygonUtils::moveOutside(best->second, comber.offset_dist_to_get_from_on_the_polygon_to_outside);
        }
        if (fail_on_unavoidable_obstacles && vSize2(out - in_or_mid) > comber.max_crossing_dist2) // moveInside moved still too far
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
        // The detour distance is generally large compared to the crossing distance.
        // While the crossing is generally about 1mm across,
        // the distance between an arbitrary point and the boundary may well be a couple of centimetres.
        // So the crossing_dist2 is about 1.000.000 while the detour_dist_2 is in the order of 400.000.000
        // In the end we just want to choose between two points which have the _same_ crossing distance, modulo rounding error.
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
    { // i.e. if best_in == nullptr or if best_out == nullptr
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
