//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Comb.h"

#include <algorithm>
#include <functional> // function
#include <unordered_set>

#include "CombPaths.h"
#include "LinePolygonsCrossings.h"
#include "../Application.h"
#include "../ExtruderTrain.h"
#include "../Slice.h"
#include "../utils/linearAlg2D.h"
#include "../utils/PolygonsPointIndex.h"
#include "../sliceDataStorage.h"
#include "../utils/SVG.h"

namespace cura {

LocToLineGrid& Comb::getOutsideLocToLine()
{
    return **outside_loc_to_line;
}

Polygons& Comb::getBoundaryOutside()
{
    return *boundary_outside;
}

Comb::Comb(const SliceDataStorage& storage, const LayerIndex layer_nr, const Polygons& comb_boundary_inside_minimum, const Polygons& comb_boundary_inside_optimal, coord_t comb_boundary_offset, coord_t travel_avoid_distance, coord_t move_inside_distance)
: storage(storage)
, layer_nr(layer_nr)
, offset_from_outlines(comb_boundary_offset) // between second wall and infill / other walls
, max_moveInside_distance2(offset_from_outlines * 2 * offset_from_outlines * 2)
, offset_from_inside_to_outside(offset_from_outlines + travel_avoid_distance)
, max_crossing_dist2(offset_from_inside_to_outside * offset_from_inside_to_outside * 2) // so max_crossing_dist = offset_from_inside_to_outside * sqrt(2) =approx 1.5 to allow for slightly diagonal crossings and slightly inaccurate crossing computation
, boundary_inside_minimum( comb_boundary_inside_minimum ) // copy the boundary, because the partsView_inside will reorder the polygons
, boundary_inside_optimal( comb_boundary_inside_optimal ) // copy the boundary, because the partsView_inside will reorder the polygons
, partsView_inside_minimum( boundary_inside_minimum.splitIntoPartsView() ) // WARNING !! changes the order of boundary_inside !!
, partsView_inside_optimal( boundary_inside_optimal.splitIntoPartsView() ) // WARNING !! changes the order of boundary_inside !!
, inside_loc_to_line_minimum(PolygonUtils::createLocToLineGrid(boundary_inside_minimum, comb_boundary_offset))
, inside_loc_to_line_optimal(PolygonUtils::createLocToLineGrid(boundary_inside_optimal, comb_boundary_offset))
, boundary_outside(
        [&storage, layer_nr, travel_avoid_distance]()
        {
            const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
            bool travel_avoid_supports = false;
            for (const ExtruderTrain& extruder : Application::getInstance().current_slice->scene.extruders)
            {
                travel_avoid_supports |= extruder_is_used[extruder.extruder_nr] && extruder.settings.get<bool>("travel_avoid_other_parts") && extruder.settings.get<bool>("travel_avoid_supports");
            }
            return storage.getLayerOutlines(layer_nr, travel_avoid_supports, travel_avoid_supports).offset(travel_avoid_distance);
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
, move_inside_distance(move_inside_distance)
{
}

bool Comb::calc(const ExtruderTrain& train, Point startPoint, Point endPoint, CombPaths& combPaths, bool _startInside, bool _endInside, coord_t max_comb_distance_ignored, bool &unretract_before_last_travel_move)
{
    if (shorterThen(endPoint - startPoint, max_comb_distance_ignored))
    {
        return true;
    }
    const Point travel_end_point_before_combing = endPoint;
    //Move start and end point inside the optimal comb boundary
    unsigned int start_inside_poly = NO_INDEX;
    const bool startInside = moveInside(boundary_inside_optimal, _startInside, inside_loc_to_line_optimal.get(), startPoint, start_inside_poly);

    unsigned int end_inside_poly = NO_INDEX;
    const bool endInside = moveInside(boundary_inside_optimal, _endInside, inside_loc_to_line_optimal.get(), endPoint, end_inside_poly);

    unsigned int start_part_boundary_poly_idx = NO_INDEX;		// Added initial value to stop MSVC throwing an exception in debug mode
    unsigned int end_part_boundary_poly_idx = NO_INDEX;
    unsigned int start_part_idx =   (start_inside_poly == NO_INDEX)?    NO_INDEX : partsView_inside_optimal.getPartContaining(start_inside_poly, &start_part_boundary_poly_idx);
    unsigned int end_part_idx =     (end_inside_poly == NO_INDEX)?      NO_INDEX : partsView_inside_optimal.getPartContaining(end_inside_poly, &end_part_boundary_poly_idx);

    const bool perform_z_hops = train.settings.get<bool>("retraction_hop_enabled");
    const bool perform_z_hops_only_when_collides = train.settings.get<bool>("retraction_hop_only_when_collides");
    const bool fail_on_unavoidable_obstacles = perform_z_hops && perform_z_hops_only_when_collides;

    // normal combing within part using optimal comb boundary
    if (startInside && endInside && start_part_idx == end_part_idx)
    {
        PolygonsPart part = partsView_inside_optimal.assemblePart(start_part_idx);
        combPaths.emplace_back();
        const bool combing_succeeded = LinePolygonsCrossings::comb(part, *inside_loc_to_line_optimal, startPoint, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when travelling to that outer wall
        unretract_before_last_travel_move = combing_succeeded && endPoint != travel_end_point_before_combing;
        return combing_succeeded;
    }

    //Move start and end point inside the minimum comb boundary
    unsigned int start_inside_poly_min = NO_INDEX;
    const bool startInsideMin = moveInside(boundary_inside_minimum, _startInside, inside_loc_to_line_minimum.get(), startPoint, start_inside_poly_min);

    unsigned int end_inside_poly_min = NO_INDEX;
    const bool endInsideMin = moveInside(boundary_inside_minimum, _endInside, inside_loc_to_line_minimum.get(), endPoint, end_inside_poly_min);

    unsigned int start_part_boundary_poly_idx_min;
    unsigned int end_part_boundary_poly_idx_min;
    unsigned int start_part_idx_min =   (start_inside_poly_min == NO_INDEX)?    NO_INDEX : partsView_inside_minimum.getPartContaining(start_inside_poly_min, &start_part_boundary_poly_idx_min);
    unsigned int end_part_idx_min =     (end_inside_poly_min == NO_INDEX)?      NO_INDEX : partsView_inside_minimum.getPartContaining(end_inside_poly_min, &end_part_boundary_poly_idx_min);

    CombPath result_path;
    bool comb_result;

    // normal combing within part using minimum comb boundary
    if (startInsideMin && endInsideMin && start_part_idx_min == end_part_idx_min)
    {
        PolygonsPart part = partsView_inside_minimum.assemblePart(start_part_idx_min);
        combPaths.emplace_back();

        comb_result = LinePolygonsCrossings::comb(part, *inside_loc_to_line_minimum, startPoint, endPoint, result_path, -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
        Comb::moveCombPathInside(boundary_inside_minimum, boundary_inside_optimal, result_path, combPaths.back());  // add altered result_path to combPaths.back()
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when travelling to that outer wall
        unretract_before_last_travel_move = comb_result && endPoint != travel_end_point_before_combing;
        return comb_result;
    }

    // comb inside part to edge (if needed) >> move through air avoiding other parts >> comb inside end part upto the endpoint (if needed)
    //  INSIDE  |          in_between            |            OUTSIDE     |              in_between         |     INSIDE
    //        ^crossing_1_in     ^crossing_1_mid  ^crossing_1_out        ^crossing_2_out    ^crossing_2_mid   ^crossing_2_in
    //
    // when startPoint is inside crossing_1_in is of interest
    // when it is in between inside and outside it is equal to crossing_1_mid

    if (perform_z_hops && !perform_z_hops_only_when_collides) //Combing via outside makes combing fail.
    {
        return false;
    }

    //Find the crossings using the minimum comb boundary, since it's guaranteed to be as close as we can get to the destination.
    //Getting as close as possible prevents exiting the polygon in the wrong direction (e.g. into a hole instead of to the outside).
    Crossing start_crossing(startPoint, startInsideMin, start_part_idx_min, start_part_boundary_poly_idx_min, boundary_inside_minimum, *inside_loc_to_line_minimum);
    Crossing end_crossing(endPoint, endInsideMin, end_part_idx_min, end_part_boundary_poly_idx_min, boundary_inside_minimum, *inside_loc_to_line_minimum);

    { // find crossing over the in-between area between inside and outside
        start_crossing.findCrossingInOrMid(partsView_inside_minimum, endPoint);
        end_crossing.findCrossingInOrMid(partsView_inside_minimum, start_crossing.in_or_mid);
    }

    bool skip_avoid_other_parts_path = false;
    if (vSize2(start_crossing.in_or_mid - end_crossing.in_or_mid) < offset_from_inside_to_outside * offset_from_inside_to_outside * 4)
    { // parts are next to eachother, i.e. the direct crossing will always be smaller than two crossings via outside
        skip_avoid_other_parts_path = true;
    }

    const std::vector<bool> extruder_is_used = storage.getExtrudersUsed(layer_nr);
    bool travel_avoid_other_parts = false;
    for (const ExtruderTrain& train : Application::getInstance().current_slice->scene.extruders)
    {
        travel_avoid_other_parts |= extruder_is_used[train.extruder_nr] && train.settings.get<bool>("travel_avoid_other_parts");
    }

    if (travel_avoid_other_parts && !skip_avoid_other_parts_path)
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
    if (startInsideMin)
    {
        // start to boundary
        assert(start_crossing.dest_part.size() > 0 && "The part we start inside when combing should have been computed already!");
        combPaths.emplace_back();
        //If we're inside the optimal bound, first try the optimal combing path. If it fails, use the minimum path instead.
        constexpr bool fail_for_optimum_bound = true;
        bool combing_succeeded = startInside && LinePolygonsCrossings::comb(boundary_inside_optimal, *inside_loc_to_line_optimal, startPoint, start_crossing.in_or_mid, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_for_optimum_bound);
        if(!combing_succeeded)
        {
            combing_succeeded = LinePolygonsCrossings::comb(start_crossing.dest_part, *inside_loc_to_line_minimum, startPoint, start_crossing.in_or_mid, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
        }
        if (!combing_succeeded)
        { // Couldn't comb between start point and computed crossing from the start part! Happens for very thin parts when the offset_to_get_off_boundary moves points to outside the polygon
            return false;
        }
    }

    // through air from boundary to boundary
    if (travel_avoid_other_parts && !skip_avoid_other_parts_path)
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
            bool combing_succeeded = LinePolygonsCrossings::comb(*boundary_outside, getOutsideLocToLine(), start_crossing.out, end_crossing.out, combPaths.back(), offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
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
                combPaths.back().cross_boundary = PolygonUtils::polygonCollidesWithLineSegment(startPoint, endPoint, *inside_loc_to_line_optimal);
            }
            else
            { // both start and end are outside
                combPaths.back().cross_boundary = PolygonUtils::polygonCollidesWithLineSegment(startPoint, endPoint, getOutsideLocToLine());
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
        //If we're inside the optimal bound, first try the optimal combing path. If it fails, use the minimum path instead.
        constexpr bool fail_for_optimum_bound = true;
        bool combing_succeeded = endInside && LinePolygonsCrossings::comb(boundary_inside_optimal, *inside_loc_to_line_optimal, end_crossing.in_or_mid, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_for_optimum_bound);
        if(!combing_succeeded)
        {
            combing_succeeded = LinePolygonsCrossings::comb(end_crossing.dest_part, *inside_loc_to_line_minimum, end_crossing.in_or_mid, endPoint, combPaths.back(), -offset_dist_to_get_from_on_the_polygon_to_outside, max_comb_distance_ignored, fail_on_unavoidable_obstacles);
        }
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when travelling to that outer wall
        unretract_before_last_travel_move = combing_succeeded && endPoint != travel_end_point_before_combing;
        if (!combing_succeeded)
        { // Couldn't comb between end point and computed crossing to the end part! Happens for very thin parts when the offset_to_get_off_boundary moves points to outside the polygon
            return false;
        }
    }

    return true;
}

// Try to move comb_path_input points inside by the amount of `move_inside_distance` and see if the points are still in boundary_inside_optimal, add result in comb_path_output
void Comb::moveCombPathInside(Polygons& boundary_inside, Polygons& boundary_inside_optimal, CombPath& comb_path_input, CombPath& comb_path_output)
{
    const coord_t dist = move_inside_distance;
    const coord_t dist2 = dist * dist;

    if (comb_path_input.size() == 0)
    {
        return;
    }
    comb_path_output.push_back(comb_path_input[0]);
    for(unsigned int point_idx = 1; point_idx<comb_path_input.size()-1; point_idx++)
    {
        Point new_point = Point(comb_path_input[point_idx]);
        PolygonUtils::moveInside(boundary_inside, new_point, dist, dist2);

        if (boundary_inside_optimal.inside(new_point))
        {
            comb_path_output.push_back(new_point);
        }
        else
        {
            comb_path_output.push_back(comb_path_input[point_idx]);
        }
    }
    if (comb_path_input.size() > 1)
    {
        comb_path_output.push_back(comb_path_input[comb_path_input.size() - 1]);
    }

}

Comb::Crossing::Crossing(const Point& dest_point, const bool dest_is_inside, const unsigned int dest_part_idx, const unsigned int dest_part_boundary_crossing_poly_idx, const Polygons& boundary_inside, const LocToLineGrid& inside_loc_to_line)
: dest_is_inside(dest_is_inside)
, boundary_inside(boundary_inside)
, inside_loc_to_line(inside_loc_to_line)
, dest_point(dest_point)
, dest_part_idx(dest_part_idx)
{
    if (dest_is_inside)
    {
        dest_crossing_poly.emplace(boundary_inside[dest_part_boundary_crossing_poly_idx]); // initialize with most obvious poly, cause mostly a combing move will move outside the part, rather than inside a hole in the part
    }
}

bool Comb::moveInside(Polygons& boundary_inside, bool is_inside, LocToLineGrid* inside_loc_to_line, Point& dest_point, unsigned int& inside_poly)
{
    if (is_inside)
    {
        ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(boundary_inside, dest_point, offset_extra_start_end, max_moveInside_distance2, &boundary_inside, inside_loc_to_line);
        if (!cpp.isValid())
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

        ClosestPolygonPoint boundary_crossing_point;
        { // set [result] to a point on the destination part closest to close_to (but also a bit close to _dest_point)
            std::unordered_set<unsigned int> dest_part_poly_indices;
            for (unsigned int poly_idx : partsView_inside[dest_part_idx])
            {
                dest_part_poly_indices.emplace(poly_idx);
            }
            coord_t dist2_score = std::numeric_limits<coord_t>::max();
            std::function<bool (const PolygonsPointIndex&)> line_processor
                = [close_to, _dest_point, &boundary_crossing_point, &dist2_score, &dest_part_poly_indices](const PolygonsPointIndex& boundary_segment)
                {
                    if (dest_part_poly_indices.find(boundary_segment.poly_idx) == dest_part_poly_indices.end())
                    { // we're not looking at a polygon from the dest_part
                        return true; // a.k.a. continue;
                    }
                    Point closest_here = LinearAlg2D::getClosestOnLineSegment(close_to, boundary_segment.p(), boundary_segment.next().p());
                    coord_t dist2_score_here = vSize2(close_to - closest_here) + vSize2(_dest_point - closest_here) / 10;
                    if (dist2_score_here < dist2_score)
                    {
                        dist2_score = dist2_score_here;
                        boundary_crossing_point = ClosestPolygonPoint(closest_here, boundary_segment.point_idx, boundary_segment.getPolygon(), boundary_segment.poly_idx);
                    }
                    return true;
                };
            inside_loc_to_line.processLine(std::make_pair(dest_point, close_to), line_processor);
        }

        Point result(boundary_crossing_point.p()); // the inside point of the crossing
        if (!boundary_crossing_point.isValid())
        { // no point has been found in the sparse grid
            result = dest_point;
        }

        ClosestPolygonPoint crossing_1_in_cp = PolygonUtils::ensureInsideOrOutside(dest_part, result, boundary_crossing_point, offset_dist_to_get_from_on_the_polygon_to_outside, &boundary_inside, &inside_loc_to_line, close_towards_start_penalty_function);
        if (crossing_1_in_cp.isValid())
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
    { // mid-case
        in_or_mid = dest_point;
    }
}

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
        assert(dest_crossing_poly && "destination crossing poly should have been instantiated!");
        std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> best = findBestCrossing(outside, **dest_crossing_poly, dest_point, close_to, comber);
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


std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> Comb::Crossing::findBestCrossing(const Polygons& outside, ConstPolygonRef from, const Point estimated_start, const Point estimated_end, Comb& comber)
{
    ClosestPolygonPoint* best_in = nullptr;
    ClosestPolygonPoint* best_out = nullptr;
    coord_t best_detour_score = std::numeric_limits<coord_t>::max();
    coord_t best_crossing_dist2;
    std::vector<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> crossing_out_candidates = PolygonUtils::findClose(from, outside, comber.getOutsideLocToLine());
    bool seen_close_enough_connection = false;
    for (std::pair<ClosestPolygonPoint, ClosestPolygonPoint>& crossing_candidate : crossing_out_candidates)
    {
        const coord_t crossing_dist2 = vSize2(crossing_candidate.first.location - crossing_candidate.second.location);
        if (crossing_dist2 > comber.max_crossing_dist2 * 2)
        { // preliminary filtering
            continue;
        }

        const coord_t dist_to_start = vSize(crossing_candidate.second.location - estimated_start); // use outside location, so that the crossing direction is taken into account
        const coord_t dist_to_end = vSize(crossing_candidate.second.location - estimated_end);
        const coord_t detour_dist = dist_to_start + dist_to_end;
        const coord_t detour_score = crossing_dist2 + detour_dist * detour_dist / 1000; // prefer a closest connection over a detour
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
    if (best_detour_score == std::numeric_limits<coord_t>::max())
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
