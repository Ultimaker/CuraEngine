// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "pathPlanning/Comb.h"

#include <algorithm>
#include <functional> // function
#include <unordered_set>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "pathPlanning/CombPaths.h"
#include "pathPlanning/LinePolygonsCrossings.h"
#include "sliceDataStorage.h"
#include "utils/PolygonsPointIndex.h"
#include "utils/SVG.h"
#include "utils/linearAlg2D.h"

namespace cura
{

LocToLineGrid& Comb::getOutsideLocToLine(const ExtruderTrain& train)
{
    if (outside_loc_to_line_[train.extruder_nr_] == nullptr)
    {
        outside_loc_to_line_[train.extruder_nr_] = PolygonUtils::createLocToLineGrid(getBoundaryOutside(train), offset_from_inside_to_outside_ * 3 / 2);
    }
    return *outside_loc_to_line_[train.extruder_nr_];
}

Shape& Comb::getBoundaryOutside(const ExtruderTrain& train)
{
    if (boundary_outside_[train.extruder_nr_].empty())
    {
        bool travel_avoid_supports = train.settings_.get<bool>("travel_avoid_supports");
        boundary_outside_[train.extruder_nr_] = storage_.getLayerOutlines(layer_nr_, travel_avoid_supports, travel_avoid_supports).offset(travel_avoid_distance_);
    }
    return boundary_outside_[train.extruder_nr_];
}

Shape& Comb::getModelBoundary(const ExtruderTrain& train)
{
    if (model_boundary_[train.extruder_nr_].empty())
    {
        bool travel_avoid_supports = train.settings_.get<bool>("travel_avoid_supports");
        model_boundary_[train.extruder_nr_] = storage_.getLayerOutlines(layer_nr_, travel_avoid_supports, travel_avoid_supports);
    }
    return boundary_outside_[train.extruder_nr_];
}

LocToLineGrid& Comb::getModelBoundaryLocToLine(const ExtruderTrain& train)
{
    if (model_boundary_loc_to_line_[train.extruder_nr_] == nullptr)
    {
        model_boundary_loc_to_line_[train.extruder_nr_] = PolygonUtils::createLocToLineGrid(getModelBoundary(train), offset_from_inside_to_outside_ * 3 / 2);
    }
    return *model_boundary_loc_to_line_[train.extruder_nr_];
}

Comb::Comb(
    const SliceDataStorage& storage,
    const LayerIndex layer_nr,
    const Shape& comb_boundary_inside_minimum,
    const Shape& comb_boundary_inside_optimal,
    coord_t comb_boundary_offset,
    coord_t travel_avoid_distance,
    coord_t move_inside_distance)
    : storage_(storage)
    , layer_nr_(layer_nr)
    , travel_avoid_distance_(travel_avoid_distance)
    , offset_from_outlines_(comb_boundary_offset) // between second wall and infill / other walls
    , max_moveInside_distance2_(offset_from_outlines_ * offset_from_outlines_)
    , max_move_inside_distance_enlarged2_(std::pow(offset_from_outlines_ + max_move_inside_enlarge_distance_, 2))
    , offset_from_inside_to_outside_(offset_from_outlines_ + travel_avoid_distance)
    , max_crossing_dist2_(
          offset_from_inside_to_outside_ * offset_from_inside_to_outside_
          * 2) // so max_crossing_dist = offset_from_inside_to_outside * sqrt(2) =approx 1.5 to allow for slightly diagonal crossings and slightly inaccurate crossing computation
    , boundary_inside_minimum_(comb_boundary_inside_minimum) // copy the boundary, because the partsView_inside will reorder the polygons
    , boundary_inside_optimal_(comb_boundary_inside_optimal) // copy the boundary, because the partsView_inside will reorder the polygons
    , parts_view_inside_minimum_(boundary_inside_minimum_.splitIntoPartsView()) // WARNING !! changes the order of boundary_inside !!
    , parts_view_inside_optimal_(boundary_inside_optimal_.splitIntoPartsView()) // WARNING !! changes the order of boundary_inside !!
    , inside_loc_to_line_minimum_(PolygonUtils::createLocToLineGrid(boundary_inside_minimum_, comb_boundary_offset))
    , inside_loc_to_line_optimal_(PolygonUtils::createLocToLineGrid(boundary_inside_optimal_, comb_boundary_offset))
    , move_inside_distance_(move_inside_distance)
{
}

bool Comb::calc(
    bool perform_z_hops,
    bool perform_z_hops_only_when_collides,
    const ExtruderTrain& train, // NOTE: USe for travel settings and 'extruder-nr' only, don't use for z-hop/retraction/wipe settings, as that should also be settable per mesh!
    Point2LL start_point,
    Point2LL end_point,
    CombPaths& comb_paths,
    bool _start_inside,
    bool _end_inside,
    coord_t max_comb_distance_ignored,
    bool& unretract_before_last_travel_move)
{
    if (shorterThen(end_point - start_point, max_comb_distance_ignored))
    {
        return true;
    }
    const Point2LL travel_end_point_before_combing = end_point;
    // Move start and end point inside the optimal comb boundary
    size_t start_inside_poly = NO_INDEX;
    const bool start_inside = moveInside(boundary_inside_optimal_, _start_inside, inside_loc_to_line_optimal_.get(), start_point, start_inside_poly);

    size_t end_inside_poly = NO_INDEX;
    const bool end_inside = moveInside(boundary_inside_optimal_, _end_inside, inside_loc_to_line_optimal_.get(), end_point, end_inside_poly);

    size_t start_part_boundary_poly_idx = NO_INDEX; // Added initial value to stop MSVC throwing an exception in debug mode
    size_t end_part_boundary_poly_idx = NO_INDEX;
    size_t start_part_idx = (start_inside_poly == NO_INDEX) ? NO_INDEX : parts_view_inside_optimal_.getPartContaining(start_inside_poly, &start_part_boundary_poly_idx);
    size_t end_part_idx = (end_inside_poly == NO_INDEX) ? NO_INDEX : parts_view_inside_optimal_.getPartContaining(end_inside_poly, &end_part_boundary_poly_idx);

    const bool fail_on_unavoidable_obstacles = perform_z_hops && perform_z_hops_only_when_collides;

    // normal combing within part using optimal comb boundary
    if (start_inside && end_inside && start_part_idx == end_part_idx)
    {
        SingleShape part = parts_view_inside_optimal_.assemblePart(start_part_idx);
        comb_paths.emplace_back();
        const bool combing_succeeded = LinePolygonsCrossings::comb(
            part,
            *inside_loc_to_line_optimal_,
            start_point,
            end_point,
            comb_paths.back(),
            -offset_dist_to_get_from_on_the_polygon_to_outside_,
            max_comb_distance_ignored,
            fail_on_unavoidable_obstacles);
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when travelling to that outer wall
        unretract_before_last_travel_move = combing_succeeded && end_point != travel_end_point_before_combing;
        return combing_succeeded;
    }

    // Move start and end point inside the optimal comb boundary
    // Give more tolerancy when calculating move inside positions, because the target points in this case will be on the borders
    size_t start_inside_poly_optimal = NO_INDEX;
    const bool start_inside_optimal
        = moveInside(boundary_inside_optimal_, _start_inside, inside_loc_to_line_optimal_.get(), start_point, start_inside_poly_optimal, max_move_inside_distance_enlarged2_);

    size_t end_inside_poly_optimal = NO_INDEX;
    const bool end_inside_optimal
        = moveInside(boundary_inside_optimal_, _end_inside, inside_loc_to_line_optimal_.get(), end_point, end_inside_poly_optimal, max_move_inside_distance_enlarged2_);

    size_t start_part_boundary_poly_idx_optimal{};
    size_t end_part_boundary_poly_idx_optimal{};
    size_t start_part_idx_optimal
        = (start_inside_poly_optimal == NO_INDEX) ? NO_INDEX : parts_view_inside_optimal_.getPartContaining(start_inside_poly_optimal, &start_part_boundary_poly_idx_optimal);
    size_t end_part_idx_optimal
        = (end_inside_poly_optimal == NO_INDEX) ? NO_INDEX : parts_view_inside_optimal_.getPartContaining(end_inside_poly_optimal, &end_part_boundary_poly_idx_optimal);

    CombPath result_path;
    bool comb_result;

    if (start_inside_optimal && end_inside_optimal && start_part_idx_optimal == end_part_idx_optimal)
    {
        SingleShape part = parts_view_inside_optimal_.assemblePart(start_part_idx_optimal);
        comb_paths.emplace_back();

        comb_result = LinePolygonsCrossings::comb(
            part,
            *inside_loc_to_line_optimal_,
            start_point,
            end_point,
            result_path,
            -offset_dist_to_get_from_on_the_polygon_to_outside_,
            max_comb_distance_ignored,
            fail_on_unavoidable_obstacles);
        Comb::moveCombPathInside(boundary_inside_minimum_, boundary_inside_optimal_, result_path, comb_paths.back()); // add altered result_path to combPaths.back()
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when travelling to that outer wall
        unretract_before_last_travel_move = comb_result && end_point != travel_end_point_before_combing;
        return comb_result;
    }

    // Move start and end point inside the minimum comb boundary
    size_t start_inside_poly_min = NO_INDEX;
    const bool start_inside_min = moveInside(boundary_inside_minimum_, _start_inside, inside_loc_to_line_minimum_.get(), start_point, start_inside_poly_min);

    size_t end_inside_poly_min = NO_INDEX;
    const bool end_inside_min = moveInside(boundary_inside_minimum_, _end_inside, inside_loc_to_line_minimum_.get(), end_point, end_inside_poly_min);

    size_t start_part_boundary_poly_idx_min{};
    size_t end_part_boundary_poly_idx_min{};
    size_t start_part_idx_min
        = (start_inside_poly_min == NO_INDEX) ? NO_INDEX : parts_view_inside_minimum_.getPartContaining(start_inside_poly_min, &start_part_boundary_poly_idx_min);
    size_t end_part_idx_min = (end_inside_poly_min == NO_INDEX) ? NO_INDEX : parts_view_inside_minimum_.getPartContaining(end_inside_poly_min, &end_part_boundary_poly_idx_min);

    // normal combing within part using minimum comb boundary
    if (start_inside_min && end_inside_min && start_part_idx_min == end_part_idx_min)
    {
        SingleShape part = parts_view_inside_minimum_.assemblePart(start_part_idx_min);
        comb_paths.emplace_back();

        comb_result = LinePolygonsCrossings::comb(
            part,
            *inside_loc_to_line_minimum_,
            start_point,
            end_point,
            result_path,
            -offset_dist_to_get_from_on_the_polygon_to_outside_,
            max_comb_distance_ignored,
            fail_on_unavoidable_obstacles);
        Comb::moveCombPathInside(boundary_inside_minimum_, boundary_inside_optimal_, result_path, comb_paths.back()); // add altered result_path to combPaths.back()
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when travelling to that outer wall
        unretract_before_last_travel_move = comb_result && end_point != travel_end_point_before_combing;
        return comb_result;
    }

    // comb inside part to edge (if needed) >> move through air avoiding other parts >> comb inside end part upto the endpoint (if needed)
    //  INSIDE  |          in_between            |            OUTSIDE     |              in_between         |     INSIDE
    //        ^crossing_1_in     ^crossing_1_mid  ^crossing_1_out        ^crossing_2_out    ^crossing_2_mid   ^crossing_2_in
    //
    // when start_point is inside crossing_1_in is of interest
    // when it is in between inside and outside it is equal to crossing_1_mid

    if (perform_z_hops && ! perform_z_hops_only_when_collides) // Combing via outside makes combing fail.
    {
        return false;
    }

    // Find the crossings using the minimum comb boundary, since it's guaranteed to be as close as we can get to the destination.
    // Getting as close as possible prevents exiting the polygon in the wrong direction (e.g. into a hole instead of to the outside).
    Crossing start_crossing(start_point, start_inside_min, start_part_idx_min, start_part_boundary_poly_idx_min, boundary_inside_minimum_, *inside_loc_to_line_minimum_);
    Crossing end_crossing(end_point, end_inside_min, end_part_idx_min, end_part_boundary_poly_idx_min, boundary_inside_minimum_, *inside_loc_to_line_minimum_);

    { // find crossing over the in-between area between inside and outside
        start_crossing.findCrossingInOrMid(parts_view_inside_minimum_, end_point);
        end_crossing.findCrossingInOrMid(parts_view_inside_minimum_, start_crossing.in_or_mid_);
    }

    bool skip_avoid_other_parts_path = false;
    if (vSize2(start_crossing.in_or_mid_ - end_crossing.in_or_mid_) < offset_from_inside_to_outside_ * offset_from_inside_to_outside_ * 4)
    { // parts are next to each other, i.e. the direct crossing will always be smaller than two crossings via outside
        skip_avoid_other_parts_path = true;
    }

    const bool travel_avoid_other_parts = train.settings_.get<bool>("travel_avoid_other_parts");

    if (travel_avoid_other_parts && ! skip_avoid_other_parts_path)
    { // compute the crossing points when moving through air
        // comb through all air, since generally the outside consists of a single part

        bool success = start_crossing.findOutside(train, getBoundaryOutside(train), end_crossing.in_or_mid_, fail_on_unavoidable_obstacles, *this);
        if (! success)
        {
            return false;
        }
        success = end_crossing.findOutside(train, getBoundaryOutside(train), start_crossing.out_, fail_on_unavoidable_obstacles, *this);
        if (! success)
        {
            return false;
        }
    }

    // generate the actual comb paths
    if (start_inside_min)
    {
        // start to boundary
        assert(start_crossing.dest_part_.size() > 0 && "The part we start inside when combing should have been computed already!");
        comb_paths.emplace_back();
        // If we're inside the optimal bound, first try the optimal combing path. If it fails, use the minimum path instead.
        constexpr bool fail_for_optimum_bound = true;
        bool combing_succeeded = start_inside
                              && LinePolygonsCrossings::comb(
                                     boundary_inside_optimal_,
                                     *inside_loc_to_line_optimal_,
                                     start_point,
                                     start_crossing.in_or_mid_,
                                     comb_paths.back(),
                                     -offset_dist_to_get_from_on_the_polygon_to_outside_,
                                     max_comb_distance_ignored,
                                     fail_for_optimum_bound);
        if (! combing_succeeded)
        {
            combing_succeeded = LinePolygonsCrossings::comb(
                start_crossing.dest_part_,
                *inside_loc_to_line_minimum_,
                start_point,
                start_crossing.in_or_mid_,
                comb_paths.back(),
                -offset_dist_to_get_from_on_the_polygon_to_outside_,
                max_comb_distance_ignored,
                fail_on_unavoidable_obstacles);
        }
        if (! combing_succeeded)
        { // Couldn't comb between start point and computed crossing from the start part! Happens for very thin parts when the offset_to_get_off_boundary moves points to outside
          // the polygon
            return false;
        }
    }

    // through air from boundary to boundary
    if (travel_avoid_other_parts && ! skip_avoid_other_parts_path)
    {
        comb_paths.emplace_back();
        comb_paths.throughAir = true;
        if (vSize(start_crossing.in_or_mid_ - end_crossing.in_or_mid_)
            < vSize(start_crossing.in_or_mid_ - start_crossing.out_) + vSize(end_crossing.in_or_mid_ - end_crossing.out_))
        { // via outside is moving more over the in-between zone
            comb_paths.back().push_back(start_crossing.in_or_mid_);
            comb_paths.back().push_back(end_crossing.in_or_mid_);
        }
        else
        {
            CombPath tmp_comb_path;
            bool combing_succeeded = LinePolygonsCrossings::comb(
                getBoundaryOutside(train),
                getOutsideLocToLine(train),
                start_crossing.out_,
                end_crossing.out_,
                tmp_comb_path,
                offset_dist_to_get_from_on_the_polygon_to_outside_,
                max_comb_distance_ignored,
                true);

            if (combing_succeeded)
            {
                // add combing travel moves if the combing was successful
                comb_paths.push_back(tmp_comb_path);
            }
            else
            {
                // if combing is not possible then move directly to the target destination
                // this happens for instance when trying to avoid skin-regions and combing from
                // an origin that is on a hole-boundary to a destination that is on the outline-border
                comb_paths.emplace_back();
                comb_paths.throughAir = true;
                comb_paths.back().cross_boundary = true;
                comb_paths.back().push_back(start_crossing.in_or_mid_);
                comb_paths.back().push_back(end_crossing.in_or_mid_);

                if (fail_on_unavoidable_obstacles)
                {
                    return false;
                }
            }
        }
    }
    else
    { // directly through air (not avoiding other parts)
        comb_paths.emplace_back();
        comb_paths.throughAir = true;
        comb_paths.back().cross_boundary = true; // note: we don't actually know whether this is cross boundary, but it might very well be
        comb_paths.back().push_back(start_crossing.in_or_mid_);
        comb_paths.back().push_back(end_crossing.in_or_mid_);
    }
    if (skip_avoid_other_parts_path)
    {
        if (start_inside == end_inside && start_part_idx == end_part_idx)
        {
            if (start_inside)
            { // both start and end are inside
                comb_paths.back().cross_boundary = PolygonUtils::polygonCollidesWithLineSegment(start_point, end_point, *inside_loc_to_line_optimal_);
            }
            else
            { // both start and end are outside
                comb_paths.back().cross_boundary = PolygonUtils::polygonCollidesWithLineSegment(start_point, end_point, getModelBoundaryLocToLine(train));
            }
        }
        else
        {
            comb_paths.back().cross_boundary = true;
        }
    }

    if (end_inside)
    {
        // boundary to end
        assert(end_crossing.dest_part_.size() > 0 && "The part we end up inside when combing should have been computed already!");
        comb_paths.emplace_back();
        // If we're inside the optimal bound, first try the optimal combing path. If it fails, use the minimum path instead.
        constexpr bool fail_for_optimum_bound = true;
        bool combing_succeeded = end_inside
                              && LinePolygonsCrossings::comb(
                                     boundary_inside_optimal_,
                                     *inside_loc_to_line_optimal_,
                                     end_crossing.in_or_mid_,
                                     end_point,
                                     comb_paths.back(),
                                     -offset_dist_to_get_from_on_the_polygon_to_outside_,
                                     max_comb_distance_ignored,
                                     fail_for_optimum_bound);
        if (! combing_succeeded)
        {
            combing_succeeded = LinePolygonsCrossings::comb(
                end_crossing.dest_part_,
                *inside_loc_to_line_minimum_,
                end_crossing.in_or_mid_,
                end_point,
                comb_paths.back(),
                -offset_dist_to_get_from_on_the_polygon_to_outside_,
                max_comb_distance_ignored,
                fail_on_unavoidable_obstacles);
        }
        // If the endpoint of the travel path changes with combing, then it means that we are moving to an outer wall
        // and we should unretract before the last travel move when traveling to that outer wall
        unretract_before_last_travel_move = combing_succeeded && end_point != travel_end_point_before_combing;
        if (! combing_succeeded)
        { // Couldn't comb between end point and computed crossing to the end part! Happens for very thin parts when the offset_to_get_off_boundary moves points to outside the
          // polygon
            return false;
        }
    }

    return true;
}

// Try to move comb_path_input points inside by the amount of `move_inside_distance` and see if the points are still in boundary_inside_optimal, add result in comb_path_output
void Comb::moveCombPathInside(Shape& boundary_inside, Shape& boundary_inside_optimal, CombPath& comb_path_input, CombPath& comb_path_output)
{
    const coord_t dist = move_inside_distance_;
    const coord_t dist2 = dist * dist;

    if (comb_path_input.size() == 0)
    {
        return;
    }
    comb_path_output.push_back(comb_path_input[0]);
    for (unsigned int point_idx = 1; point_idx < comb_path_input.size() - 1; point_idx++)
    {
        Point2LL new_point = Point2LL(comb_path_input[point_idx]);
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

Comb::Crossing::Crossing(
    const Point2LL& dest_point,
    const bool dest_is_inside,
    const unsigned int dest_part_idx,
    const unsigned int dest_part_boundary_crossing_poly_idx,
    const Shape& boundary_inside,
    const LocToLineGrid& inside_loc_to_line)
    : dest_is_inside_(dest_is_inside)
    , boundary_inside_(boundary_inside)
    , inside_loc_to_line_(inside_loc_to_line)
    , dest_point_(dest_point)
    , dest_part_idx_(dest_part_idx)
{
    if (dest_is_inside)
    {
        dest_crossing_poly_ = &(boundary_inside[dest_part_boundary_crossing_poly_idx]); // initialize with most obvious poly, cause mostly a combing move will move outside the
                                                                                        // part, rather than inside a hole in the part
    }
}

bool Comb::moveInside(
    Shape& boundary_inside,
    bool is_inside,
    LocToLineGrid* inside_loc_to_line,
    Point2LL& dest_point,
    size_t& inside_poly,
    const std::optional<coord_t>& max_move_inside_distance_squared)
{
    if (is_inside)
    {
        ClosestPointPolygon cpp = PolygonUtils::ensureInsideOrOutside(
            boundary_inside,
            dest_point,
            offset_extra_start_end_,
            max_move_inside_distance_squared.value_or(max_moveInside_distance2_),
            &boundary_inside,
            inside_loc_to_line);
        if (! cpp.isValid())
        {
            return false;
        }
        else
        {
            inside_poly = cpp.poly_idx_;
            return true;
        }
    }
    return false;
}

void Comb::Crossing::findCrossingInOrMid(const PartsView& partsView_inside, const Point2LL close_to)
{
    if (dest_is_inside_)
    { // in-case
        // find the point on the start inside-polygon closest to the endpoint, but also kind of close to the start point
        Point2LL _dest_point(dest_point_); // copy to local variable for lambda capture
        std::function<int(Point2LL)> close_towards_start_penalty_function(
            [_dest_point](Point2LL candidate)
            {
                return vSize2((candidate - _dest_point) / 10);
            });
        dest_part_ = partsView_inside.assemblePart(dest_part_idx_);

        ClosestPointPolygon boundary_crossing_point;
        { // set [result] to a point on the destination part closest to close_to (but also a bit close to _dest_point)
            std::unordered_set<unsigned int> dest_part_poly_indices;
            for (unsigned int poly_idx : partsView_inside[dest_part_idx_])
            {
                dest_part_poly_indices.emplace(poly_idx);
            }
            coord_t dist2_score = std::numeric_limits<coord_t>::max();
            std::function<bool(const PolygonsPointIndex&)> line_processor
                = [close_to, _dest_point, &boundary_crossing_point, &dist2_score, &dest_part_poly_indices](const PolygonsPointIndex& boundary_segment)
            {
                if (dest_part_poly_indices.find(boundary_segment.poly_idx_) == dest_part_poly_indices.end())
                { // we're not looking at a polygon from the dest_part
                    return true; // a.k.a. continue;
                }
                Point2LL closest_here = LinearAlg2D::getClosestOnLineSegment(close_to, boundary_segment.p(), boundary_segment.next().p());
                coord_t dist2_score_here = vSize2(close_to - closest_here) + vSize2(_dest_point - closest_here) / 10;
                if (dist2_score_here < dist2_score)
                {
                    dist2_score = dist2_score_here;
                    boundary_crossing_point = ClosestPointPolygon(closest_here, boundary_segment.point_idx_, &boundary_segment.getPolygon(), boundary_segment.poly_idx_);
                }
                return true;
            };
            inside_loc_to_line_.processLine(std::make_pair(dest_point_, close_to), line_processor);
        }

        Point2LL result(boundary_crossing_point.p()); // the inside point of the crossing
        if (! boundary_crossing_point.isValid())
        { // no point has been found in the sparse grid
            result = dest_point_;
        }

        ClosestPointPolygon crossing_1_in_cp = PolygonUtils::ensureInsideOrOutside(
            dest_part_,
            result,
            boundary_crossing_point,
            offset_dist_to_get_from_on_the_polygon_to_outside_,
            &boundary_inside_,
            &inside_loc_to_line_,
            close_towards_start_penalty_function);
        if (crossing_1_in_cp.isValid())
        {
            dest_crossing_poly_ = crossing_1_in_cp.poly_;
            in_or_mid_ = result;
        }
        else
        { // part is too small to be ensuring a point inside with the given distance
            in_or_mid_ = dest_point_; // just use the startPoint or endPoint itself
        }
    }
    else
    { // mid-case
        in_or_mid_ = dest_point_;
    }
}

bool Comb::Crossing::findOutside(const ExtruderTrain& train, const Shape& outside, const Point2LL close_to, const bool fail_on_unavoidable_obstacles, Comb& comber)
{
    out_ = in_or_mid_;
    if (dest_is_inside_ || outside.inside(in_or_mid_, true)) // start in_between
    { // move outside
        Point2LL preferred_crossing_1_out = in_or_mid_ + normal(close_to - in_or_mid_, comber.offset_from_inside_to_outside_);
        std::function<int(Point2LL)> close_to_penalty_function(
            [preferred_crossing_1_out](Point2LL candidate)
            {
                return vSize2((candidate - preferred_crossing_1_out) / 2);
            });
        std::optional<ClosestPointPolygon> crossing_1_out_cpp = PolygonUtils::findClose(in_or_mid_, outside, comber.getOutsideLocToLine(train), close_to_penalty_function);
        if (crossing_1_out_cpp)
        {
            out_ = PolygonUtils::moveOutside(*crossing_1_out_cpp, comber.offset_dist_to_get_from_on_the_polygon_to_outside_);
        }
        else
        {
            PolygonUtils::moveOutside(outside, out_, comber.offset_dist_to_get_from_on_the_polygon_to_outside_);
        }
    }
    int64_t in_out_dist2_1 = vSize2(out_ - in_or_mid_);
    if (dest_is_inside_ && in_out_dist2_1 > comber.max_crossing_dist2_) // moveInside moved too far
    { // if move is too far over in_between
        // find crossing closer by
        assert(dest_crossing_poly_ && "destination crossing poly should have been instantiated!");
        std::shared_ptr<std::pair<ClosestPointPolygon, ClosestPointPolygon>> best = findBestCrossing(train, outside, **dest_crossing_poly_, dest_point_, close_to, comber);
        if (best)
        {
            in_or_mid_ = PolygonUtils::moveInside(best->first, comber.offset_dist_to_get_from_on_the_polygon_to_outside_);
            out_ = PolygonUtils::moveOutside(best->second, comber.offset_dist_to_get_from_on_the_polygon_to_outside_);
        }
        if (fail_on_unavoidable_obstacles && vSize2(out_ - in_or_mid_) > comber.max_crossing_dist2_) // moveInside moved still too far
        {
            return false;
        }
    }
    return true;
}


std::shared_ptr<std::pair<ClosestPointPolygon, ClosestPointPolygon>> Comb::Crossing::findBestCrossing(
    const ExtruderTrain& train,
    const Shape& outside,
    const Polygon& from,
    const Point2LL estimated_start,
    const Point2LL estimated_end,
    Comb& comber)
{
    ClosestPointPolygon* best_in = nullptr;
    ClosestPointPolygon* best_out = nullptr;
    coord_t best_detour_score = std::numeric_limits<coord_t>::max();
    coord_t best_crossing_dist2;
    std::vector<std::pair<ClosestPointPolygon, ClosestPointPolygon>> crossing_out_candidates = PolygonUtils::findClose(from, outside, comber.getOutsideLocToLine(train));
    bool seen_close_enough_connection = false;
    for (std::pair<ClosestPointPolygon, ClosestPointPolygon>& crossing_candidate : crossing_out_candidates)
    {
        const coord_t crossing_dist2 = vSize2(crossing_candidate.first.location_ - crossing_candidate.second.location_);
        if (crossing_dist2 > comber.max_crossing_dist2_ * 2)
        { // preliminary filtering
            continue;
        }

        const coord_t dist_to_start = vSize(crossing_candidate.second.location_ - estimated_start); // use outside location, so that the crossing direction is taken into account
        const coord_t dist_to_end = vSize(crossing_candidate.second.location_ - estimated_end);
        const coord_t detour_dist = dist_to_start + dist_to_end;
        const coord_t detour_score = crossing_dist2 + detour_dist * detour_dist / 1000; // prefer a closest connection over a detour
        // The detour distance is generally large compared to the crossing distance.
        // While the crossing is generally about 1mm across,
        // the distance between an arbitrary point and the boundary may well be a couple of centimetres.
        // So the crossing_dist2 is about 1.000.000 while the detour_dist_2 is in the order of 400.000.000
        // In the end we just want to choose between two points which have the _same_ crossing distance, modulo rounding error.
        if ((! seen_close_enough_connection && detour_score < best_detour_score) // keep the best as long as we havent seen one close enough (so that we may walk along the polygon
                                                                                 // to find a closer connection from it in the code below)
            || (! seen_close_enough_connection && crossing_dist2 <= comber.max_crossing_dist2_) // make the one which is close enough the best as soon as we see one close enough
            || (seen_close_enough_connection && crossing_dist2 <= comber.max_crossing_dist2_
                && detour_score < best_detour_score)) // update to keep the best crossing which is close enough already
        {
            if (! seen_close_enough_connection && crossing_dist2 <= comber.max_crossing_dist2_)
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
        return std::shared_ptr<std::pair<ClosestPointPolygon, ClosestPointPolygon>>();
    }
    if (best_crossing_dist2 > comber.max_crossing_dist2_)
    { // find closer point on line segments, rather than moving between vertices of the polygons only
        PolygonUtils::walkToNearestSmallestConnection(*best_in, *best_out);
        best_crossing_dist2 = vSize2(best_in->location_ - best_out->location_);
        if (best_crossing_dist2 > comber.max_crossing_dist2_)
        {
            return std::shared_ptr<std::pair<ClosestPointPolygon, ClosestPointPolygon>>();
        }
    }
    return std::make_shared<std::pair<ClosestPointPolygon, ClosestPointPolygon>>(*best_in, *best_out);
}

} // namespace cura
