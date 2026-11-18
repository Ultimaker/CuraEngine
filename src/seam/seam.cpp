// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "seam/seam.h"

#include <functional>
#include <memory>
#include <optional>

#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/algorithm/rotate.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/sliding.hpp>
#include <spdlog/spdlog.h>

#include "ExtruderTrain.h"
#include "SliceDataStorage.h"
#include "SliceLayer.h"
#include "SliceMeshStorage.h"
#include "types/AngleDegrees.h"
#include "types/CuraViz.h"
#include "types/EZSeamCornerPrefType.h"
#include "types/EZSeamType.h"
#include "types/geometry/PointsSet.h"
#include "types/scoring/BestElementFinder.h"
#include "types/scoring/CornerScoringCriterion.h"
#include "types/scoring/DistanceScoringCriterion.h"
#include "types/scoring/ExclusionAreaScoringCriterion.h"
#include "types/scoring/RandomScoringCriterion.h"
#include "types/scoring/TextureScoringCriterion.h"
#include "types/scoring/VerticalAlignmentScoringCriterion.h"
#include "utils/linear_alg_2d.h"


namespace curaengine::seam
{
using namespace types;
using namespace types::geometry;
using namespace types::scoring;
using namespace utils;

struct AddSplitJunctionCandidate
{
    Point2LL reference_position;
    Point2LL split_position;
};

using SegmentEvaluator = std::function<std::optional<AddSplitJunctionCandidate>(const Point2LL&, const Point2LL&)>;

/*!
 * Evaluates all the segments of the extrusion line with the given function, and inserts a junction in the best one (the shortest distance).
 * @param segment_evaluator Function called with the start and end points of each segment, which returns either nullopt if the segment is not a candidate, or the candidate data
 * @return The index of the junction at the best split position, or nullopt if no segment was a candidate
 */
static std::optional<size_t> insertBestNewJunction(ExtrusionLine& extrusion_line, const SegmentEvaluator& segment_evaluator)
{
    if (extrusion_line.size() < 3 || ! extrusion_line.is_closed_)
    {
        return std::nullopt;
    }

    struct BestCandidate
    {
        size_t start_index;
        Point2LL segment_start;
        Point2LL segment_end;
        Point2LL split_position;
        double distance_to_ref_squared;
    };

    std::optional<BestCandidate> best_candidate;

    // Extrusion lines are explicitly closed, so there is no need to loop
    for (const auto&& [segment_index, segment] : extrusion_line | ranges::views::sliding(2) | ranges::views::enumerate)
    {
        const Point2LL& segment_start = segment[0].p_;
        const Point2LL& segment_end = segment[1].p_;

        const std::optional<AddSplitJunctionCandidate> candidate = segment_evaluator(segment_start, segment_end);
        if (! candidate.has_value())
        {
            continue;
        }

        const double distance_to_ref_squared = vSize2(candidate->split_position - candidate->reference_position);
        if (! best_candidate.has_value() || distance_to_ref_squared < best_candidate->distance_to_ref_squared)
        {
            best_candidate = BestCandidate{ .start_index = segment_index,
                                            .segment_start = segment_start,
                                            .segment_end = segment_end,
                                            .split_position = candidate->split_position,
                                            .distance_to_ref_squared = distance_to_ref_squared };
        }
    }

    if (! best_candidate.has_value())
    {
        return std::nullopt;
    }

    const Point2LL& split_position = best_candidate->split_position;
    if (vSize2(split_position - best_candidate->segment_start) <= epsilon_squared)
    {
        return best_candidate->start_index;
    }
    if (vSize2(split_position - best_candidate->segment_end) <= epsilon_squared)
    {
        return best_candidate->start_index + 1;
    }

    return extrusion_line.insertJunction(best_candidate->start_index, split_position);
}

static void addCloseToSeamPositionCandidate(ExtrusionLine& extrusion_line, const std::vector<Point2LL>& last_layer_seams)
{
    for (const Point2LL& last_layer_seam : last_layer_seams)
    {
        insertBestNewJunction(
            extrusion_line,
            [&last_layer_seam](const Point2LL& segment_start, const Point2LL& segment_end) -> std::optional<AddSplitJunctionCandidate>
            {
                const Point2LL closest_on_segment = linear_alg_2d::getClosestOnLineSegment(last_layer_seam, segment_start, segment_end);
                return AddSplitJunctionCandidate{ .reference_position = last_layer_seam, .split_position = closest_on_segment };
            });
    }
}

static bool addFixedPositionCandidate(
    ExtrusionLine& extrusion_line,
    std::optional<size_t>& force_start_index,
    const EZSeamType z_seam_type,
    const Point2LL& z_seam_position,
    const Point2LL& models_center)
{
    if (z_seam_type != EZSeamType::UserSpecified)
    {
        return false;
    }

    // For user-defined seams you usually don't _actually_ want the _closest_ point, per-se,
    // since you want the seam-line to be continuous in 3D space.
    // To that end, take the center of the 3D model (not of the current polygon, as that would give the same problems)
    // and project the point along the ray from the center to the request_point.
    const Point2LL ray_origin = models_center;
    const Point2LL request_point = ray_origin + (z_seam_position - ray_origin) * 10;

    const std::optional<size_t> inserted_index = insertBestNewJunction(
        extrusion_line,
        [&ray_origin, &request_point](const Point2LL& segment_start, const Point2LL& segment_end) -> std::optional<AddSplitJunctionCandidate>
        {
            float t, u;
            if (! linear_alg_2d::segmentSegmentIntersection(ray_origin, request_point, segment_start, segment_end, t, u))
            {
                return std::nullopt;
            }

            const Point2LL intersection = ray_origin + (request_point - ray_origin) * t;
            return AddSplitJunctionCandidate{ .reference_position = request_point, .split_position = intersection };
        });

    if (! inserted_index.has_value())
    {
        return false;
    }

    force_start_index = inserted_index;
    return true;
}

static PointsSet makePositionCandidates(
    ExtrusionLine& extrusion_line,
    std::optional<size_t>& force_start_index,
    const EZSeamType z_seam_type,
    const Point2LL& z_seam_position,
    const bool z_seam_on_vertex,
    const Point2LL& models_center,
    const std::vector<Point2LL>& last_layer_seams)
{
    PointsSet points;

    if (! z_seam_on_vertex)
    {
        addCloseToSeamPositionCandidate(extrusion_line, last_layer_seams);
        addFixedPositionCandidate(extrusion_line, force_start_index, z_seam_type, z_seam_position, models_center);
    }

    for (const ExtrusionJunction& junction : extrusion_line)
    {
        points.push_back(junction.p_);
    }

    return points;
}

static void addExclusionAreaCriterion(BestElementFinder::CriteriaPass& pass, const PointsSet& points, const Shape& exclusion_area)
{
    if (! exclusion_area.empty())
    {
        constexpr double overhang_exclusion_weight{ 2.0 };
        pass.appendCriterion(std::make_shared<ExclusionAreaScoringCriterion>(points, exclusion_area), overhang_exclusion_weight);
    }
}

static BestElementFinder makeBestCandidateFinderOuterWall(
    BestElementFinder& best_candidate_finder,
    std::shared_ptr<TextureDataProvider> texture_data_provider,
    const PointsSet& points,
    const std::vector<Point2LL>& layer_below_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const Point2LL& target_pos,
    const std::optional<size_t>& force_start_index,
    const coord_t wall_line_width_0,
    const Shape& seam_overhang_exclusion_areas)
{
    // ## Step 1: define the main criteria to be applied and their weights
    // Standard weight for the "main" selection criterion, depending on the selected strategy. There should be
    // exactly one calculation using this criterion.
    {
        BestElementFinder::CriteriaPass main_criteria_pass;
        main_criteria_pass.outsider_delta_threshold = 0.05;

        if (texture_data_provider)
        {
            constexpr double texture_weight{ 5.0 };
            main_criteria_pass.appendCriterion(std::make_shared<TextureScoringCriterion>(points, texture_data_provider, "seam"), texture_weight);
        }

        BestElementFinder::WeighedCriterion main_criterion;

        if (force_start_index.has_value())
        {
            // Use a much smaller distance divider because we want points around the forced points to be filtered out very easily
            constexpr double distance_divider = 1.0;
            constexpr auto distance_type = DistanceScoringCriterion::DistanceType::Euclidian;
            main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, points.at(force_start_index.value()), distance_type, distance_divider);
        }
        else if (seam_type == EZSeamType::UserSpecified)
        {
            main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, target_pos);
        }
        else if (seam_type == EZSeamType::SharpestCorner && corner_pref != EZSeamCornerPrefType::Plugin)
        {
            main_criterion.criterion = std::make_shared<CornerScoringCriterion>(points, corner_pref);
        }
        else if (seam_type == EZSeamType::Random)
        {
            main_criterion.criterion = std::make_shared<RandomScoringCriterion>();
        }

        if (main_criterion.criterion)
        {
            main_criteria_pass.criteria.push_back(main_criterion);
        }
        else
        {
            spdlog::warn("Missing main criterion calculator");
        }

        // Second criterion with higher weight to avoid overhanging areas
        addExclusionAreaCriterion(main_criteria_pass, points, seam_overhang_exclusion_areas);

        // constexpr double alignment_weight{ 0.2 };
        // main_criteria_pass.appendCriterion(std::make_shared<VerticalAlignmentScoringCriterion>(points, layer_below_seams, wall_line_width_0 * 4), alignment_weight);

        best_candidate_finder.appendCriteriaPass(main_criteria_pass);
    }

    best_candidate_finder.appendSingleCriterionPass(std::make_shared<VerticalAlignmentScoringCriterion>(points, layer_below_seams, wall_line_width_0 * 4));

    // ########## Step 2: add fallback passes for criteria with very similar scores (e.g. corner on a cylinder)
    if (seam_type == EZSeamType::SharpestCorner)
    {
        const AABB path_bounding_box(points);

        { // First fallback strategy is to take points on the back-most position
            const auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::YOnly);
            constexpr double outsider_delta_threshold = 0.01;
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion, outsider_delta_threshold);
        }

        { // Second fallback strategy, in case we still have multiple points that are aligned on Y (e.g. cube), take the right-most point
            const auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::XOnly);
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion);
        }
    }

    return best_candidate_finder;
}

static void makeBestCandidateFinderInnerWall(BestElementFinder& best_candidate_finder, const PointsSet& points, const std::vector<Point2LL>& layer_seams)
{
    // For inner walls, we only try to be close to an outer wall seam. If it is not the case, in the end the starting position will not be set and the 'shortest' strategy
    // will be applied, which is the best we can do in such cases anyway.
    best_candidate_finder.appendSingleCriterionPass(std::make_shared<DistanceScoringCriterion>(points, layer_seams));
}

/*!
 * Checks that the given position is close enough to at least one of the seam positions of the outer walls
 * @param position The candidate position to be checked
 * @param layer_seams The seam positions of the outer walls
 * @param max_distance The maximum allowed distance to any of the seam positions
 */
static bool isCloseToOuterWallSeam(const Point2LL& position, const std::vector<Point2LL>& layer_seams, const coord_t max_distance)
{
    const coord_t max_distance_squared = max_distance * max_distance;
    return ranges::any_of(
        layer_seams,
        [&position, max_distance_squared](const Point2LL& seam)
        {
            return vSize2(position - seam) <= max_distance_squared;
        });
}

static std::vector<Point2LL> calculateWallsSeamPosition(
    SliceLayer& layer,
    const std::vector<Point2LL>& layer_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const Point2LL& target_pos,
    const coord_t wall_line_width_0,
    const bool z_seam_on_vertex,
    const AngleDegrees& seam_overhang_angle,
    const coord_t layer_height,
    const Shape& machine_borders,
    const Point2LL& models_center,
    const coord_t max_inner_wall_seam_distance,
    const bool outer_walls)
{
    std::vector<Point2LL> new_layer_seams;

    for (SliceLayerPart& part : layer.parts)
    {
        for (VariableWidthLines& extrusion_lines : part.wall_toolpaths)
        {
            for (ExtrusionLine& extrusion_line : extrusion_lines)
            {
                if (extrusion_line.is_outer_wall() != outer_walls || ! extrusion_line.is_closed_) // Only inner walls may be non-closed, so for those we can't do much
                {
                    continue;
                }

                std::optional<size_t> force_start_index;
                const PointsSet points{ makePositionCandidates(
                    extrusion_line,
                    force_start_index,
                    seam_type,
                    target_pos,
                    z_seam_on_vertex,
                    models_center,
                    extrusion_line.is_outer_wall() ? layer_seams : std::vector<Point2LL>{}) };

                BestElementFinder best_candidate_finder;
                Shape seam_overhang_exclusion_areas; // Don't move to inner scope, it is used as a reference in the criterion and has to remain alive long enough
                if (outer_walls)
                {
                    if (! machine_borders.empty())
                    {
                        seam_overhang_exclusion_areas = machine_borders.difference(part.getSupportedRegion(layer_height, seam_overhang_angle));
                    }

                    makeBestCandidateFinderOuterWall(
                        best_candidate_finder,
                        layer.texture_data_provider_,
                        points,
                        layer_seams,
                        seam_type,
                        corner_pref,
                        target_pos,
                        force_start_index,
                        wall_line_width_0,
                        seam_overhang_exclusion_areas);
                }
                else
                {
                    makeBestCandidateFinderInnerWall(best_candidate_finder, points, layer_seams);
                }

                const std::optional<size_t> best_seam_candidate = best_candidate_finder.findBestElement(points.size());
                if (best_seam_candidate.has_value() && (outer_walls || isCloseToOuterWallSeam(points[*best_seam_candidate], layer_seams, max_inner_wall_seam_distance)))
                {
                    if (outer_walls)
                    {
                        new_layer_seams.push_back(points[*best_seam_candidate]);
                    }

                    extrusion_line.has_start_position_set_ = true;

                    if (best_seam_candidate.value() != 0)
                    {
                        // Re-order the toolpath so that it starts with the most appropriate seam position
                        ranges::rotate(extrusion_line, std::next(extrusion_line.begin(), *best_seam_candidate));
                    }
                }
            }
        }
    }

    return new_layer_seams;
}

void calculateWallsSeamPositions(SliceDataStorage& storage)
{
    const Point2LL models_center = storage.getModelBoundingBox().flatten().getMiddle();

    for (SliceMeshStorage::Ptr& mesh : storage.meshes)
    {
        const Settings& settings = mesh->settings;
        const auto seam_type = settings.get<EZSeamType>("z_seam_type");

        if (seam_type == EZSeamType::Shortest)
        {
            // We can't calculate shortest beforehand, leave it undone for later
            continue;
        }

        const auto corner_pref = settings.get<EZSeamCornerPrefType>("z_seam_corner");
        const Point2LL target_pos = mesh->getZSeamHint();
        const auto wall_line_width_0 = settings.get<coord_t>("wall_line_width_0");
        const bool z_seam_on_vertex = settings.get<bool>("z_seam_on_vertex");
        const AngleDegrees seam_overhang_angle{ settings.get<AngleDegrees>("seam_overhang_angle") };
        const coord_t layer_height{ settings.get<coord_t>("layer_height") };
        const auto wall_line_count = settings.get<size_t>("wall_line_count");
        const auto wall_line_width_x = settings.get<coord_t>("wall_line_width_x");
        const size_t inner_walls_count = wall_line_count > 0 ? wall_line_count - 1 : 0;
        const coord_t max_inner_wall_seam_distance = (wall_line_width_0 / 2) + (static_cast<coord_t>(inner_walls_count) * wall_line_width_x);
        const size_t wall_0_extruder_nr{ settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ };
        const Shape machine_borders{ seam_overhang_angle < 90.0 ? storage.getMachineBorder(static_cast<int>(wall_0_extruder_nr)) : Shape{} };

        std::vector<Point2LL> layer_seams;

        for (SliceLayer& layer : mesh->layers)
        {
            constexpr bool outer_walls = true;
            layer_seams = calculateWallsSeamPosition(
                layer,
                layer_seams,
                seam_type,
                corner_pref,
                target_pos,
                wall_line_width_0,
                z_seam_on_vertex,
                seam_overhang_angle,
                layer_height,
                machine_borders,
                models_center,
                max_inner_wall_seam_distance,
                outer_walls);

            constexpr bool not_outer_walls = false;
            calculateWallsSeamPosition(
                layer,
                layer_seams,
                seam_type,
                corner_pref,
                target_pos,
                wall_line_width_0,
                z_seam_on_vertex,
                seam_overhang_angle,
                layer_height,
                machine_borders,
                models_center,
                max_inner_wall_seam_distance,
                not_outer_walls);
        }
    }
}

size_t calculateShortestSeamPosition(const PointsSet& candidates, const Point2LL& target_pos, const Shape& seam_exclusion_areas)
{
    BestElementFinder best_candidate_finder;
    BestElementFinder::CriteriaPass pass;

    auto shortest_distance_criterion = std::make_shared<DistanceScoringCriterion>(candidates, target_pos);
    pass.criteria.push_back(BestElementFinder::WeighedCriterion(shortest_distance_criterion));

    addExclusionAreaCriterion(pass, candidates, seam_exclusion_areas);

    best_candidate_finder.appendCriteriaPass(pass);

    return best_candidate_finder.findBestElement(candidates.size()).value_or(0);
}

} // namespace curaengine::seam