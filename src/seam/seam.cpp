// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "seam/seam.h"

#include <memory>

#include <range/v3/algorithm/rotate.hpp>
#include <spdlog/spdlog.h>

#include "ExtruderTrain.h"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/CuraViz.h"
#include "utils/linearAlg2D.h"
#include "utils/scoring/BestElementFinder.h"
#include "utils/scoring/CornerScoringCriterion.h"
#include "utils/scoring/DistanceScoringCriterion.h"
#include "utils/scoring/ExclusionAreaScoringCriterion.h"
#include "utils/scoring/RandomScoringCriterion.h"
#include "utils/scoring/TextureScoringCriterion.h"
#include "utils/scoring/VerticalAlignmentScoringCriterion.h"


namespace cura::seam
{

size_t insertJunction(ExtrusionLine& extrusion_line, const Point2LL& position, size_t after_index)
{
    const size_t i0 = after_index;
    const size_t i1 = (after_index + 1) % extrusion_line.size();
    const ExtrusionJunction& junction_start = extrusion_line[i0];
    const ExtrusionJunction& junction_end = extrusion_line[i1];
    const Point2LL& segment_start = junction_start.p_;
    const Point2LL& segment_end = junction_end.p_;

    if (vSize2(position - segment_start) <= EPSILON_SQUARED)
    {
        return i0;
    }
    if (vSize2(position - segment_end) <= EPSILON_SQUARED)
    {
        return i1;
    }

    return extrusion_line.insertJunction(after_index, position);
}

void addVerticalAlignmentPositionCandidate(ExtrusionLine& extrusion_line, const std::vector<Point2LL>& last_layer_seams)
{
    for (const Point2LL& last_layer_seam : last_layer_seams)
    {
        struct ClosestCandidate
        {
            size_t start_index;
            coord_t distance_squared;
            Point2LL split_position;
        };

        std::optional<ClosestCandidate> closest_candidate;

#warning dont forget the closed lines
        for (const auto&& [segment_index, segment] : extrusion_line | ranges::views::sliding(2) | ranges::views::enumerate)
        {
            const Point2LL& segment_start = segment[0].p_;
            const Point2LL& segment_end = segment[1].p_;
            const Point2LL closest_on_segment = LinearAlg2D::getClosestOnLineSegment(last_layer_seam, segment_start, segment_end);
            const coord_t distance_squared = vSize2(last_layer_seam - closest_on_segment);
            if (! closest_candidate.has_value() || distance_squared < closest_candidate->distance_squared)
            {
                closest_candidate = ClosestCandidate{ .start_index = segment_index, .distance_squared = distance_squared, .split_position = closest_on_segment };
            }
        }

        if (closest_candidate.has_value())
        {
            insertJunction(extrusion_line, closest_candidate->split_position, closest_candidate->start_index);
        }
    }
}

bool addFixedPositionCandidate(
    ExtrusionLine& extrusion_line,
    std::optional<size_t>& force_start_index,
    const EZSeamType z_seam_type,
    const Point2LL& z_seam_position,
    const Point2LL& models_center)
{
    if (! extrusion_line.is_closed_ || extrusion_line.size() < 3 || z_seam_type != EZSeamType::USER_SPECIFIED)
    {
        return false;
    }

    Point2LL request_point = z_seam_position;

    // Find the 'closest' point on the polygon to the request_point.
    Point2LL closest_point;
    size_t closest_junction_idx = 0;
    coord_t closest_distance_sqd = std::numeric_limits<coord_t>::max();

    // For user-defined seams you usually don't _actually_ want the _closest_ point, per-se,
    // since you want the seam-line to be continuous in 3D space.
    // To that end, take the center of the 3D model (not of the current polygon, as that would give the same problems)
    // and project the point along the ray from the center to the request_point.

    const Point2LL ray_origin = models_center;
    request_point = ray_origin + (request_point - ray_origin) * 10;

    for (const auto& [i, junction] : extrusion_line.junctions_ | ranges::views::enumerate)
    {
        // NOTE: Maybe rewrite this once we can use C++23 ranges::views::adjacent
        const auto& next_junction = extrusion_line.junctions_[(i + 1) % extrusion_line.junctions_.size()];

        float t, u;
        if (LinearAlg2D::segmentSegmentIntersection(ray_origin, request_point, junction.p_, next_junction.p_, t, u))
        {
            const Point2LL intersection = ray_origin + (request_point - ray_origin) * t;
            const coord_t distance_sqd = vSize2(request_point - intersection);
            if (distance_sqd < closest_distance_sqd)
            {
                closest_point = intersection;
                closest_distance_sqd = distance_sqd;
                closest_junction_idx = i;
            }
        }
    }

    if (closest_distance_sqd >= std::numeric_limits<coord_t>::max())
    {
        return false;
    }

    const auto& start_pt = extrusion_line.junctions_[closest_junction_idx];
    const auto& end_pt = extrusion_line.junctions_[(closest_junction_idx + 1) % extrusion_line.junctions_.size()];
    closest_point = LinearAlg2D::getClosestOnLineSegment(request_point, start_pt.p_, end_pt.p_);
    force_start_index = insertJunction(extrusion_line, closest_point, closest_junction_idx);
    return true;
}

PointsSet makePositionCandidates(
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
        addVerticalAlignmentPositionCandidate(extrusion_line, last_layer_seams);
        addFixedPositionCandidate(extrusion_line, force_start_index, z_seam_type, z_seam_position, models_center);
    }

    for (const ExtrusionJunction& junction : extrusion_line)
    {
        points.push_back(junction.p_);
    }

    return points;
}

void addExclusionAreaCriterion(BestElementFinder::CriteriaPass& pass, const PointsSet& points, const Shape& exclusion_area)
{
    if (! exclusion_area.empty())
    {
        BestElementFinder::WeighedCriterion overhang_criterion;
        overhang_criterion.weight = 2.0;
        overhang_criterion.criterion = std::make_shared<ExclusionAreaScoringCriterion>(points, exclusion_area);
        pass.criteria.push_back(overhang_criterion);
    }
}

BestElementFinder makeBestCandidateFinderOuterWall(
    BestElementFinder& best_candidate_finder,
    std::shared_ptr<TextureDataProvider> texture_data_provider,
    const PointsSet& points,
    const std::vector<Point2LL>& layer_below_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const Point2LL& target_pos,
    std::optional<size_t>& force_start_index,
    const coord_t wall_line_width_0,
    const Shape& seam_overhang_exclusion_areas)
{
    // ## Step 1: define the main criteria to be applied and their weights
    // Standard weight for the "main" selection criterion, depending on the selected strategy. There should be
    // exactly one calculation using this criterion.
    BestElementFinder::CriteriaPass main_criteria_pass;
    main_criteria_pass.outsider_delta_threshold = 0.1;

    if (texture_data_provider)
    {
        BestElementFinder::WeighedCriterion texture_criterion;
        texture_criterion.criterion = std::make_shared<TextureScoringCriterion>(points, texture_data_provider, "seam");
        texture_criterion.weight = 5.0;
        main_criteria_pass.criteria.push_back(texture_criterion);
    }

#if 0
    {
        BestElementFinder::WeighedCriterion alignment_criterion;
        alignment_criterion.criterion = std::make_shared<VerticalAlignmentScoringCriterion>(points, last_layer_seams, 400);
        alignment_criterion.weight = 0.1;
        main_criteria_pass.criteria.push_back(alignment_criterion);
    }
#endif

    BestElementFinder::WeighedCriterion main_criterion;

    if (force_start_index.has_value())
    {
        // Use a much smaller distance divider because we want points around the forced points to be filtered out very easily
        constexpr double distance_divider = 1.0;
        constexpr auto distance_type = DistanceScoringCriterion::DistanceType::Euclidian;
        main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, points.at(force_start_index.value()), distance_type, distance_divider);
    }
    else if (seam_type == EZSeamType::USER_SPECIFIED)
    {
        main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, target_pos);
    }
    else if (seam_type == EZSeamType::SHARPEST_CORNER && corner_pref != EZSeamCornerPrefType::PLUGIN)
    {
        main_criterion.criterion = std::make_shared<CornerScoringCriterion>(points, corner_pref);
    }
    else if (seam_type == EZSeamType::RANDOM)
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

    best_candidate_finder.appendCriteriaPass(main_criteria_pass);

#if 1
    {
        auto alignment_criterion = std::make_shared<VerticalAlignmentScoringCriterion>(points, layer_below_seams, wall_line_width_0);
        best_candidate_finder.appendSingleCriterionPass(alignment_criterion);
    }
#endif

    // ########## Step 2: add fallback passes for criteria with very similar scores (e.g. corner on a cylinder)
    if (seam_type == EZSeamType::SHARPEST_CORNER)
    {
        const AABB path_bounding_box(points);

        { // First fallback strategy is to take points on the back-most position
            auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::YOnly);
            constexpr double outsider_delta_threshold = 0.01;
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion, outsider_delta_threshold);
        }

        { // Second fallback strategy, in case we still have multiple points that are aligned on Y (e.g. cube), take the right-most point
            auto fallback_criterion = std::make_shared<DistanceScoringCriterion>(points, path_bounding_box.max_, DistanceScoringCriterion::DistanceType::XOnly);
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion);
        }
    }

    return best_candidate_finder;
}

void makeBestCandidateFinderInnerWall(BestElementFinder& best_candidate_finder, const PointsSet& points, const std::vector<Point2LL>& layer_seams)
{
#warning we should use a max distance, and in case there is no seam close enough, just dont calculate and let shortest be applied in just-in-time
    best_candidate_finder.appendSingleCriterionPass(std::make_shared<DistanceScoringCriterion>(points, layer_seams));

    // const bool outer_to_inner = inset_direction == InsetDirection::OUTSIDE_IN;
    // const bool use_one_extruder = wall_0_extruder_nr_ == wall_x_extruder_nr_;
    // const bool current_extruder_is_wall_x = wall_x_extruder_nr_ == extruder_nr_;
    //
    // const bool reverse = shouldReversePath(use_one_extruder, current_extruder_is_wall_x, outer_to_inner);
    // const bool use_shortest_for_inner_walls = outer_to_inner;
}

std::vector<Point2LL> calculateWallsSeamPosition(
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
    const bool outer_walls)
{
    std::vector<Point2LL> new_layer_seams;

    for (SliceLayerPart& part : layer.parts)
    {
        for (VariableWidthLines& extrusion_lines : part.wall_toolpaths)
        {
            for (ExtrusionLine& extrusion_line : extrusion_lines)
            {
                if (extrusion_line.is_outer_wall() != outer_walls)
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
                if (best_seam_candidate.has_value())
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

    for (std::shared_ptr<SliceMeshStorage>& mesh : storage.meshes)
    {
        const Settings& settings = mesh->settings;
        const auto seam_type = settings.get<EZSeamType>("z_seam_type");

        if (seam_type == EZSeamType::SHORTEST)
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

} // namespace cura::seam