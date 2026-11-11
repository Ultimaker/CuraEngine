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


namespace cura
{

PointsSet makePositionCandidates(ExtrusionLine& extrusion_line, const std::vector<Point2LL>& last_layer_seams = {})
{
    PointsSet points;

    for (const Point2LL& last_layer_seam : last_layer_seams)
    {
        std::optional<size_t> closest_start_index;
        coord_t closest_distance_squared;
        Point2LL split_position;

#warning dont forget the closed lines
        for (const auto&& [segment_index, segment] : extrusion_line | ranges::views::sliding(2) | ranges::views::enumerate)
        {
            const Point2LL& segment_start = segment[0].p_;
            const Point2LL& segment_end = segment[1].p_;
            const Point2LL closest_on_segment = LinearAlg2D::getClosestOnLineSegment(last_layer_seam, segment_start, segment_end);
            if (closest_on_segment == segment_start || closest_on_segment == segment_end)
            {
                continue;
            }

            const coord_t distance_squared = vSize2(last_layer_seam - closest_on_segment);
            if (! closest_start_index.has_value() || distance_squared < closest_distance_squared)
            {
                closest_distance_squared = distance_squared;
                closest_start_index = segment_index;
                split_position = closest_on_segment;
            }
        }

        if (closest_start_index.has_value())
        {
            const ExtrusionJunction& junction_start = extrusion_line[closest_start_index.value()];
            const ExtrusionJunction& junction_end = extrusion_line[closest_start_index.value() + 1];
            const Point2LL& segment_start = junction_start.p_;
            const Point2LL& segment_end = junction_end.p_;

            if (vSize2(split_position - segment_start) <= EPSILON_SQUARED || vSize2(split_position - segment_end) <= EPSILON_SQUARED)
            {
                continue;
            }

#warning interpolate line width
            extrusion_line.emplace(std::next(extrusion_line.begin(), *closest_start_index + 1), split_position, junction_start.w_, junction_start.perimeter_index_);
        }
    }

    for (const ExtrusionJunction& junction : extrusion_line)
    {
        points.push_back(junction.p_);
    }

    return points;
}

BestElementFinder makeBestCandidateFinderOuterWall(
    BestElementFinder& best_candidate_finder,
    std::shared_ptr<TextureDataProvider> texture_data_provider,
    const PointsSet& points,
    const std::vector<Point2LL>& layer_below_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const Point2LL& target_pos,
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

    /*if (path.force_start_index_.has_value()) // Handles EZSeamType::USER_SPECIFIED with "seam_on_vertex" disabled
    {
        // Use a much smaller distance divider because we want points around the forced points to be filtered out very easily
        constexpr double distance_divider = 1.0;
        constexpr auto distance_type = DistanceScoringCriterion::DistanceType::Euclidian;
        main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, points.at(path.force_start_index_.value()), distance_type,
    distance_divider);
    }
    else */
    if (seam_type == EZSeamType::SHORTEST || seam_type == EZSeamType::USER_SPECIFIED)
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
    if (! seam_overhang_exclusion_areas.empty())
    {
        BestElementFinder::WeighedCriterion overhang_criterion;
        overhang_criterion.weight = 2.0;
        overhang_criterion.criterion = std::make_shared<ExclusionAreaScoringCriterion>(points, seam_overhang_exclusion_areas);
        main_criteria_pass.criteria.push_back(overhang_criterion);
    }

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
}

std::vector<Point2LL> calculateWallsSeamPosition(
    const SliceDataStorage& storage,
    const SliceMeshStorage::Ptr& mesh,
    SliceLayer& layer,
    const std::vector<Point2LL>& layer_seams,
    const EZSeamType seam_type,
    const EZSeamCornerPrefType corner_pref,
    const Point2LL& target_pos,
    const coord_t wall_line_width_0,
    const bool outer_walls)
{
    std::vector<Point2LL> new_layer_seams;

    const AngleDegrees seam_overhang_angle{ mesh->settings.get<AngleDegrees>("seam_overhang_angle") };
    const coord_t layer_height{ mesh->settings.get<coord_t>("layer_height") };
    const size_t wall_0_extruder_nr{ mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr_ };
    const Shape machine_borders{ seam_overhang_angle < 90.0 ? storage.getMachineBorder(wall_0_extruder_nr) : Shape{} };

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

                const PointsSet points{ makePositionCandidates(extrusion_line, extrusion_line.is_outer_wall() ? layer_seams : std::vector<Point2LL>{}) };
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

void calculateSeamPositions(SliceDataStorage& storage)
{
    for (std::shared_ptr<SliceMeshStorage>& mesh : storage.meshes)
    {
        const Settings& settings = mesh->settings;
        const auto seam_type = settings.get<EZSeamType>("z_seam_type");
        const auto corner_pref = settings.get<EZSeamCornerPrefType>("z_seam_corner");
        const Point2LL target_pos(settings.get<coord_t>("z_seam_x"), settings.get<coord_t>("z_seam_y"));
        const auto wall_line_width_0 = settings.get<coord_t>("wall_line_width_0");
        std::vector<Point2LL> layer_seams;

#warning multithread it
        for (SliceLayer& layer : mesh->layers)
        {
#warning restore this (with a proper criterion)
            // if (! disallowed_area_for_seams.empty())
            // {
            //     best_i = pathIfZseamIsInDisallowedArea(best_i.value_or(0), path, 0);
            // }

            constexpr bool outer_walls = true;
            layer_seams = calculateWallsSeamPosition(storage, mesh, layer, layer_seams, seam_type, corner_pref, target_pos, wall_line_width_0, outer_walls);

            constexpr bool not_outer_walls = false;
            calculateWallsSeamPosition(storage, mesh, layer, layer_seams, seam_type, corner_pref, target_pos, wall_line_width_0, not_outer_walls);
        }
    }
}

} // namespace cura