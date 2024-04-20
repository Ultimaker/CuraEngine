// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TreeSupportTipGenerator.h"

#include <chrono>
#include <fstream>
#include <stdio.h>
#include <string>

#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/reverse.hpp>
#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "TreeSupportUtils.h"
#include "infill/SierpinskiFillProvider.h"
#include "settings/EnumSettings.h"
#include "utils/Simplify.h"
#include "utils/ThreadPool.h"
#include "utils/algorithm.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/polygonUtils.h" //For moveInside.


namespace cura
{

TreeSupportTipGenerator::TreeSupportTipGenerator(const SliceMeshStorage& mesh, TreeModelVolumes& volumes_s)
    : config_(mesh.settings)
    , use_fake_roof_(! mesh.settings.get<bool>("support_roof_enable"))
    , volumes_(volumes_s)
    , minimum_support_area_(mesh.settings.get<double>("minimum_support_area"))
    , minimum_roof_area_(! use_fake_roof_ ? mesh.settings.get<double>("minimum_roof_area") : std::max(SUPPORT_TREE_MINIMUM_FAKE_ROOF_AREA, minimum_support_area_))
    , support_roof_layers_(
          mesh.settings.get<bool>("support_roof_enable") ? round_divide(mesh.settings.get<coord_t>("support_roof_height"), config_.layer_height)
          : use_fake_roof_                               ? SUPPORT_TREE_MINIMUM_FAKE_ROOF_LAYERS
                                                         : 0)
    , support_tree_top_rate_( mesh.settings.get<double>("support_tree_top_rate"))
    , support_roof_line_distance_(
          use_fake_roof_ ? (config_.support_pattern == EFillMethod::TRIANGLES ? 3 : (config_.support_pattern == EFillMethod::GRID ? 2 : 1))
                              * (config_.support_line_width * 100 / mesh.settings.get<double>("support_tree_top_rate"))
                        : mesh.settings.get<coord_t>("support_roof_line_distance"))
    , support_outset_(0)
    , // Since we disable support offset when tree support is enabled we use an offset of 0 rather than the setting value mesh.settings.get<coord_t>("support_offset")
    roof_outset_(use_fake_roof_ ? support_outset_ : mesh.settings.get<coord_t>("support_roof_offset"))
    , support_tree_limit_branch_reach_(mesh.settings.get<bool>("support_tree_limit_branch_reach"))
    , support_tree_branch_reach_limit_(support_tree_limit_branch_reach_ ? mesh.settings.get<coord_t>("support_tree_branch_reach_limit") : 0)
    , z_distance_delta_(std::min(config_.z_distance_top_layers + 1, mesh.overhang_areas.size()))
    , xy_overrides_(config_.support_overrides == SupportDistPriority::XY_OVERRIDES_Z)
    , already_inserted_(mesh.overhang_areas.size())
    , support_roof_drawn_(mesh.overhang_areas.size(), Polygons())
    , support_roof_drawn_fractional_(mesh.overhang_areas.size(), Polygons())
    , cradle_data_(mesh.overhang_areas.size())
    , force_minimum_roof_area_(use_fake_roof_ || SUPPORT_TREE_MINIMUM_ROOF_AREA_HARD_LIMIT)
    , force_initial_layer_radius_(retrieveSetting<bool>(mesh.settings, "support_tree_enforce_initial_layer_diameter"))
    , cradle_layers_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_height") / config_.layer_height)
    , cradle_layers_min_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_min_height") / config_.layer_height)
    , cradle_line_count_(retrieveSetting<size_t>(mesh.settings, "support_tree_cradle_line_count"))
    , cradle_length_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_length"))
    , cradle_length_min_(retrieveSetting<coord_t>(mesh.settings, "support_tree_min_cradle_length"))
    , cradle_line_width_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_line_width"))
    , cradle_lines_roof_(! use_fake_roof_ && retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") != "none")
    , cradle_base_roof_(
          ! use_fake_roof_
          && (retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") == "cradle_and_base"
              || retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") == "large_cradle_and_base"))
    , large_cradle_base_(! use_fake_roof_ && retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") == "large_cradle_and_base")
    , cradle_area_threshold_(1000 * 1000 * retrieveSetting<double>(mesh.settings, "support_tree_maximum_pointy_area"))
    , cradle_tip_dtt_(config_.tip_layers * retrieveSetting<double>(mesh.settings, "support_tree_cradle_base_tip_percentage") / 100.0)
    , large_cradle_line_tips_(retrieveSetting<bool>(mesh.settings, "support_tree_large_cradle_line_tips"))
    , cradle_z_distance_layers_(round_divide(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_z_distance") , config_.layer_height))
{

    if(cradle_layers_)
    {
        cradle_layers_ += cradle_z_distance_layers_;
    }
    if(cradle_length_ - cradle_line_width_ >0)
    {
        cradle_length_ -= cradle_line_width_;
        cradle_length_min_ -= cradle_line_width_;
    }

    const double support_overhang_angle = mesh.settings.get<AngleRadians>("support_angle");
    const coord_t max_overhang_speed = (support_overhang_angle < TAU / 4) ? (coord_t)(tan(support_overhang_angle) * config_.layer_height) : std::numeric_limits<coord_t>::max();

    if (max_overhang_speed == 0)
    {
        max_overhang_insert_lag_ = std::numeric_limits<coord_t>::max();
    }
    else
    {
        max_overhang_insert_lag_ = std::max((size_t)round_up_divide(config_.xy_distance, max_overhang_speed / 2), 2 * config_.z_distance_top_layers);
        // ^^^ Cap for how much layer below the overhang a new support point may be added, as other than with regular support every new inserted point may cause extra material and
        // time cost.
        //     Could also be an user setting or differently calculated. Idea is that if an overhang does not turn valid in double the amount of layers a slope of support angle
        //     would take to travel xy_distance, nothing reasonable will come from it. The 2*z_distance_delta is only a catch for when the support angle is very high.
    }

    coord_t connect_length = (config_.support_line_width * 100 / support_tree_top_rate_) + std::max(2 * config_.min_radius - 1.0 * config_.support_line_width, 0.0);
    coord_t support_tree_branch_distance = (config_.support_pattern == EFillMethod::TRIANGLES ? 3 : (config_.support_pattern == EFillMethod::GRID ? 2 : 1)) * connect_length;

    std::vector<coord_t> known_z(mesh.layers.size());

    for (auto [z, layer] : ranges::views::enumerate(mesh.layers))
    {
        known_z[z] = layer.printZ;
    }
    config_.setActualZ(known_z);


    coord_t dtt_when_tips_can_merge = 1;

    if (config_.branch_radius * config_.diameter_angle_scale_factor < 2 * config_.maximum_move_distance_slow)
    {
        while ((2 * config_.maximum_move_distance_slow * dtt_when_tips_can_merge - config_.support_line_width) < config_.getRadius(dtt_when_tips_can_merge))
        {
            dtt_when_tips_can_merge++;
        }
    }
    else
    {
        dtt_when_tips_can_merge = config_.tip_layers; // arbitrary default for when there is no guarantee that the while loop above will terminate
    }
    support_supporting_branch_distance_ = 2 * config_.getRadius(dtt_when_tips_can_merge) + config_.support_line_width + FUDGE_LENGTH;


    for(size_t cradle_z_dist_ctr=0; cradle_z_dist_ctr < cradle_z_distance_layers_ + 1; cradle_z_dist_ctr++)
    {
        cradle_xy_distance_.emplace_back(config_.xy_min_distance);
    }

    for (int i = 0; i < 9; i++)
    {
        std::stringstream setting_name_dist;
        setting_name_dist << "support_tree_cradle_xy_dist_";
        setting_name_dist << i;
        coord_t next_cradle_xy_dist = retrieveSetting<coord_t>(mesh.settings, setting_name_dist.str());
        std::stringstream setting_name_height;
        setting_name_height << "support_tree_cradle_xy_height_";
        setting_name_height << i;
        coord_t next_cradle_xy_dist_height = retrieveSetting<coord_t>(mesh.settings, setting_name_height.str());
        if (next_cradle_xy_dist_height == 0)
        {
            break;
        }

        for (int layer_delta = 0; layer_delta < round_up_divide(next_cradle_xy_dist_height, config_.layer_height); layer_delta++)
        {
            cradle_xy_distance_.emplace_back(next_cradle_xy_dist);
        }
    }

    for (int cradle_xy_dist_fill = cradle_xy_distance_.size(); cradle_xy_dist_fill < cradle_layers_ + 1; cradle_xy_dist_fill++)
    {
        cradle_xy_distance_.emplace_back(config_.xy_min_distance);
    }
}


std::vector<TreeSupportTipGenerator::LineInformation> TreeSupportTipGenerator::convertLinesToInternal(Polygons polylines, LayerIndex layer_idx)
{
    // NOTE: The volumes below (on which '.inside(p, true)' is called each time below) are the same each time. The values being calculated here are strictly local as well.
    //       So they could in theory be pre-calculated here (outside of the loop). However, when I refatored it to be that way, it seemed to cause deadlocks each time for some
    //       settings.
    // NOTE2: When refactoring ensure that avoidance to buildplate is only requested when support_rest_preference == RestPreference::BUILDPLATE as otherwise it has not been
    // precalculated (causing long delays while it is calculated when requested here).

    std::vector<LineInformation> result;
    // Also checks if the position is valid, if it is NOT, it deletes that point
    for (const auto& line : polylines)
    {
        LineInformation res_line;
        for (const Point2LL& p : line)
        {
            if (config_.support_rest_preference == RestPreference::BUILDPLATE
                && ! volumes_.getAvoidance(config_.getRadius(0), layer_idx, AvoidanceType::FAST_SAFE, false, ! xy_overrides_).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_BP_SAFE);
            }
            else if (
                config_.support_rest_preference == RestPreference::BUILDPLATE
                && ! volumes_.getAvoidance(config_.getRadius(0), layer_idx, AvoidanceType::FAST, false, ! xy_overrides_).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_BP);
            }
            else if (config_.support_rests_on_model && ! volumes_.getAvoidance(config_.getRadius(0), layer_idx, AvoidanceType::FAST_SAFE, true, ! xy_overrides_).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS_SAFE);
            }
            else if (config_.support_rests_on_model && ! volumes_.getAvoidance(config_.getRadius(0), layer_idx, AvoidanceType::FAST, true, ! xy_overrides_).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS);
            }
            else if (config_.support_rests_on_model && ! volumes_.getAvoidance(config_.getRadius(0), layer_idx, AvoidanceType::COLLISION, true, ! xy_overrides_).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL);
            }
            else if (! res_line.empty())
            {
                result.emplace_back(res_line);
                res_line.clear();
            }
        }
        if (! res_line.empty())
        {
            result.emplace_back(res_line);
            res_line.clear();
        }
    }

    return result;
}

Polygons TreeSupportTipGenerator::convertInternalToLines(std::vector<TreeSupportTipGenerator::LineInformation> lines)
{
    Polygons result;
    for (const LineInformation& line : lines)
    {
        Polygon path;
        for (const auto& point_data : line)
        {
            path.add(point_data.first);
        }
        result.add(path);
    }
    return result;
}

std::function<bool(std::pair<Point2LL, TreeSupportTipGenerator::LineStatus>)> TreeSupportTipGenerator::getEvaluatePointForNextLayerFunction(size_t current_layer)
{
    std::function<bool(std::pair<Point2LL, LineStatus>)> evaluatePoint = [this, current_layer](std::pair<Point2LL, LineStatus> p)
    {
        if (config_.support_rest_preference != RestPreference::GRACEFUL
            && ! volumes_
                     .getAvoidance(
                         config_.getRadius(0),
                         current_layer - 1,
                         p.second == LineStatus::TO_BP_SAFE ? AvoidanceType::FAST_SAFE : AvoidanceType::FAST,
                         false,
                         ! xy_overrides_)
                     .inside(p.first, true))
        {
            return true;
        }
        if (config_.support_rests_on_model && (p.second != LineStatus::TO_BP && p.second != LineStatus::TO_BP_SAFE))
        {
            if (p.second == LineStatus::TO_MODEL_GRACIOUS || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE)
            {
                return ! volumes_
                             .getAvoidance(
                                 config_.getRadius(0),
                                 current_layer - 1,
                                 p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE ? AvoidanceType::FAST_SAFE : AvoidanceType::FAST,
                                 true,
                                 ! xy_overrides_)
                             .inside(p.first, true);
            }
            else
            {
                return ! volumes_.getAvoidance(config_.getRadius(0), current_layer - 1, AvoidanceType::COLLISION, true, ! xy_overrides_).inside(p.first, true);
            }
        }
        return false;
    };
    return evaluatePoint;
}

std::pair<std::vector<TreeSupportTipGenerator::LineInformation>, std::vector<TreeSupportTipGenerator::LineInformation>> TreeSupportTipGenerator::splitLines(
    std::vector<TreeSupportTipGenerator::LineInformation> lines,
    std::function<bool(std::pair<Point2LL, TreeSupportTipGenerator::LineStatus>)> evaluatePoint)
{
    // Assumes all Points on the current line are valid.

    constexpr bool KEEPING = true;
    constexpr bool FREEING = false;
    std::vector<LineInformation> keep(1);
    std::vector<LineInformation> set_free(1);

    for (const std::vector<std::pair<Point2LL, LineStatus>>& line : lines)
    {
        auto current = KEEPING;
        LineInformation resulting_line;
        for (const std::pair<Point2LL, LineStatus>& me : line)
        {
            if (evaluatePoint(me) == (current == FREEING))
            {
                if (! resulting_line.empty())
                {
                    (current == KEEPING ? keep : set_free).emplace_back(resulting_line);
                    resulting_line.clear();
                }
                current = ! current;
            }
            resulting_line.emplace_back(me);
        }
        if (! resulting_line.empty())
        {
            (current == KEEPING ? keep : set_free).emplace_back(resulting_line);
        }
    }
    return std::pair<
        std::vector<std::vector<std::pair<Point2LL, TreeSupportTipGenerator::LineStatus>>>,
        std::vector<std::vector<std::pair<Point2LL, TreeSupportTipGenerator::LineStatus>>>>(keep, set_free);
}

Polygons TreeSupportTipGenerator::ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points, bool enforce_distance) const
{
    Polygons result;
    for (auto part : input)
    {
        if (part.size() == 0)
        {
            continue;
        }
        const coord_t length = Polygon(part).offset(0).polyLineLength();
        Polygon line;
        coord_t current_distance = std::max(distance, coord_t(FUDGE_LENGTH * 2));
        if (length < 2 * distance && min_points <= 1)
        {
            ClosestPolygonPoint middle_point(part[0], 0, part);
            middle_point = PolygonUtils::walk(middle_point, coord_t(length / 2));
            line.add(middle_point.location_);
        }
        else
        {
            size_t optimal_end_index = part.size() - 1;

            if (part.front() == part.back())
            {
                size_t optimal_start_index = 0;
                // If the polyline was a polygon, there is a high chance it was an overhang. Overhangs that are <60 degree tend to be very thin areas, so lets get the beginning and
                // end of them and ensure that they are supported. The first point of the line will always be supported, so rotate the order of points in this polyline that one of
                // the two corresponding points that are furthest from each other is in the beginning. The other will be manually added (optimal_end_index)
                coord_t max_dist2_between_vertices = 0;
                for (auto [idx, p_a] : part | ranges::views::enumerate | ranges::views::drop_last(1))
                {
                    for (auto [inner_idx, p_b] : part | ranges::views::enumerate | ranges::views::drop_last(1))
                    {
                        if (vSize2(p_a - p_b) > max_dist2_between_vertices)
                        {
                            optimal_start_index = idx;
                            optimal_end_index = inner_idx;
                            max_dist2_between_vertices = vSize2(p_a - p_b);
                        }
                    }
                }
                std::rotate(part.begin(), part.begin() + optimal_start_index, part.end() - 1);
                part[part.size() - 1] = part[0]; // restore that property that this polyline ends where it started.
                optimal_end_index = (optimal_end_index - optimal_start_index + part.size() - 1) % (part.size() - 1);
            }

            while (line.size() < min_points && current_distance >= coord_t(FUDGE_LENGTH * 2))
            {
                line.clear();
                Point2LL current_point = part[0];
                line.add(part[0]);

                bool should_add_endpoint = min_points > 1 || vSize2(part[0] - part[optimal_end_index]) > (current_distance * current_distance);
                bool added_endpoint = ! should_add_endpoint; // If no endpoint should be added all endpoints are already added.

                size_t current_index = 0;
                GivenDistPoint next_point;
                coord_t next_distance = current_distance;
                // Get points so that at least min_points are added and they each are current_distance away from each other. If that is impossible, decrease current_distance a bit.
                // (Regarding the while-loop) The input are lines, that means that the line from the last to the first vertex does not have to exist, so exclude all points that are
                // on this line!
                while (PolygonUtils::getNextPointWithDistance(current_point, next_distance, part, current_index, 0, next_point) && next_point.pos < coord_t(part.size()) - 1)
                {
                    if (! added_endpoint && next_point.pos >= optimal_end_index)
                    {
                        current_index = optimal_end_index;
                        current_point = part[optimal_end_index];
                        added_endpoint = true;
                        line.add(part[optimal_end_index]);
                        continue;
                    }

                    // Not every point that is distance away, is valid, as it may be much closer to another point. This is especially the case when the overhang is very thin.
                    // So this ensures that the points are actually a certain distance from each other.
                    // This assurance is only made on a per polygon basis, as different but close polygon may not be able to use support below the other polygon.
                    coord_t min_distance_to_existing_point_sqd = std::numeric_limits<coord_t>::max();
                    if (enforce_distance)
                    {
                        for (Point2LL p : line)
                        {
                            min_distance_to_existing_point_sqd = std::min(min_distance_to_existing_point_sqd, vSize2(p - next_point.location));
                        }
                    }
                    if (! enforce_distance || min_distance_to_existing_point_sqd >= (current_distance * current_distance))
                    {
                        // viable point was found. Add to possible result.
                        line.add(next_point.location);
                        current_point = next_point.location;
                        current_index = next_point.pos;
                        next_distance = current_distance;
                    }
                    else
                    {
                        if (current_point == next_point.location)
                        {
                            // In case a fixpoint is encountered, better aggressively overcompensate so the code does not become stuck here...
                            spdlog::warn(
                                "Tree Support: Encountered a fixpoint in getNextPointWithDistance. This is expected to happen if the distance (currently {}) is smaller than 100",
                                next_distance);
                            if (next_distance > 2 * current_distance)
                            {
                                // This case should never happen, but better safe than sorry.
                                break;
                            }
                            next_distance += current_distance;
                            continue;
                        }
                        // if the point was too close, the next possible viable point is at least distance-min_distance_to_existing_point away from the one that was just checked.
                        next_distance = std::max(static_cast<coord_t>(current_distance - std::sqrt(min_distance_to_existing_point_sqd)), coord_t(FUDGE_LENGTH * 2));
                        current_point = next_point.location;
                        current_index = next_point.pos;
                    }
                }

                if (! added_endpoint)
                {
                    line.add(part[optimal_end_index]);
                }

                current_distance *= 0.9;
            }
        }
        result.add(line);
    }
    return result;
}


std::shared_ptr<SierpinskiFillProvider> TreeSupportTipGenerator::getCrossFillProvider(const SliceMeshStorage& mesh, coord_t line_distance, coord_t line_width)
{
    if (config_.support_pattern == EFillMethod::CROSS || config_.support_pattern == EFillMethod::CROSS_3D)
    {
        std::pair<coord_t,coord_t> key = std::pair<coord_t,coord_t>(line_distance, line_width);
        if(cross_fill_providers_.contains(key))
        {
            return cross_fill_providers_[key];
        }
        std::lock_guard<std::mutex> critical_section_generate_cross_fill(critical_cross_fill_);

        AABB3D aabb;
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            spdlog::warn("Tree support tried to generate a CrossFillProvider for a non model mesh.");
            return nullptr;
        }

        const coord_t aabb_expansion = mesh.settings.get<coord_t>("support_offset");
        AABB3D aabb_here(mesh.bounding_box);
        aabb_here.include(aabb_here.min_ - Point3LL(-aabb_expansion, -aabb_expansion, 0));
        aabb_here.include(aabb_here.max_ + Point3LL(-aabb_expansion, -aabb_expansion, 0));
        aabb.include(aabb_here);

        const std::string cross_subdisivion_spec_image_file = mesh.settings.get<std::string>("cross_support_density_image");
        std::ifstream cross_fs(cross_subdisivion_spec_image_file.c_str());
        if (cross_subdisivion_spec_image_file != "" && cross_fs.good())
        {
            return std::make_shared<SierpinskiFillProvider>(aabb, line_distance, line_width, cross_subdisivion_spec_image_file);
        }
        return std::make_shared<SierpinskiFillProvider>(aabb, line_distance, line_width);
    }
    return nullptr;
}

void TreeSupportTipGenerator::calculateFloatingParts(const SliceMeshStorage& mesh)
{
    LayerIndex max_layer = 0;

    for(LayerIndex layer_idx = 0; layer_idx < mesh.overhang_areas.size(); layer_idx++)
    {
        if(!mesh.overhang_areas[layer_idx].empty())
        {
            max_layer = layer_idx;
        }
    }
    max_layer = std::min(max_layer + cradle_layers_, LayerIndex(mesh.overhang_areas.size() - 1));

    LayerIndex start_layer = 1;
    floating_parts_cache_.resize(max_layer+1);
    floating_parts_map_.resize(max_layer+1);
    std::mutex critical_sections;

    Polygons completely_supported = volumes_.getCollision(0,0,true);
    Polygons layer_below = completely_supported;  //technically wrong, but the xy distance error on layer 1 should not matter
    for (size_t layer_idx = start_layer; layer_idx < max_layer; layer_idx++)
    {
        Polygons next_completely_supported;

        // As the collision contains z distance and xy distance it cant be used to clearly identify which parts of the model are connected to the buildplate.
        Polygons layer = mesh.layers[layer_idx].getOutlines();

        const std::vector<PolygonsPart> layer_parts = layer.splitIntoParts();
        cura::parallel_for<size_t>(
        0,
        layer_parts.size(),
        [&](const size_t part_idx)
        {
                const PolygonsPart& part = layer_parts[part_idx];
                AABB part_aabb(part);
                bool has_support_below = !PolygonUtils::clipPolygonWithAABB(layer_below,part_aabb).intersection(part).empty();

                if(! completely_supported.intersection(part).empty() || (! has_support_below && part.area() > cradle_area_threshold_))
                {
                    std::lock_guard<std::mutex> critical_section_add(critical_sections);
                    next_completely_supported.add(part);
                    return ;
                }

                Polygons overhang = mesh.overhang_areas[layer_idx].intersection(part);
                coord_t overhang_area = std::max(overhang.area(), std::numbers::pi * config_.min_wall_line_width * config_.min_wall_line_width);
                if (!has_support_below)
                {
                    std::lock_guard<std::mutex> critical_section_add(critical_sections);
                    floating_parts_cache_[layer_idx].emplace_back(part,floating_parts_cache_[layer_idx].size(),0,overhang_area);
                    floating_parts_map_ [layer_idx].emplace_back(std::vector<size_t>());
                    return ;
                }

                size_t min_resting_on_layers = 0;
                coord_t supported_overhang_area = 0;
                bool add = false;
                std::vector<size_t> idx_of_floating_below;
                for (auto [idx, floating] : floating_parts_cache_[layer_idx-1] | ranges::views::enumerate)
                {
                    if(layer_idx > 1 && floating.height < cradle_layers_ - 1 && !floating.area.intersection(part).empty())
                    {
                        idx_of_floating_below.emplace_back(idx);
                        supported_overhang_area += floating.accumulated_supportable_overhang;
                        min_resting_on_layers = std::max(min_resting_on_layers,floating.height);
                        add = true;
                    }
                }

                if(min_resting_on_layers < cradle_layers_ && add && overhang_area + supported_overhang_area < cradle_area_threshold_)
                {
                    std::lock_guard<std::mutex> critical_section_add(critical_sections);
                    for(size_t idx: idx_of_floating_below)
                    {
                        floating_parts_map_[layer_idx-1][idx].emplace_back(floating_parts_cache_[layer_idx].size());
                    }

                    floating_parts_map_[layer_idx].emplace_back(std::vector<size_t>());
                    floating_parts_cache_[layer_idx].emplace_back(part, floating_parts_cache_[layer_idx].size(),min_resting_on_layers+1,overhang_area + supported_overhang_area);
                }
                else
                {
                    std::lock_guard<std::mutex> critical_section_add(critical_sections);
                    next_completely_supported.add(part);
                }
        });
        layer_below = layer;
        completely_supported = next_completely_supported;

    }

}

std::vector<TreeSupportTipGenerator::UnsupportedAreaInformation> TreeSupportTipGenerator::getUnsupportedArea(LayerIndex layer_idx, size_t idx_of_area_below)
{
    std::vector<UnsupportedAreaInformation> result;

    if(layer_idx == 0)
    {
        return result;
    }

    bool has_result = false;

    {
        std::lock_guard<std::mutex> critical_section(*critical_floating_parts_cache_);
        has_result = layer_idx < floating_parts_cache_.size();
    }

    if (has_result)
    {
        std::lock_guard<std::mutex> critical_section(*critical_floating_parts_cache_);
        if (floating_parts_cache_[layer_idx].size())
        {
            for (size_t resting_idx : floating_parts_map_[layer_idx - 1][idx_of_area_below])
            {
                result.emplace_back(floating_parts_cache_[layer_idx][resting_idx]);
            }
        }
    }
    else
    {
        spdlog::error("Requested not calculated unsupported area.");
        return result;
    }

    return result;


}

std::vector<TreeSupportTipGenerator::UnsupportedAreaInformation> TreeSupportTipGenerator::getFullyUnsupportedArea(LayerIndex layer_idx)
{
    std::vector<UnsupportedAreaInformation> result;

    if(layer_idx == 0)
    {
        return result;
    }

    bool has_result = false;
    {
        std::lock_guard<std::mutex> critical_section(*critical_floating_parts_cache_);
        has_result = layer_idx < floating_parts_cache_.size();
    }

    if(has_result)
    {
        std::lock_guard<std::mutex> critical_section(*critical_floating_parts_cache_);
        for (auto [idx, floating_data] : floating_parts_cache_[layer_idx] | ranges::views::enumerate)
        {
            if(floating_data.height == 0)
            {
                result.emplace_back(floating_data);
            }
        }
    }
    else
    {
        spdlog::error("Requested not calculated unsupported area.", layer_idx);
        return result;
    }
    return result;

}

std::vector<std::vector<std::vector<Polygons>>> TreeSupportTipGenerator::generateCradleCenters(const SliceMeshStorage& mesh)
{
    std::mutex critical_dedupe;
    std::vector<std::unordered_set<size_t>> dedupe(mesh.overhang_areas.size());
    std::vector<std::vector<std::vector<Polygons>>> shadows(mesh.overhang_areas.size());
    cura::parallel_for<coord_t>(
        1,
        mesh.overhang_areas.size() - (z_distance_delta_ + 1),
        [&](const LayerIndex layer_idx)
        {
            if (mesh.overhang_areas[layer_idx + z_distance_delta_].empty() || getFullyUnsupportedArea(layer_idx + z_distance_delta_).empty())
            {
                return;
            }

            for (auto pointy_info : getFullyUnsupportedArea(layer_idx + z_distance_delta_))
            {
                AABB overhang_aabb(mesh.overhang_areas[layer_idx + z_distance_delta_]);
                if (PolygonUtils::clipPolygonWithAABB(mesh.overhang_areas[layer_idx + z_distance_delta_], overhang_aabb).intersection(pointy_info.area).empty())
                {
                    // It will be assumed that if it touches this mesh's overhang, it will be part of that mesh.
                    continue;
                }

                std::vector<Polygons> accumulated_model(std::min(cradle_layers_ + z_distance_delta_, mesh.overhang_areas.size() - layer_idx), Polygons());
                std::vector<size_t> all_pointy_idx{ pointy_info.index };

                Point2LL center_prev = Polygon(pointy_info.area.getOutsidePolygons()[0]).centerOfMass();
                std::vector<Point2LL> additional_centers;
                TreeSupportCradle* cradle_main = new TreeSupportCradle(layer_idx,center_prev,shadows[layer_idx].size(), cradle_base_roof_, cradle_layers_min_, cradle_length_min_);
                for(size_t z_distance = 0; z_distance < config_.z_distance_top_layers; z_distance++)
                {
                    accumulated_model[z_distance] = pointy_info.area;
                    cradle_main->centers_.emplace_back(center_prev);
                }
                Polygons shadow; // A combination of all outlines of the model that will be supported with a cradle.
                bool aborted = false;
                bool contacted_other_pointy = false;
                std::vector<Polygons> unsupported_model(accumulated_model.size());
                for (size_t cradle_up_layer = 0; cradle_up_layer < accumulated_model.size(); cradle_up_layer++)
                {

                    // shadow model up => not cradle where model
                    // then drop cradle down
                    // cut into parts => get close to original pointy that are far enough from each other.
                    std::vector<size_t> next_pointy_idx;
                    Polygons model_outline;
                    bool blocked_by_dedupe = false;
                    // The cradle base is below the bottommost unsupported and the first cradle layer is around it, so this will be needed only for the second one and up
                    if (cradle_up_layer > 1)
                    {
                        for (size_t pointy_idx : all_pointy_idx)
                        {
                            for (auto next_pointy_data : getUnsupportedArea(layer_idx + cradle_up_layer - 1 + z_distance_delta_, pointy_idx))
                            {
                                if (next_pointy_data.height
                                    != (cradle_up_layer - 1) + pointy_info.height) // If the area belongs to another pointy overhang stop and let this other overhang handle it
                                {
                                    contacted_other_pointy = true;
                                    continue;
                                }
                                unsupported_model[cradle_up_layer].add(next_pointy_data.area);
                                // Ensure each area is only handles once
                                std::lock_guard<std::mutex> critical_section_cradle(critical_dedupe);
                                if (! dedupe[layer_idx + cradle_up_layer].contains(next_pointy_data.index))
                                {
                                    dedupe[layer_idx + cradle_up_layer].emplace(next_pointy_data.index);
                                    model_outline.add(next_pointy_data.area);
                                    next_pointy_idx.emplace_back(next_pointy_data.index);
                                }
                                else
                                {
                                    blocked_by_dedupe = true;
                                }
                            }
                        }
                        all_pointy_idx = next_pointy_idx;
                    }
                    else
                    {
                        model_outline.add(pointy_info.area);
                    }

                    if (model_outline.empty())
                    {
                        if (cradle_up_layer < cradle_layers_min_)
                        {
                            aborted = true;
                            break;
                        }

                        if (! blocked_by_dedupe)
                        {
                            // The model is surrounded with cradle based on the area above (z distance).
                            // When an area that should have a cradle merges with a buildplate supported area above, it will no longer exist for a cradle.
                            // But if the cradle stops there will be z distance layer between the end of the cradle and said merge.
                            // To reduce the impact an area is estimated where the cradle should be for these areas.
                            Polygons previous_area = shadow;
                            for (size_t cradle_up_layer_z_distance = cradle_up_layer;
                                 cradle_up_layer_z_distance < std::min(cradle_up_layer + z_distance_delta_ - 1, accumulated_model.size() - config_.z_distance_top_layers);
                                 cradle_up_layer_z_distance++)
                            {
                                accumulated_model[cradle_up_layer_z_distance + config_.z_distance_top_layers] = unsupported_model[cradle_up_layer_z_distance].unionPolygons();
                            }
                        }
                        break;
                    }

                    model_outline = model_outline.unionPolygons();
                    shadow = shadow.offset(-config_.maximum_move_distance).unionPolygons(model_outline);
                    accumulated_model[cradle_up_layer + config_.z_distance_top_layers] = shadow;

                    if(cradle_up_layer > 0)
                    {
                        Point2LL shadow_center = Polygon(shadow.getOutsidePolygons()[0]).centerOfMass();
                        coord_t center_move_distance = std::min(config_.maximum_move_distance_slow, cradle_line_width_ /3);
                        center_move_distance = std::min(center_move_distance, vSize(shadow_center-center_prev));
                        center_prev = center_prev + normal(shadow_center-center_prev, center_move_distance);
                        cradle_main->centers_.emplace_back(center_prev);
                    }
                }

                if (aborted)
                {
                    // If aborted remove all model information for the cradle generation except the pointy overhang, as it may be needed to cut a small hole in the large interface
                    // base. todo reimplement that

                    Polygons cradle_0 = accumulated_model[0];
                    accumulated_model.clear();
                    accumulated_model.emplace_back(cradle_0);
                    delete cradle_main;
                }
                else
                {
                    cradle_data_[layer_idx].emplace_back(cradle_main);
                }
                shadows[layer_idx].emplace_back(accumulated_model);
            }
        });

    return shadows;
}

void TreeSupportTipGenerator::generateCradleLines(std::vector<std::vector<std::vector<Polygons>>>& shadow_data)
{
    const coord_t max_cradle_xy_distance = *std::max_element(cradle_xy_distance_.begin(), cradle_xy_distance_.end());

    cura::parallel_for<coord_t>(
        1,
        cradle_data_.size(),
        [&](const LayerIndex layer_idx)
        {
            for (auto [center_idx, cradle] : cradle_data_[layer_idx] | ranges::views::enumerate)
            {
                std::vector<Point2LL> removed_directions;
                const auto& accumulated_model = shadow_data[layer_idx][cradle->shadow_idx_];
                for (auto [idx, model_shadow] : accumulated_model | ranges::views::enumerate)
                {
                    Point2LL center = cradle->getCenter(layer_idx + idx);
                    const coord_t current_cradle_xy_distance = cradle_xy_distance_[idx]; // todo fix crash out of bounds
                    const coord_t current_cradle_length = cradle_length_ + max_cradle_xy_distance - current_cradle_xy_distance;

                    if(cradle->lines_.empty())
                    {
                        cradle->lines_.resize(cradle_line_count_);
                    }

                    if (idx > cradle_z_distance_layers_ && ! model_shadow.empty())
                    {

                        Polygons relevant_forbidden = volumes_.getAvoidance(
                            0,
                            layer_idx + idx,
                            (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                            config_.support_rests_on_model,
                            true);

                        Polygons this_part_influence = model_shadow.offset(current_cradle_xy_distance + cradle_line_width_ / 2);

                        for (size_t layer_offset = 1; layer_offset <= config_.z_distance_bottom_layers && layer_offset <= idx; layer_offset++)
                        {
                            this_part_influence.add(accumulated_model[idx - layer_offset].offset(current_cradle_xy_distance + cradle_line_width_ / 2));
                        }

                        for (coord_t layer_offset = 1; layer_offset <= config_.z_distance_top_layers && layer_offset + idx < accumulated_model.size(); layer_offset++)
                        {
                            const coord_t required_range_x = coord_t(
                                (current_cradle_xy_distance + cradle_line_width_ / 2) - ((layer_offset - (config_.z_distance_top_layers == 1 ? 0.5 : 0)) * (current_cradle_xy_distance + cradle_line_width_ / 2) / config_.z_distance_top_layers));
                            this_part_influence.add(accumulated_model[idx + layer_offset].offset(required_range_x));
                        }

                        this_part_influence = this_part_influence.unionPolygons();

                        coord_t cradle_min_xy_distance_delta = std::max(config_.xy_min_distance - current_cradle_xy_distance, coord_t(0));

                        // Somewhere, Somehow there is a small rounding error which causes small slivers of collision of the model to remain.
                        //  To prevent this offset my the delta before removing the influence of the model.
                        relevant_forbidden = relevant_forbidden.offset(-cradle_min_xy_distance_delta)
                                                 .difference(this_part_influence.offset(cradle_min_xy_distance_delta + EPSILON).unionPolygons())
                                                 .offset(cradle_min_xy_distance_delta)
                                                 .unionPolygons(this_part_influence)
                                                 .unionPolygons(volumes_.getSupportBlocker(layer_idx + idx).offset(cradle_line_width_ / 2));
                        coord_t max_distance2 = 0;
                        for (auto line : model_shadow)
                        {
                            for (Point2LL p : line)
                            {
                                max_distance2 = std::max(max_distance2, vSize2(center - p));
                            }
                        }

                        Polygon max_outer_points = PolygonUtils::makeCircle(center, sqrt(max_distance2) + current_cradle_length * 2.0,
                                                                            std::min((2.0 * std::numbers::pi) / double(cradle_line_count_), 1.9 * std::numbers::pi));

                        // create lines that go from the furthest possible location to the center
                        Polygons lines_to_center;
                        for (Point2LL p : max_outer_points)
                        {
                            Point2LL direction = p - center;
                            lines_to_center.addLine(p, center + normal(direction,config_.support_line_width));
                        }

                        // Subtract the model shadow up until this layer from the lines.
                        if (idx > 0)
                        {
                            lines_to_center = model_shadow.offset(current_cradle_xy_distance + cradle_line_width_ / 2).unionPolygons().differencePolyLines(lines_to_center, false);
                        }
                        // Store valid distances from the center in relation to the direction of the line.
                        // Used to detect if a line may be intersecting another model part.
                        std::vector<std::pair<Point2LL, coord_t>> vector_distance_map;

                        // shorten lines to be at most SUPPORT_TREE_CRADLE_WIDTH long, with the location closest to the center not changing
                        Polygons shortened_lines_to_center;
                        for (auto [line_idx, line] : lines_to_center | ranges::views::enumerate)
                        {
                            bool front_closer = vSize2(line.front() - center) < vSize2(line.back() - center);
                            Point2LL closer = front_closer ? line.front() : line.back();
                            Point2LL further = front_closer ? line.back() : line.front();
                            coord_t cradle_line_length = Polygon(line).polylineLength();
                            if (cradle_line_length < cradle_length_min_)
                            {
                                continue;
                            }
                            if (Polygon(line).polylineLength() <= current_cradle_length)
                            {
                                shortened_lines_to_center.add(line);
                            }
                            else
                            {
                                double scale = (double(current_cradle_length) / double(vSize(further - closer)));
                                Point2LL correct_length = closer + (further - closer) * scale;
                                shortened_lines_to_center.addLine(correct_length, closer);
                            }

                            if (further != closer)
                            {
                                vector_distance_map.emplace_back((further - closer), vSize(center - closer));
                            }
                        }
                        // If a line is drawn, but half of it removed as it would collide with the collision, there may not actually be a print line. The offset should prevent
                        // this.
                        shortened_lines_to_center = relevant_forbidden.differencePolyLines(shortened_lines_to_center, false);

                        // Evaluate which lines are still valid after the avoidance was subtracted
                        for (auto [line_idx, line] : shortened_lines_to_center | ranges::views::enumerate)
                        {
                            bool front_closer = vSize2(line.front() - center) < vSize2(line.back() - center);
                            Point2LL closer = front_closer ? line.front() : line.back();
                            Point2LL further = front_closer ? line.back() : line.front();
                            Point2LL current_direction = further - closer;
                            coord_t distance_from_center = vSize(closer - center);

                            shortened_lines_to_center[line_idx].clear();
                            shortened_lines_to_center[line_idx].add(closer);
                            shortened_lines_to_center[line_idx].add(further);
                            bool keep_line = false;
                            bool found_candidate = false;
                            bool too_short = (vSize(closer - further)) < cradle_length_min_;
                            // a cradle line should also be removed if there will be no way to support it
                            if(idx >= cradle_z_distance_layers_ + 1)
                            {
                                const Polygons actually_forbidden = volumes_.getAvoidance(
                                    config_.getRadius(0),
                                    layer_idx + idx - (cradle_z_distance_layers_ + 1) ,
                                    (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                    config_.support_rests_on_model,
                                    true);
                                too_short |= actually_forbidden.differencePolyLines(shortened_lines_to_center[line_idx].offset(0)).polyLineLength() < config_.support_line_width;
                            }
                            if (! too_short)
                            {
                                for (auto [direction_idx, direction] : vector_distance_map | ranges::views::enumerate)
                                {
                                    double cosine = (dot(direction.first, current_direction)) / double(vSize(current_direction) * vSize(direction.first));
                                    if (cosine > 0.99)
                                    {
                                        found_candidate = true;
                                        // found line, check if there is one that is closer, if not then check if distance to center is expected
                                        bool found_close_line = direction.second + config_.xy_min_distance + config_.min_feature_size > distance_from_center;
                                        if (found_close_line)
                                        {
                                            keep_line = true;
                                            break;
                                        }
                                    }
                                }
                            }

                            bool was_removed = false;

                            for (auto [direction_idx, direction] : removed_directions | ranges::views::enumerate)
                            {
                                double cosine = (dot(direction, current_direction)) / (vSize(current_direction) * vSize(direction));
                                if (cosine > 0.99)
                                {
                                    was_removed = true;
                                }
                            }

                            if (too_short || was_removed || (! keep_line && found_candidate))
                            {
                                if (! was_removed)
                                {
                                    removed_directions.emplace_back(current_direction);
                                }
                                shortened_lines_to_center[line_idx].clear();
                            }
                        }

                        size_t line_remove_idx = 0;

                        while (line_remove_idx < shortened_lines_to_center.size())
                        {
                            if (shortened_lines_to_center[line_remove_idx].empty())
                            {
                                shortened_lines_to_center.remove(line_remove_idx);
                            }
                            else
                            {
                                line_remove_idx++;
                            }
                        }


                        for (auto [next_line_idx, next_line] : shortened_lines_to_center | ranges::views::enumerate)
                        {
                            Point2LL current_direction = next_line.front() - next_line.back();
                            double angle = std::atan2(current_direction.Y, current_direction.X);

                            size_t angle_idx = size_t(std::round(((angle+std::numbers::pi)/(2.0*std::numbers::pi)) * double(cradle_line_count_))) % cradle_line_count_;
                            Polygon line(next_line);
                            //Handle cradle_z_distance_layers by overwriting first element in the vector until valid distance is reached.
                            if(idx <= cradle_z_distance_layers_ + 1 && !cradle->lines_[angle_idx].empty())
                            {
                                cradle->lines_[angle_idx][0] = TreeSupportCradleLine(line,layer_idx+idx, cradle_lines_roof_);
                            }
                            else
                            {
                                cradle->lines_[angle_idx].emplace_back(TreeSupportCradleLine(line,layer_idx+idx, cradle_lines_roof_));
                            }
                        }
                    }
                }

                // enlarge cradle lines below to minimize overhang of cradle lines.
                for (auto [line_idx, cradle_lines] : cradle->lines_ | ranges::views::enumerate)
                {
                    if(!cradle_lines.empty())
                    {
                        Point2LL line_end = cradle_lines.back().line_.back();
                        for (auto [up_idx, line] : cradle_lines | ranges::views::enumerate | ranges::views::reverse)
                        {
                            Point2LL center = cradle->getCenter(line.layer_idx_);
                            if(vSize2(line_end - center) > vSize2(line.line_.back() - center))
                            {
                                Polygons line_extension;
                                line_extension.addLine(line.line_.back(),line_end);
                                coord_t line_length_before = line_extension.polyLineLength();
                                Polygons actually_forbidden = volumes_.getAvoidance(
                                    0,
                                    line.layer_idx_,
                                    (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                    config_.support_rests_on_model,
                                    true);
                                line_extension = actually_forbidden.differencePolyLines(line_extension);

                                if(line_extension.polyLineLength()+EPSILON < line_length_before)
                                {
                                    for(auto line_part:line_extension)
                                    {
                                        bool front_closer = vSize2(line_part.front() - center) < vSize2(line_part.back() - center);
                                        Point2LL closer = front_closer ? line_part.front() : line_part.back();
                                        Point2LL further = front_closer ? line_part.back() : line_part.front();

                                        if(vSize2(closer-line.line_.back() < EPSILON * EPSILON))
                                        {
                                            line_end = further;
                                        }
                                    }
                                }
                                //As the center can move there is no guarantee that the point of the current line lies on the line below.
                                line_end = LinearAlg2D::getClosestOnLine(line_end,line.line_.front(),line.line_.back());
                                line.line_.back() = line_end;
                            }
                        }
                    }
                }
            }
        });
}


void TreeSupportTipGenerator::cleanCradleLineOverlaps()
{
    const coord_t min_distance_between_lines = cradle_line_width_ +
                                               std::max(config_.xy_distance, (config_.fill_outline_gaps ? config_.min_feature_size/ 2 - 5 : config_.min_wall_line_width/ 2 - 5));
    const coord_t max_cradle_xy_distance = *std::max_element(cradle_xy_distance_.begin(), cradle_xy_distance_.end());
    std::vector<std::vector<TreeSupportCradleLine*>> all_cradles_per_layer(cradle_data_.size() + cradle_layers_);
    for (LayerIndex layer_idx = 0; layer_idx < cradle_data_.size(); layer_idx++)
    {
        for (size_t cradle_idx = 0; cradle_idx < cradle_data_[layer_idx].size(); cradle_idx++)
        {
            for (size_t cradle_line_idx = 0; cradle_line_idx < cradle_data_[layer_idx][cradle_idx]->lines_.size(); cradle_line_idx++)
            {
                for (size_t height = 0; height < cradle_data_[layer_idx][cradle_idx]->lines_[cradle_line_idx].size(); height++)
                {
                    TreeSupportCradleLine* result = &cradle_data_[layer_idx][cradle_idx]->lines_[cradle_line_idx][height];
                    all_cradles_per_layer[layer_idx + height].emplace_back(result);
                }
            }
        }
    }

    std::vector<Polygons> removed_lines(cradle_data_.size());

    cura::parallel_for<coord_t>(
        1,
        all_cradles_per_layer.size(),
        [&](const LayerIndex layer_idx)
        {
            std::vector<TreeSupportCradleLine*> all_cradles_on_layer = all_cradles_per_layer[layer_idx];

            std::function<void(size_t, Point2LL)> handleNewEnd = [&](size_t cradle_line_idx, Point2LL new_end)
            {
                auto& line =  (*all_cradles_on_layer[cradle_line_idx]);
                if(LinearAlg2D::pointIsProjectedBeyondLine(new_end, line.line_.front(), line.line_.back())
                    || vSize(new_end - line.line_.front()) < cradle_length_min_)
                {
                    all_cradles_on_layer[cradle_line_idx]->addLineToRemoved(line.line_);
                    all_cradles_on_layer[cradle_line_idx]->line_.clear();
                }
                else
                {
                    all_cradles_on_layer[cradle_line_idx]->line_.back() = new_end;
                }
            };

            for (size_t cradle_idx = 0; cradle_idx < all_cradles_on_layer.size(); cradle_idx++)
            {
                AABB bounding_box_current = AABB(all_cradles_on_layer[cradle_idx]->line_);
                bounding_box_current.expand(min_distance_between_lines);
                for (size_t cradle_idx_inner = cradle_idx + 1; cradle_idx_inner < all_cradles_on_layer.size(); cradle_idx_inner++)
                {
                    if(all_cradles_on_layer[cradle_idx_inner]->line_.empty()||all_cradles_on_layer[cradle_idx]->line_.empty())
                    {
                        continue ;
                    }
                    if (bounding_box_current.hit(AABB(all_cradles_on_layer[cradle_idx_inner]->line_)))
                    {
                        Polygon& outer_line = (*all_cradles_on_layer[cradle_idx]).line_;
                        Polygon& inner_line = (*all_cradles_on_layer[cradle_idx_inner]).line_;
                        Point2LL intersect;
                        if (LinearAlg2D::lineLineIntersection(outer_line.front(), outer_line.back(), inner_line.front(), inner_line.back(), intersect)
                            && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, outer_line.front(), outer_line.back())
                            && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, inner_line.front(), inner_line.back()))
                        {
                            coord_t inner_intersect_dist = vSize(inner_line.front() - intersect);
                            coord_t outer_intersect_dist = vSize(outer_line.front() - intersect);

                            if (inner_intersect_dist > outer_intersect_dist)
                            {
                                //this does not ensure that the line ends will not touch. Line ends not touching is handled later
                                Point2LL new_end_inner = intersect + normal((inner_line.front() - intersect), min_distance_between_lines);
                                handleNewEnd(cradle_idx_inner,new_end_inner);

                            }
                            if (outer_intersect_dist > inner_intersect_dist)
                            {
                                Point2LL new_end_outer = intersect + normal((outer_line.front() - intersect), min_distance_between_lines);
                                handleNewEnd(cradle_idx,new_end_outer);
                            }
                        }

                        if(!outer_line.empty() && !inner_line.empty())
                        {
                            // Touching lines have the same issue Lines touch if the end is to close to another line
                            Point2LL inner_direction = inner_line.back() - inner_line.front();
                            Point2LL outer_direction = outer_line.back() - outer_line.front();
                            double cosine = std::abs((dot(inner_direction, outer_direction)) / double(vSize(outer_direction) * vSize(inner_direction)));
                            // If both lines point in the same/opposite direction check that them being to close is not the end line of one to the start of the other
                            if (cosine < 0.99 || vSize2(outer_line.back() - inner_line.back()) + EPSILON * EPSILON < vSize2(outer_line.front() - inner_line.front()))
                            {
                                coord_t inner_end_to_outer_distance = sqrt(LinearAlg2D::getDist2FromLineSegment(outer_line.front(), inner_line.back(), outer_line.back()));
                                if(inner_end_to_outer_distance < min_distance_between_lines && inner_end_to_outer_distance < vSize(outer_line.front() - inner_line.front()))
                                {
                                    Point2LL new_end_inner = inner_line.back() + normal(inner_line.front()-inner_line.back(),min_distance_between_lines - inner_end_to_outer_distance);
                                    double error = min_distance_between_lines - sqrt(LinearAlg2D::getDist2FromLineSegment(outer_line.front(), new_end_inner, outer_line.back()));
                                    double error_correction_factor = 1.0 + error/double(min_distance_between_lines - inner_end_to_outer_distance);
                                    new_end_inner = inner_line.back() +
                                                    normal(inner_line.front()-inner_line.back(),(min_distance_between_lines - inner_end_to_outer_distance)*error_correction_factor);
                                    handleNewEnd(cradle_idx_inner,new_end_inner);
                                }
                                else
                                {
                                    coord_t outer_end_to_inner_distance = sqrt(LinearAlg2D::getDist2FromLineSegment(inner_line.front(), outer_line.back(), inner_line.back()));
                                    if(outer_end_to_inner_distance < min_distance_between_lines && outer_end_to_inner_distance < vSize(outer_line.front() - inner_line.front()))
                                    {
                                        Point2LL new_end_outer = outer_line.back() + normal(outer_line.front()-outer_line.back(),min_distance_between_lines - outer_end_to_inner_distance);
                                        double error = min_distance_between_lines - sqrt(LinearAlg2D::getDistFromLine(new_end_outer,outer_line.front(), outer_line.back()));
                                        double error_correction_factor = 1.0 + error/double(min_distance_between_lines - outer_end_to_inner_distance);
                                        new_end_outer = outer_line.back() +
                                                        normal(outer_line.front()-outer_line.back(),(min_distance_between_lines - outer_end_to_inner_distance)*error_correction_factor);
                                        handleNewEnd(cradle_idx,new_end_outer);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });

    cura::parallel_for<coord_t>(
    1,
    cradle_data_.size(),
    [&](const LayerIndex layer_idx)
    {
            for (size_t cradle_idx = 0; cradle_idx < cradle_data_[layer_idx].size(); cradle_idx++)
            {
                TreeSupportCradle* cradle = cradle_data_[layer_idx][cradle_idx];
                cradle->verifyLines();
                // As cradle lines (causing lines below to be longer) may have been removed to prevent them intersecting, all cradle lines are now shortened again if required.
                for (auto [line_idx, cradle_lines] : cradle->lines_ | ranges::views::enumerate)
                {
                    if(!cradle_lines.empty())
                    {
                        Point2LL line_end = cradle_lines.back().line_.back();
                        Point2LL line_front_uppermost = cradle_lines.back().line_.front();

                        if(vSize2(line_end - cradle_lines.back().line_.front()) > cradle_length_ * cradle_length_)
                        {
                            coord_t current_cradle_xy_distance = cradle_xy_distance_[cradle_lines.back().layer_idx_ - layer_idx];
                            coord_t current_cradle_length = cradle_length_ + max_cradle_xy_distance - current_cradle_xy_distance;
                            cradle_lines.back().line_.back() = line_front_uppermost + normal(line_end - line_front_uppermost, current_cradle_length);
                            for (auto [up_idx, line] : cradle_lines | ranges::views::enumerate | ranges::views::reverse)
                            {
                                Point2LL center = cradle->getCenter(line.layer_idx_);
                                Point2LL line_back_inner = line.line_.back();
                                Point2LL line_front_inner = line.line_.front();
                                if(vSize2(line_back_inner - line_front_inner) > cradle_length_ * cradle_length_)
                                {
                                    //As the center can move there is no guarantee that the point of the current line lies on the line below.
                                    Point2LL projected_line_end = LinearAlg2D::getClosestOnLine(line_end,line.line_.front(),line.line_.back());
                                    current_cradle_xy_distance = cradle_xy_distance_[line.layer_idx_ - layer_idx];
                                    current_cradle_length = cradle_length_ + max_cradle_xy_distance - current_cradle_xy_distance;
                                    if(vSize2(line_front_inner - projected_line_end) > current_cradle_length * current_cradle_length)
                                    {
                                        line.line_.back() = projected_line_end;
                                    }
                                    else
                                    {
                                        line.line_.back() = line_front_inner +  normal(line_back_inner - line_front_inner, current_cradle_length);
                                    }
                                }
                                line_end = line.line_.back();
                            }
                        }
                    }
                }
            }
    });
}


void TreeSupportTipGenerator::generateCradleLineAreasAndBase(std::vector<std::vector<std::vector<Polygons>>>& shadow_data, std::vector<Polygons>& support_free_areas)
{

        const coord_t min_distance_between_lines = FUDGE_LENGTH + (config_.fill_outline_gaps ? config_.min_feature_size/ 2 - 5 : config_.min_wall_line_width/ 2 - 5); // based on calculation in WallToolPath
        // Some angles needed to move cradle lines outwards to prevent them from toughing.
        double center_angle = (2.0 * std::numbers::pi) / double(cradle_line_count_);
        double outer_angle = (std::numbers::pi - center_angle) / 2;
        coord_t outer_radius = (double(min_distance_between_lines + config_.support_line_width) / sin(center_angle)) * sin(outer_angle);
        const coord_t small_hole_size
            = EPSILON + (config_.fill_outline_gaps ? config_.min_feature_size / 2 - 5 : config_.min_wall_line_width / 2 - 5); // based on calculation in WallToolPath
        std::mutex critical_support_free_areas_and_cradle_areas;

        cura::parallel_for<coord_t>(
            0,
            cradle_data_.size(),
            [&](const LayerIndex layer_idx)
            {
                std::vector<TreeSupportCradle *> valid_cradles;
                for (size_t cradle_idx = 0; cradle_idx < cradle_data_[layer_idx].size(); cradle_idx++)
                {
                    TreeSupportCradle& cradle = *cradle_data_[layer_idx][cradle_idx];
                    for (size_t cradle_height = 0; cradle_height <= cradle_layers_; cradle_height++)
                    {
                        Polygons line_tips;

                        std::vector<std::pair<Point2LL, Point2LL>> all_tips_center;
                        // generate trapezoid line tip with front width of support line width, back cradle_width.

                        for(size_t line_idx = 0;line_idx < cradle.lines_.size(); line_idx++)
                        {

                            std::optional<TreeSupportCradleLine*> line_opt = cradle.getCradleLineOfIndex(layer_idx+cradle_height,line_idx);
                            if(!line_opt)
                            {
                                continue;
                            }
                            TreeSupportCradleLine* cradle_line = line_opt.value();
                            Polygon line = cradle_line->line_;

                            coord_t current_cradle_line_width = cradle_line_width_;

                            double assumed_half_center_angle = std::numbers::pi / (1.5 * cradle_line_count_);
                            coord_t triangle_length = cradle_line_count_ <= 2 ? 0 : ((current_cradle_line_width - config_.support_line_width) / 2) *
                                                                                        tan(std::numbers::pi / 2 - assumed_half_center_angle);

                            const coord_t line_length = line.polylineLength();
                            if(triangle_length >= line_length + cradle_line_width_)
                            {
                                triangle_length = line_length + cradle_line_width_;
                                current_cradle_line_width = config_.support_line_width +
                                    2 * triangle_length * tan(assumed_half_center_angle);
                            }

                            Point2LL direction = line.back() - line.front();
                            Point2LL center_front = line.front() - normal(direction, cradle_line_width_ / 2);

                            Point2LL direction_up_center = normal(rotate(direction, std::numbers::pi / 2), config_.support_line_width / 2);
                            Point2LL center_up = center_front + direction_up_center;
                            Point2LL center_down = center_front - direction_up_center;

                            coord_t tip_shift = 0;
                            for (auto existing_center : all_tips_center)
                            {
                                Point2LL intersect;
                                bool centers_touch = LinearAlg2D::lineLineIntersection(center_up, center_down, existing_center.first, existing_center.second, intersect)
                                                  && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, existing_center.first, existing_center.second)
                                                  && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, center_up, center_down);
                                if (centers_touch
                                    || std::min(vSize(center_down - existing_center.first), vSize(center_up - existing_center.second)) < min_distance_between_lines)
                                {
                                    // This cradle line is to close to another.
                                    // Move it back todo If line gets smaller than min length => abort
                                    coord_t tip_shift_here = outer_radius - vSize(center_front - cradle.getCenter(cradle_line->layer_idx_));
                                    tip_shift += tip_shift_here;
                                    center_front = center_front + normal(direction, tip_shift_here);
                                    center_up = center_front + direction_up_center;
                                    center_down = center_front - direction_up_center;
                                }
                            }

                            Point2LL back_center = center_front + normal(direction, triangle_length);
                            Point2LL direction_up_back = normal(rotate(direction, std::numbers::pi / 2), current_cradle_line_width / 2);

                            Point2LL back_up = back_center + direction_up_back;
                            Point2LL back_down = back_center - direction_up_back;

                            line.front() = back_center + normal(direction, current_cradle_line_width / 2 - FUDGE_LENGTH / 2);
                            all_tips_center.emplace_back(center_up, center_down);

                            Polygon line_tip;
                            line_tip.add(back_down);
                            if(current_cradle_line_width == cradle_line_width_)
                            {
                                coord_t distance_end_front = line_length - triangle_length + cradle_line_width_ - tip_shift;
                                Point2LL line_end_down = back_down + normal(direction, distance_end_front);
                                Point2LL line_end_up = back_up + normal(direction, distance_end_front);
                                line_tip.add(line_end_down);
                                line_tip.add(line_end_up);
                            }
                            line_tip.add(back_up);
                            line_tip.add(center_up);
                            line_tip.add(center_down);
                            if (line_tip.area() < 0)
                            {
                                line_tip.reverse();
                            }
                            cradle_line->area_.add(line_tip);

                            Polygons anti_preferred = cradle_line->area_.offset(config_.xy_distance);
                            std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                            for(size_t z_distance_idx = 0; z_distance_idx < config_.z_distance_top_layers; z_distance_idx++)
                            {
                                volumes_.addAreaToAntiPreferred(anti_preferred, layer_idx + cradle_height + z_distance_idx);
                            }
                        }

                    }

                    Polygons shadow = shadow_data[layer_idx][cradle.shadow_idx_][0];
                    Polygons cradle_base = shadow;

                    if (! use_fake_roof_ && support_roof_layers_)
                    {
                        Polygons cut_line_base;
                        Polygons first_cradle_areas;
                        if (large_cradle_base_)
                        {
                            // If a large cradle base is used there needs to be a small hole cut into it to ensure that there will be a line for the pointy overhang to rest
                            // on. This line is just a diagonal though the original pointy overhang area (from min to max). Technically this line is covering more than just
                            // the overhang, but that should not cause any issues.
                            Point2LL min_p = cradle_base.min();
                            Point2LL max_p = cradle_base.max();
                            Polygons rest_line;
                            rest_line.addLine(min_p, max_p);
                            cut_line_base = rest_line.offsetPolyLine(small_hole_size);
                        }
                        {
                            std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                            for (size_t interface_down = 0; interface_down < layer_idx && interface_down < support_roof_layers_; interface_down++)
                            {
                                support_free_areas[layer_idx - interface_down].add(cut_line_base);
                                volumes_.addAreaToAntiPreferred(cradle_base, layer_idx - interface_down);
                            }
                        }
                        if (large_cradle_base_)
                        {
                            Polygons forbidden_here =
                                volumes_.getAvoidance(0, layer_idx, (only_gracious_||!config_.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config_.support_rests_on_model, ! xy_overrides_);

                            cradle_base = cradle_base.offset(config_.getRadius(cradle_tip_dtt_), ClipperLib::jtRound).difference(forbidden_here);
                            Polygons center_removed = cradle_base.difference(cut_line_base);
                            if(center_removed.area()>1)
                            {
                                cradle_base = center_removed;
                            }
                        }
                        else if(cradle_base_roof_)
                        {
                            // collect all inner points and connect to center for thin cradle base
                            Polygons connected_cradle_base;
                            for(size_t line_idx = 0;line_idx<cradle.lines_.size();line_idx++)
                            {
                                std::optional<TreeSupportCradleLine*> line_opt = cradle.getCradleLineOfIndex(layer_idx + cradle_z_distance_layers_ +1, line_idx);
                                if(line_opt)
                                {
                                    connected_cradle_base.addLine(cradle.getCenter(line_opt.value()->layer_idx_),line_opt.value()->line_.front());
                                }
                            }
                            Polygons relevant_forbidden =
                                volumes_.getAvoidance(0, layer_idx, (only_gracious_||!config_.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config_.support_rests_on_model, ! xy_overrides_);
                            cradle_base = connected_cradle_base.offsetPolyLine(cradle_line_width_ /2).unionPolygons(cradle_base).difference(relevant_forbidden);
                        }
                    }

                    cradle_base = cradle_base.unionPolygons();

                    if(cradle_lines_roof_)
                    {
                        Polygons forbidden_here =
                            volumes_.getAvoidance(0, layer_idx, (only_gracious_||!config_.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config_.support_rests_on_model, ! xy_overrides_);
                        std::vector<std::pair<Polygons,int32_t>> roofs;
                        roofs.emplace_back(cradle_base, -1);

                        for(size_t line_idx = 0;line_idx < cradle.lines_.size(); line_idx++)
                        {
                            std::optional<TreeSupportCradleLine*> line_opt = cradle.getCradleLineOfIndex(layer_idx + cradle_z_distance_layers_ + 1,line_idx);
                            if(!line_opt)
                            {
                                continue;
                            }

                            coord_t line_base_offset = large_cradle_base_ ? std::max(coord_t(0),config_.getRadius(cradle_tip_dtt_) - cradle_line_width_ /2) : 0;
                            roofs.emplace_back(line_opt.value()->area_.offset(line_base_offset,ClipperLib::jtRound).difference(forbidden_here),line_idx);
                        }


                        for(auto roof_area_pair : roofs)
                        {
                            Polygons full_overhang_area = TreeSupportUtils::safeOffsetInc(
                                roof_area_pair.first,roof_outset_,forbidden_here, config_.support_line_width, 0, 1, config_.support_line_distance / 2, &config_.simplifier);
                            for (LayerIndex dtt_roof = 0; dtt_roof <= support_roof_layers_ && layer_idx - dtt_roof >= 1; dtt_roof++)
                            {
                                const Polygons forbidden_next =
                                    volumes_.getAvoidance(0, layer_idx - (dtt_roof + 1), (only_gracious_||!config_.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config_.support_rests_on_model, ! xy_overrides_);

                                Polygons roof_area_before = full_overhang_area;
                                full_overhang_area = full_overhang_area.difference(forbidden_next);

                                if (full_overhang_area.area()>EPSILON && dtt_roof < support_roof_layers_)
                                {
                                    if(roof_area_pair.second != -1) //If is_line
                                    {
                                        TreeSupportCradleLine roof_base_line(cradle.lines_[roof_area_pair.second].front());
                                        roof_base_line.area_ = full_overhang_area;
                                        roof_base_line.is_base_ = true;
                                        roof_base_line.layer_idx_ = layer_idx - dtt_roof;
                                        cradle.lines_[roof_area_pair.second].emplace_front(roof_base_line);
                                    }
                                    else
                                    {
                                        if(dtt_roof < cradle.base_below_.size())
                                        {
                                            cradle.base_below_[dtt_roof].add(full_overhang_area);
                                        }
                                        else
                                        {
                                            cradle.base_below_.emplace_back(full_overhang_area);
                                        }
                                    }
                                    const Polygons forbidden_before =
                                        volumes_.getAvoidance(0, layer_idx - dtt_roof, (only_gracious_||!config_.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config_.support_rests_on_model, ! xy_overrides_);

                                    Polygons supported_by_roof_below = TreeSupportUtils::safeOffsetInc(full_overhang_area,
                                                                                                config_.maximum_move_distance,
                                                                                                forbidden_before,
                                                                                                config_.xy_min_distance + config_.min_feature_size,
                                                                                                0,
                                                                                                1,
                                                                                                config_.support_line_distance / 2,
                                                                                                &config_.simplifier);
                                    Polygons overhang_part = roof_area_before.difference(supported_by_roof_below);
                                    if(overhang_part.area()>EPSILON)
                                    {
                                        OverhangInformation cradle_overhang(overhang_part,true, cradle_data_[layer_idx][cradle_idx],layer_idx-dtt_roof,roof_area_pair.second);
                                        cradle.overhang_[layer_idx-dtt_roof].emplace_back(cradle_overhang);
                                    }
                                }
                                else
                                {
                                    if(roof_area_before.area()>1)
                                    {
                                        LayerIndex line_layer_idx = roof_area_pair.second<0 ? LayerIndex(-1) : cradle_data_[layer_idx][cradle_idx]->lines_[roof_area_pair.second].front().layer_idx_;
                                        OverhangInformation cradle_overhang(roof_area_before, true, cradle_data_[layer_idx][cradle_idx], line_layer_idx, roof_area_pair.second);
                                        cradle.overhang_[layer_idx-dtt_roof].emplace_back(cradle_overhang);
                                    }
                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        Polygons forbidden_here =
                            volumes_.getAvoidance(0, layer_idx, (only_gracious_||!config_.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config_.support_rests_on_model, ! xy_overrides_);

                        coord_t offset_radius = config_.getRadius(cradle_tip_dtt_);
                        if(cradle_base.offset(offset_radius,ClipperLib::jtRound).difference(forbidden_here).area()>1)
                        {
                            OverhangInformation cradle_overhang(cradle_base, false, cradle_data_[layer_idx][cradle_idx]);
                            cradle.overhang_[layer_idx].emplace_back(cradle_overhang);
                        }

                        for(size_t line_idx = 0;line_idx < cradle.lines_.size(); line_idx++)
                        {
                            if(!cradle.lines_[line_idx].empty())
                            {
                                LayerIndex support_cradle_on_layer_idx = cradle.lines_[line_idx].front().layer_idx_ - (cradle_z_distance_layers_ + 1);
                                OverhangInformation line_overhang(cradle.lines_[line_idx].front().area_,false,
                                    cradle_data_[layer_idx][cradle_idx],cradle.lines_[line_idx].front().layer_idx_,line_idx);
                                cradle.overhang_[support_cradle_on_layer_idx].emplace_back(line_overhang);
                            }
                        }
                    }
                    if(!cradle.overhang_.empty())
                    {
                        valid_cradles.emplace_back(cradle_data_[layer_idx][cradle_idx]);
                    }
                    else
                    {
                        delete cradle_data_[layer_idx][cradle_idx];
                    }
                }
                cradle_data_[layer_idx] = valid_cradles;
            });

}


void TreeSupportTipGenerator::generateCradle(const SliceMeshStorage& mesh, std::vector<Polygons>& support_free_areas)
{
    calculateFloatingParts(mesh);


    // todo move up cradle if no line contacts model
    // todo generate additional overhang if model may bend... Is this a TreeSupport Feature though?

    std::vector<std::vector<std::vector<Polygons>>> shadow_data = generateCradleCenters(mesh);
    generateCradleLines(shadow_data);
    cleanCradleLineOverlaps();
    generateCradleLineAreasAndBase(shadow_data,support_free_areas);
}

void TreeSupportTipGenerator::dropOverhangAreas(const SliceMeshStorage& mesh, std::vector<Polygons>& result, bool roof)
{
    std::mutex critical;

    // this is ugly, but as far as i can see there is not way to ensure a parallel_for loop is calculated for each iteration up to a certain point before it continues;
    cura::parallel_for<coord_t>(
        1,
        mesh.overhang_areas.size() - z_distance_delta_,
        [&](const LayerIndex layer_idx)
        {
            if (mesh.overhang_areas[layer_idx + z_distance_delta_].empty() || result.size() < layer_idx)
            {
                return; // This is a continue if imagined in a loop context.
            }

            Polygons relevant_forbidden = volumes_.getCollision(roof ? 0 : config_.getRadius(0), layer_idx, ! xy_overrides_);
            // ^^^ Take the least restrictive avoidance possible

            // Technically this also makes support blocker smaller, which is wrong as they do not have a xy_distance, but it should be good enough.
            Polygons model_outline = volumes_.getCollision(0, layer_idx, ! xy_overrides_).offset(-config_.xy_min_distance, ClipperLib::jtRound);

            //Use full_overhang to ensure dropped overhang will overlap with overhang further down. not leaving a small hole between model and roof where support could creep into.
                Polygons overhang_full = TreeSupportUtils::safeOffsetInc(
                mesh.full_overhang_areas[layer_idx + z_distance_delta_],
                roof ? roof_outset_ : support_outset_,
                relevant_forbidden,
                config_.min_radius * 1.75 + config_.xy_min_distance,
                0,
                1,
                config_.support_line_distance / 2,
                &config_.simplifier);
            Polygons remaining_overhang = mesh.full_overhang_areas[layer_idx + z_distance_delta_]
                                              .offset(roof ? roof_outset_ : support_outset_)
                                              .difference(overhang_full)
                                              .intersection(relevant_forbidden)
                                              .difference(model_outline);
            for (size_t lag_ctr = 1; lag_ctr <= max_overhang_insert_lag_ && layer_idx - coord_t(lag_ctr) >= 1 && ! remaining_overhang.empty(); lag_ctr++)
            {
                {
                    std::lock_guard<std::mutex> critical_section_storage(critical);
                    result[layer_idx - lag_ctr].add(remaining_overhang);
                }

                Polygons relevant_forbidden_below = volumes_.getCollision(roof ? 0 : config_.getRadius(0), layer_idx - lag_ctr, ! xy_overrides_).offset(EPSILON);
                remaining_overhang = remaining_overhang.intersection(relevant_forbidden_below).unionPolygons().difference(model_outline);
            }
        });

    cura::parallel_for<coord_t>(

            0,
            result.size(),
            [&](const LayerIndex layer_idx)
            {
                result[layer_idx]=result[layer_idx].unionPolygons();
            }
        );
}

void TreeSupportTipGenerator::calculateRoofAreas(const cura::SliceMeshStorage& mesh)
{
    std::vector<Polygons> potential_support_roofs(mesh.overhang_areas.size(), Polygons());
    std::mutex critical_potential_support_roofs;
    std::vector<Polygons> dropped_overhangs(mesh.overhang_areas.size(), Polygons());

    if (xy_overrides_)
    {
        dropOverhangAreas(mesh, dropped_overhangs, true);
    }

    cura::parallel_for<coord_t>(
        0,
        mesh.overhang_areas.size() - z_distance_delta_,
        [&](const LayerIndex layer_idx)
        {
            if (mesh.overhang_areas[layer_idx + z_distance_delta_].empty())
            {
                return; // This is a continue if imagined in a loop context.
            }

            // Roof does not have a radius, so remove it using offset. Note that there is no 0 radius avoidance, and it would not be identical with the avoidance offset with
            // -radius. This is intentional here, as support roof is still valid if only a part of the tip may reach it.
            Polygons forbidden_here = volumes_
                                          .getAvoidance(
                                              0,
                                              layer_idx,
                                              (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                              config_.support_rests_on_model,
                                              ! xy_overrides_)
                                          ;

            // todo Since arachnea the assumption that an area smaller then line_width is not printed is no longer true all such safeOffset should have config_.support_line_width
            // replaced with another setting. It should still work in most cases, but it should be possible to create a situation where a overhang outset lags though a wall. I will
            // take a look at this later.
            Polygons full_overhang_area = TreeSupportUtils::safeOffsetInc(
                mesh.full_overhang_areas[layer_idx + z_distance_delta_].unionPolygons(dropped_overhangs[layer_idx]),
                roof_outset_,
                forbidden_here,
                config_.support_line_width,
                0,
                1,
                config_.support_line_distance / 2,
                &config_.simplifier);

            for (LayerIndex dtt_roof = 0; dtt_roof < support_roof_layers_ && layer_idx - dtt_roof >= 1; dtt_roof++)
            {
                const Polygons forbidden_next = volumes_
                                                    .getAvoidance(
                                                        0,
                                                        layer_idx - (dtt_roof + 1),
                                                        (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                                        config_.support_rests_on_model,
                                                        ! xy_overrides_)
                                                    ;

                full_overhang_area = full_overhang_area.difference(forbidden_next);


                if(force_minimum_roof_area_)
                {
                    full_overhang_area.removeSmallAreas(minimum_roof_area_);
                }

                if (full_overhang_area.area() > EPSILON)
                {
                    std::lock_guard<std::mutex> critical_section_potential_support_roofs(critical_potential_support_roofs);
                    potential_support_roofs[layer_idx - dtt_roof].add((full_overhang_area));
                    if (dtt_roof == 0)
                    {
                        support_roof_drawn_fractional_[layer_idx].add(full_overhang_area);
                    }
                }
                else
                {
                    break;
                }
            }
        });

    cura::parallel_for<coord_t>(
        0,
        potential_support_roofs.size(),
        [&](const LayerIndex layer_idx)
        {
            // Now, because the avoidance/collision was subtracted above, the overhang parts that are of xy distance were removed, so to merge areas that should have been one
            // offset by xy_min_distance and then undo it. In a perfect world the offset here would be of a mode that makes sure that
            // area.offset(config_.xy_min_distance).unionPolygons().offset(-config_.xy_min_distance) = area if there is only one polygon in said area. I have not encountered issues
            // with using the default mitered here. Could be that i just have not encountered an issue with it yet though.

            potential_support_roofs[layer_idx] = potential_support_roofs[layer_idx].unionPolygons().offset(config_.xy_min_distance).unionPolygons().offset(-config_.xy_min_distance).unionPolygons(potential_support_roofs[layer_idx]);
        }
    );

    std::vector<Polygons> additional_support_roofs(mesh.overhang_areas.size(), Polygons());

    cura::parallel_for<coord_t>(
        0,
        potential_support_roofs.size(),
        [&](const LayerIndex layer_idx)
        {
            if (! potential_support_roofs[layer_idx].empty())
            {
                // Roof does not have a radius, so remove it using offset. Note that there is no 0 radius avoidance, and it would not be identical with the avoidance offset with
                // -radius. This is intentional here, as support roof is still valid if only a part of the tip may reach it.
                Polygons forbidden_here = volumes_
                                              .getAvoidance(
                                                  0,
                                                  layer_idx,
                                                  (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                                  config_.support_rests_on_model,
                                                  ! xy_overrides_)
                                              ;

                if (! force_minimum_roof_area_)
                {
                    Polygons fuzzy_area = Polygons();

                    // the roof will be combined with roof above and below, to see if a part of this roof may be part of a valid roof further up/down.
                    // This prevents the situation where a roof gets removed even tough its area would contribute to a (better) printable roof area further down.
                    for (const LayerIndex layer_offset : ranges::views::iota(
                             -LayerIndex{ std::min(layer_idx, LayerIndex{ support_roof_layers_ }) },
                             LayerIndex{ std::min(LayerIndex{ potential_support_roofs.size() - layer_idx }, LayerIndex{ support_roof_layers_ + 1 }) }))
                    {
                        fuzzy_area.add(support_roof_drawn_[layer_idx + layer_offset]);
                        fuzzy_area.add(potential_support_roofs[layer_idx + layer_offset]);
                    }
                    fuzzy_area = fuzzy_area.unionPolygons();
                    fuzzy_area.removeSmallAreas(minimum_roof_area_);

                    for (Polygons potential_roof : potential_support_roofs[layer_idx].difference(forbidden_here).splitIntoParts())
                    {
                        if (! potential_roof.intersection(fuzzy_area).empty())
                        {
                            additional_support_roofs[layer_idx].add(potential_roof);
                        }
                    }
                }
                else
                {
                    Polygons valid_roof = potential_support_roofs[layer_idx].difference(forbidden_here);
                    valid_roof.removeSmallAreas(minimum_roof_area_);
                    additional_support_roofs[layer_idx].add(valid_roof);
                }
            }
        });

    cura::parallel_for<coord_t>(

            0,
            additional_support_roofs.size(),
            [&](const LayerIndex layer_idx)
            {
                support_roof_drawn_[layer_idx] = support_roof_drawn_[layer_idx].unionPolygons(additional_support_roofs[layer_idx]);
            }
        );
}


TreeSupportElement* TreeSupportTipGenerator::addPointAsInfluenceArea(
    std::vector<std::set<TreeSupportElement*>>& move_bounds,
    std::pair<Point2LL, TreeSupportTipGenerator::LineStatus> p,
    size_t dtt,
    double hidden_radius_increase,
    LayerIndex insert_layer,
    size_t dont_move_until,
    TipRoofType roof,
    bool cradle,
    bool skip_ovalisation,
    std::vector<Point2LL> additional_ovalization_targets)
{
    const bool to_bp = p.second == LineStatus::TO_BP || p.second == LineStatus::TO_BP_SAFE;
    const bool gracious = to_bp || p.second == LineStatus::TO_MODEL_GRACIOUS || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
    const bool safe_radius = p.second == LineStatus::TO_BP_SAFE || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
    if (! config_.support_rests_on_model && ! to_bp)
    {
        spdlog::warn("Tried to add an invalid support point");
        return nullptr;
    }
    Polygon circle;
    Polygon base_circle = TreeSupportBaseCircle::getBaseCircle();
    for (Point2LL corner : base_circle)
    {
        circle.add(p.first + corner);
    }
    Polygons area = circle.offset(0);
    {
        std::lock_guard<std::mutex> critical_section_movebounds(critical_move_bounds_);
        if (! already_inserted_[insert_layer].count(p.first / ((config_.min_radius + 1) / 10)))
        {
            // Normalize the point a bit to also catch points which are so close that inserting it would achieve nothing.
            already_inserted_[insert_layer].emplace(p.first / ((config_.min_radius + 1) / 10));
            TreeSupportElement* elem = new TreeSupportElement(
                dtt,
                insert_layer,
                p.first,
                to_bp,
                gracious,
                ! xy_overrides_,
                dont_move_until + dtt,
                roof == TipRoofType::SUPPORTS_ROOF,
                cradle,
                safe_radius,
                roof == TipRoofType::IS_ROOF && ! use_fake_roof_,
                skip_ovalisation,
                support_tree_limit_branch_reach_,
                support_tree_branch_reach_limit_,
                hidden_radius_increase);
            elem->area_ = new Polygons(area);
            for (Point2LL target : additional_ovalization_targets)
            {
                elem->additional_ovalization_targets_.emplace_back(target);
            }

            move_bounds[insert_layer].emplace(elem);
            return elem;
        }
    }
    return nullptr;
}


void TreeSupportTipGenerator::addLinesAsInfluenceAreas(std::vector<std::set<TreeSupportElement *>>& move_bounds,
                                                                                   std::vector<TreeSupportTipGenerator::LineInformation> lines,
                                                                                   size_t roof_tip_layers,
                                                                                   LayerIndex insert_layer_idx,
                                                                                   coord_t tip_radius,
                                                                                   OverhangInformation& overhang_data,
                                                                                   size_t dont_move_until,
                                                                                   bool connect_points)
{
    for (LineInformation line : lines)
    {
        // If a line consists of enough tips, the assumption is that it is not a single tip, but part of a simulated support pattern.
        // Ovalisation should be disabled if they may be placed close to each other to prevent tip-areas merging. If the tips has to turn into roof, the area is most likely not
        // large enough for this to cause issues. todo does this still make sense?
        const bool disable_ovalization = ! connect_points && config_.min_radius < 3 * config_.support_line_width && roof_tip_layers == 0;
        for (auto [idx, point_data] : line | ranges::views::enumerate)
        {
            std::vector<Point2LL> additional_ovalization_targets;
            if (connect_points) // If the radius is to large then the ovalization would cause the area to float in the air.
            {
                if (idx != 0)
                {
                    additional_ovalization_targets.emplace_back(line[idx - 1].first);
                }
                if (idx != line.size() - 1)
                {
                    additional_ovalization_targets.emplace_back(line[idx + 1].first);
                }
            }

            if(overhang_data.is_cradle_)
            {
                if((overhang_data.isCradleLine() && large_cradle_line_tips_) || (!overhang_data.isCradleLine() && (! cradle_base_roof_ || large_cradle_line_tips_)))
                {
                    tip_radius =  cradle_tip_dtt_;
                }
            }

            size_t tip_dtt = tip_radius >= config_.branch_radius ? config_.tip_layers :
                                                                 (config_.tip_layers * (tip_radius - config_.min_radius))/(config_.branch_radius - config_.min_radius);

            coord_t hidden_radius = tip_radius > config_.branch_radius ? tip_radius - config_.branch_radius : 0;
            double hidden_radius_increases = hidden_radius  / (config_.branch_radius * (std::max(config_.diameter_scale_bp_radius - config_.diameter_angle_scale_factor, 0.0)));

            TreeSupportElement* elem = addPointAsInfluenceArea(
                move_bounds,
                point_data,
                tip_dtt,
                hidden_radius_increases,
                insert_layer_idx,
                dont_move_until,
                (! overhang_data.is_cradle_ && roof_tip_layers > 0) ? TipRoofType::IS_ROOF : (overhang_data.is_roof_ ? TipRoofType::SUPPORTS_ROOF : TipRoofType::NONE),
                overhang_data.is_cradle_,
                disable_ovalization,
                additional_ovalization_targets);

            if(elem)
            {
                if(overhang_data.isCradleLine())
                {
                    elem->cradle_line_ = std::make_shared<CradlePresenceInformation>(overhang_data.cradle_, overhang_data.cradle_layer_idx_, overhang_data.cradle_line_idx_);
                }
            }
        }
    }
}


void TreeSupportTipGenerator::removeUselessAddedPoints(
    std::vector<std::set<TreeSupportElement*>>& move_bounds,
    SliceDataStorage& storage)
{
    std::vector<Polygons> all_cradle_roofs(storage.support.supportLayers.size());
    for (auto [layer_idx, cradles] : cradle_data_ | ranges::views::enumerate)
    {
        for(auto cradle:cradles)
        {
            if(cradle->is_roof_)
            {

                for (auto [base_idx, base] : cradle->base_below_ | ranges::views::enumerate)
                {
                    all_cradle_roofs[layer_idx-base_idx].add(base);
                }
            }
        }
    }

    cura::parallel_for<coord_t>(
        0,
        move_bounds.size(),
        [&](const LayerIndex layer_idx)
        {
            if (layer_idx + 1 < storage.support.supportLayers.size())
            {
                std::vector<TreeSupportElement*> to_be_removed;
                Polygons roof_on_layer_above = support_roof_drawn_[layer_idx + 1];
                roof_on_layer_above = roof_on_layer_above.unionPolygons(all_cradle_roofs[layer_idx+1]);
                Polygons roof_on_layer = support_roof_drawn_[layer_idx];

                for (TreeSupportElement* elem : move_bounds[layer_idx])
                {
                    if (roof_on_layer.inside(elem->result_on_layer_)) // Remove branches that start inside of support interface
                    {
                        to_be_removed.emplace_back(elem);
                    }
                    else if (elem->supports_roof_ && elem->cradle_line_.get() == nullptr)
                    {
                        Point2LL from = elem->result_on_layer_;
                        PolygonUtils::moveInside(roof_on_layer_above, from);
                        // Remove branches should have interface above them, but don't. Should never happen.
                        if (roof_on_layer_above.empty()
                            || (!roof_on_layer_above.inside(elem->result_on_layer_) && vSize2(from-elem->result_on_layer_) > std::pow(config_.getRadius(0) + FUDGE_LENGTH,2)))
                        {
                            to_be_removed.emplace_back(elem);
                            spdlog::warn("Removing already placed tip that should have roof above it. Distance from roof is {}.",vSize(from-elem->result_on_layer_));
                        }
                    }
                }

                for (auto elem : to_be_removed)
                {
                    move_bounds[layer_idx].erase(elem);
                    delete elem->area_;
                    delete elem;
                }
            }
        });
}


void TreeSupportTipGenerator::generateTips(
    SliceDataStorage& storage,
    const SliceMeshStorage& mesh,
    std::vector<std::set<TreeSupportElement*>>& move_bounds,
    std::vector<std::vector<FakeRoofArea>>& placed_fake_roof_areas,
    std::vector<Polygons>& support_free_areas,
    std::vector<std::vector<TreeSupportCradle*>>& cradle_data_export)
{
    std::vector<std::set<TreeSupportElement*>> new_tips(move_bounds.size());

    const coord_t circle_length_to_half_linewidth_change
        = config_.min_radius < config_.support_line_width ? config_.min_radius / 2 : sqrt(square(config_.min_radius) - square(config_.min_radius - config_.support_line_width / 2));
    // ^^^ As r*r=x*x+y*y (circle equation): If a circle with center at (0,0) the top most point is at (0,r) as in y=r. This calculates how far one has to move on the x-axis so
    // that y=r-support_line_width/2.
    //     In other words how far does one need to move on the x-axis to be support_line_width/2 away from the circle line. As a circle is round this length is identical for every
    //     axis as long as the 90� angle between both remains.

    const coord_t extra_outset = std::max(coord_t(0), config_.min_radius - config_.support_line_width / 2) + (xy_overrides_ ? 0 : config_.support_line_width / 2);
    // ^^^ Extra support offset to compensate for larger tip radiis. Also outset a bit more when z overwrites xy, because supporting something with a part of a support line is
    // better than not supporting it at all.

    if(cradle_layers_ >0)
    {
        generateCradle(mesh,support_free_areas);
    }

    if (support_roof_layers_)
    {
        calculateRoofAreas(mesh);
    }

    std::vector<std::vector<TreeSupportCradle*>> all_cradles_requiring_support(move_bounds.size());
    std::vector<Polygons> all_cradle_areas(move_bounds.size());
    for(LayerIndex layer_idx = 0; layer_idx < cradle_data_.size(); layer_idx++)
    {
        for(size_t cradle_idx = 0; cradle_idx < cradle_data_[layer_idx].size(); cradle_idx++)
        {
            for(auto overhang_pair: cradle_data_[layer_idx][cradle_idx]->overhang_)
            {
                all_cradles_requiring_support[overhang_pair.first].emplace_back(cradle_data_[layer_idx][cradle_idx]);
            }

            for (auto [line_idx, lines] : cradle_data_[layer_idx][cradle_idx]->lines_ | ranges::views::enumerate)
            {
                for (auto [height_idx, line] : lines | ranges::views::enumerate)
                {
                    all_cradle_areas[line.layer_idx_].add(line.area_);
                }
            }
            for (auto [base_idx, base] : cradle_data_[layer_idx][cradle_idx]->base_below_ | ranges::views::enumerate)
            {
                all_cradle_areas[layer_idx-base_idx].add(base);
            }
        }
    }

    cura::parallel_for<coord_t>(
        1,
        mesh.overhang_areas.size() - z_distance_delta_,
        [&](const LayerIndex layer_idx)
        {
            if (mesh.overhang_areas[layer_idx + z_distance_delta_].empty() &&
                    (layer_idx + 1 >= support_roof_drawn_.size() || support_roof_drawn_[layer_idx + 1].empty()) &&
                    (layer_idx >= all_cradles_requiring_support.size() || all_cradles_requiring_support[layer_idx].empty())
                )
            {
                return; // This is a continue if imagined in a loop context.
            }

            coord_t current_tip_radius = (force_initial_layer_radius_ && config_.recommendedMinRadius(layer_idx) > config_.min_radius) ? config_.recommendedMinRadius(layer_idx) : config_.min_radius;
            coord_t connect_length = (config_.support_line_width * 100 / support_tree_top_rate_) + std::max(2 * current_tip_radius - 1.0 * config_.support_line_width, 0.0);
            coord_t support_tree_branch_distance = (config_.support_pattern == EFillMethod::TRIANGLES ? 3 : (config_.support_pattern == EFillMethod::GRID ? 2 : 1)) * connect_length;
            bool force_tip_to_roof = (current_tip_radius * current_tip_radius * std::numbers::pi > minimum_roof_area_ * (1000 * 1000)) && !use_fake_roof_ && support_roof_layers_;

            Polygons relevant_forbidden = volumes_.getAvoidance(
                config_.getRadius(0),
                layer_idx,
                (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                config_.support_rests_on_model,
                ! xy_overrides_);
            // ^^^ Take the least restrictive avoidance possible
            relevant_forbidden
                = relevant_forbidden.offset(EPSILON)
                      .unionPolygons(); // Prevent rounding errors down the line, points placed directly on the line of the forbidden area may not be added otherwise.



            std::function<Polygons(const Polygons&, bool, LayerIndex)> generateLines = [&](const Polygons& area, bool roof, LayerIndex generate_layer_idx)
            {
                //todo ensure larger tips have reasonable density. How would one do that though?
                // If tips are 7mm thick, does 20% fill mean a distance of 35mm between tips? Does not make sense...
                coord_t upper_line_distance = support_supporting_branch_distance_;
                coord_t line_distance = std::max(roof ? support_roof_line_distance_ : support_tree_branch_distance, upper_line_distance);
                coord_t current_tip_radius_generate = (force_initial_layer_radius_ && config_.recommendedMinRadius(generate_layer_idx) > config_.min_radius) ? config_.recommendedMinRadius(generate_layer_idx) : config_.min_radius;

                bool use_grid = line_distance == upper_line_distance;

                return TreeSupportUtils::generateSupportInfillLines(
                    area,
                    config_,
                    roof && ! use_fake_roof_,
                    generate_layer_idx,
                    line_distance,
                    roof? nullptr : getCrossFillProvider(mesh, line_distance, current_tip_radius_generate),
                    (roof && ! use_fake_roof_) ? config_.support_roof_wall_count : 0,
                    use_grid ? EFillMethod::GRID:EFillMethod::NONE);
            };


            std::vector<OverhangInformation> overhang_processing;
            // ^^^ Every overhang has saved if a roof should be generated for it.
            //     This can NOT be done in the for loop as an area may NOT have a roof even if it is larger than the minimum_roof_area when it is only larger because of the support
            //     horizontal expansion and it would not have a roof if the overhang is offset by support roof horizontal expansion instead. (At least this is the current behavior
            //     of the regular support)

            Polygons core_overhang = mesh.overhang_areas[layer_idx + z_distance_delta_];


            if (support_roof_layers_ && layer_idx + 1 < support_roof_drawn_.size())
            {
                core_overhang = core_overhang.difference(support_roof_drawn_[layer_idx].offset(config_.support_roof_line_width / 2).unionPolygons());
                for (Polygons roof_part : support_roof_drawn_[layer_idx + 1]
                                              .difference(support_roof_drawn_[layer_idx].offset(config_.maximum_move_distance_slow).unionPolygons())
                                              .splitIntoParts(true)) // If there is a roof, the roof will be one layer above the tips.
                {
                    //^^^Technically one should also subtract the avoidance of radius 0 (similarly how calculated in calculateRoofArea), as there can be some rounding errors
                    // introduced since then. But this does not fully prevent some rounding errors either way, so just handle the error later.
                    overhang_processing.emplace_back(roof_part, true);
                }
            }
            all_cradle_areas[layer_idx] = all_cradle_areas[layer_idx].unionPolygons().offset(EPSILON).unionPolygons();
            core_overhang = core_overhang.difference(support_free_areas[layer_idx]);
            core_overhang = core_overhang.difference(all_cradle_areas[layer_idx]);

            for(size_t cradle_idx = 0; cradle_idx < all_cradles_requiring_support[layer_idx].size(); cradle_idx++)
            {
                for(size_t cradle_overhang_idx = 0; cradle_overhang_idx < all_cradles_requiring_support[layer_idx][cradle_idx]->overhang_[layer_idx].size(); cradle_overhang_idx++)
                {
                    overhang_processing.emplace_back(all_cradles_requiring_support[layer_idx][cradle_idx]->overhang_[layer_idx][cradle_overhang_idx]);
                    core_overhang = core_overhang.difference(all_cradles_requiring_support[layer_idx][cradle_idx]->overhang_[layer_idx][cradle_overhang_idx].overhang_);
                }
            }

            Polygons overhang_regular = TreeSupportUtils::safeOffsetInc(
                core_overhang,
                support_outset_,
                relevant_forbidden,
                config_.min_radius * 1.75 + config_.xy_min_distance,
                0,
                1,
                config_.support_line_distance / 2,
                &config_.simplifier);
            Polygons remaining_overhang
                = core_overhang.offset(support_outset_).difference(overhang_regular.offset(config_.support_line_width * 0.5)).intersection(relevant_forbidden);


            // Offset ensures that areas that could be supported by a part of a support line, are not considered unsupported overhang
            coord_t extra_total_offset_acc = 0;

                // Offset the area to compensate for large tip radiis. Offset happens in multiple steps to ensure the tip is as close to the original overhang as possible.

                 {
                    Polygons already_supported = support_roof_drawn_[layer_idx];
                    already_supported.add(all_cradle_areas[layer_idx]);
                    already_supported.add(support_free_areas[layer_idx]); // While point there are not supported, there may be no support anyway.
                    already_supported = already_supported.unionPolygons();
                    while (extra_total_offset_acc + config_.support_line_width / 8 < extra_outset) //+mesh_config_.support_line_width / 8  to avoid calculating very small (useless) offsets because of rounding errors.
                    {
                        coord_t offset_current_step =
                            extra_total_offset_acc + 2 * config_.support_line_width > config_.min_radius
                                                                                                                ? std::min(config_.support_line_width / 8, extra_outset - extra_total_offset_acc)
                                                                                                                : std::min(circle_length_to_half_linewidth_change, extra_outset - extra_total_offset_acc);
                        extra_total_offset_acc += offset_current_step;
                        Polygons overhang_offset = TreeSupportUtils::safeOffsetInc(overhang_regular, 1.5 * extra_total_offset_acc, volumes_.getCollision(0, layer_idx, true), config_.xy_min_distance + config_.support_line_width, 0, 1, config_.support_line_distance / 2, &config_.simplifier);
                        remaining_overhang = remaining_overhang.difference(overhang_offset.unionPolygons(already_supported.offset(1.5 * extra_total_offset_acc))).unionPolygons(); //overhang_offset is combined with roof, as all area that has a roof, is already supported by said roof.
                        Polygons next_overhang = TreeSupportUtils::safeOffsetInc(remaining_overhang, extra_total_offset_acc, volumes_.getCollision(0, layer_idx, true), config_.xy_min_distance + config_.support_line_width, 0, 1, config_.support_line_distance / 2, &config_.simplifier);
                        overhang_regular = overhang_regular.unionPolygons(next_overhang.difference(relevant_forbidden));
                    }
                }

            // If the xy distance overrides the z distance, some support needs to be inserted further down.
            //=> Analyze which support points do not fit on this layer and check if they will fit a few layers down
            //   (while adding them an infinite amount of layers down would technically be closer the setting description, it would not produce reasonable results. )
            if (xy_overrides_)
            {
                for (Polygons& remaining_overhang_part : remaining_overhang.splitIntoParts(false))
                {
                    if (remaining_overhang_part.area() <= MM2_2INT(minimum_support_area_))
                    {
                        continue;
                    }
                    all_cradle_areas[layer_idx] = all_cradle_areas[layer_idx].unionPolygons();
                    std::vector<LineInformation> overhang_lines;
                    Polygons polylines = ensureMaximumDistancePolyline(generateLines(remaining_overhang_part, false, layer_idx), current_tip_radius, 1, false);
                    // ^^^ Support_line_width to form a line here as otherwise most will be unsupported.
                    // Technically this violates branch distance, but not only is this the only reasonable choice,
                    //   but it ensures consistent behavior as some infill patterns generate each line segment as its own polyline part causing a similar line forming behavior.
                    // Also it is assumed that the area that is valid a layer below is too small for support roof.
                    if (polylines.pointCount() <= 3)
                    {
                        // Add the outer wall to ensure it is correct supported instead.
                        polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(remaining_overhang_part), connect_length, 3, true);
                    }

                    for (auto line : polylines)
                    {
                        LineInformation res_line;
                        for (Point2LL p : line)
                        {
                            res_line.emplace_back(p, LineStatus::INVALID);
                        }
                        overhang_lines.emplace_back(res_line);
                    }

                    for (size_t lag_ctr = 1; lag_ctr <= max_overhang_insert_lag_ && ! overhang_lines.empty() && layer_idx - coord_t(lag_ctr) >= 1; lag_ctr++)
                    {
                        LayerIndex layer_idx_lag = layer_idx - lag_ctr;
                        coord_t current_tip_radius_lag = std::max(config_.min_radius, config_.recommendedMinRadius(layer_idx_lag)); //todo setting

                        // get least restricted avoidance for layer_idx-lag_ctr
                        Polygons relevant_forbidden_below = volumes_.getAvoidance(
                            config_.getRadius(0),
                            layer_idx - lag_ctr,
                            (only_gracious_ || ! config_.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                            config_.support_rests_on_model,
                            ! xy_overrides_);
                        // It is not required to offset the forbidden area here as the points won't change:
                        // If points here are not inside the forbidden area neither will they be later when placing these points, as these are the same points.
                        std::function<bool(std::pair<Point2LL, LineStatus>)> evaluatePoint = [&](std::pair<Point2LL, LineStatus> p)
                        {
                            return relevant_forbidden_below.inside(p.first, true);
                        };

                        Polygons already_supported = support_roof_drawn_[layer_idx-lag_ctr];
                        already_supported.add(support_free_areas[layer_idx-lag_ctr]); // While point there are not supported, there may be no support anyway.
                        already_supported=already_supported.unionPolygons();

                        //Remove all points that are for some reason are already supported
                        std::function<bool(std::pair<Point2LL, LineStatus>)> evaluateAlreadySupported =
                            [&](std::pair<Point2LL, LineStatus> p) { return already_supported.inside(p.first, true);
                        };

                        overhang_lines = splitLines(overhang_lines, evaluateAlreadySupported).second;

                        std::pair<std::vector<TreeSupportTipGenerator::LineInformation>, std::vector<TreeSupportTipGenerator::LineInformation>> split
                            = splitLines(overhang_lines, evaluatePoint); // Keep all lines that are invalid.
                        overhang_lines = split.first;
                        std::vector<LineInformation> fresh_valid_points = convertLinesToInternal(convertInternalToLines(split.second), layer_idx - lag_ctr);
                        // ^^^ Set all now valid lines to their correct LineStatus. Easiest way is to just discard Avoidance information for each point and evaluate them again.

                        OverhangInformation dummy_overhang_info(remaining_overhang_part,false);
                        addLinesAsInfluenceAreas(new_tips,fresh_valid_points, (force_tip_to_roof && lag_ctr <= support_roof_layers_) ? support_roof_layers_ : 0,
                                                 layer_idx - lag_ctr, current_tip_radius_lag, dummy_overhang_info, support_roof_layers_, false);
                    }
                }
            }

            overhang_regular.removeSmallAreas(minimum_support_area_);

            for (Polygons support_part : overhang_regular.splitIntoParts(true))
            {
                overhang_processing.emplace_back(support_part, false);
            }

            for (OverhangInformation overhang_data : overhang_processing)
            {
                Polygons overhang_outset = overhang_data.overhang_;
                const size_t min_support_points = std::max(coord_t(2), std::min(coord_t(EPSILON), overhang_outset.polygonLength() / connect_length));
                std::vector<LineInformation> overhang_lines;

                bool only_lines = true;
                Polygons polylines;
                // The tip positions are determined here.
                if(overhang_data.isCradleLine())
                {
                    std::optional<TreeSupportCradleLine *> cradle_line_opt = overhang_data.cradle_->getCradleLineOfIndex(overhang_data.cradle_layer_idx_,overhang_data.cradle_line_idx_);
                    Polygons line = cradle_line_opt.value()->line_.offset(0);
                    polylines = ensureMaximumDistancePolyline(
                        line,
                        ! overhang_data.is_roof_ ? config_.min_radius * 2 : support_supporting_branch_distance_,
                        1,
                        false);
                }
                else
                {
                    // todo can cause inconsistent support density if a line exactly aligns with the model
                    polylines = ensureMaximumDistancePolyline(
                            generateLines(overhang_outset, overhang_data.is_roof_, layer_idx + overhang_data.is_roof_),
                        ! overhang_data.is_roof_ ? config_.min_radius * 2
                        : use_fake_roof_             ? support_supporting_branch_distance_
                                                     : connect_length,
                        1,
                        false);
                }


                // support_line_width to form a line here as otherwise most will be unsupported.
                // Technically this violates branch distance, but not only is this the only reasonable choice,
                //   but it ensures consistent behaviour as some infill patterns generate each line segment as its own polyline part causing a similar line forming behaviour.
                // This is not done when a roof is above as the roof will support the model and the trees only need to support the roof

                if (polylines.pointCount() <= min_support_points)
                {
                    only_lines = false;
                    // Add the outer wall (of the overhang) to ensure it is correct supported instead.
                    // Try placing the support points in a way that they fully support the outer wall, instead of just the with half of the support line width.
                    Polygons reduced_overhang_outset = overhang_outset.offset(-config_.support_line_width / 2.2);
                    // ^^^ It's assumed that even small overhangs are over one line width wide, so lets try to place the support points in a way that the full support area
                    // generated from them will support the overhang.
                    //     (If this is not done it may only be half). This WILL NOT be the case when supporting an angle of about < 60� so there is a fallback, as some support is
                    //     better than none.)
                    if (! reduced_overhang_outset.empty()
                        && overhang_outset.difference(reduced_overhang_outset.offset(std::max(config_.support_line_width, connect_length))).area() < 1)
                    {
                        polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(reduced_overhang_outset), connect_length, min_support_points, true);
                    }
                    else
                    {
                        polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(overhang_outset), connect_length, min_support_points, true);
                    }
                }

                if (overhang_data.is_roof_ || overhang_data.is_cradle_) // Some roof may only be supported by a part of a tip
                {
                    polylines = TreeSupportUtils::movePointsOutside(polylines, relevant_forbidden, (overhang_data.is_cradle_?2:1) * config_.getRadius(0) + FUDGE_LENGTH / 2);
                }

                overhang_lines = convertLinesToInternal(polylines, layer_idx);

                if (overhang_lines.empty()) // some error handling and logging
                {
                    Polygons enlarged_overhang_outset = overhang_outset.offset(config_.getRadius(0) + FUDGE_LENGTH / 2, ClipperLib::jtRound).difference(relevant_forbidden);
                    polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(enlarged_overhang_outset), connect_length, min_support_points, true);
                    overhang_lines = convertLinesToInternal(polylines, layer_idx);

                    if (! overhang_lines.empty())
                    {
                        spdlog::debug("Compensated for overhang area that had no valid tips. Now has a tip.");
                    }
                    else
                    {
                        spdlog::warn("Overhang area has no valid tips! Was roof: {} Was Cradle {} Was Line {} On Layer: {} Area size was {}",
                                     overhang_data.is_roof_, overhang_data.is_cradle_,overhang_data.isCradleLine() ,layer_idx, overhang_data.overhang_.area());
                    }
                }

                size_t dont_move_for_layers = support_roof_layers_ ? (force_tip_to_roof ? support_roof_layers_ : (overhang_data.is_roof_ ? 0 : support_roof_layers_)) : 0;
                addLinesAsInfluenceAreas(
                    new_tips,
                    overhang_lines,
                    (!overhang_data.is_roof_ && force_tip_to_roof) ? support_roof_layers_ : 0,
                    layer_idx,
                    current_tip_radius,
                    overhang_data,
                    dont_move_for_layers,
                    only_lines);
            }
        });

    cura::parallel_for<coord_t>(
        0,
        support_roof_drawn_.size(),
        [&](const LayerIndex layer_idx)
        {
            if (use_fake_roof_)
            {
                placed_fake_roof_areas[layer_idx].emplace_back(support_roof_drawn_[layer_idx], support_roof_line_distance_, false);

                if (config_.z_distance_top % config_.layer_height != 0 && layer_idx > 0)
                {
                    Polygons all_roof_fractional = support_roof_drawn_fractional_[layer_idx - 1];
                    placed_fake_roof_areas[layer_idx].emplace_back(all_roof_fractional, support_roof_line_distance_, true);
                }
            }
            else
            {
                if (config_.z_distance_top % config_.layer_height != 0 && layer_idx > 0)
                {
                    Polygons all_roof_below = support_roof_drawn_[layer_idx - 1];
                    Polygons all_roof_fractional = support_roof_drawn_fractional_[layer_idx - 1].intersection(all_roof_below).difference(support_roof_drawn_[layer_idx]);

                    if (config_.support_wall_count > 0 && config_.support_roof_wall_count == 0) // Need to check every area whether it has lines
                    {
                        for (auto roof_area : all_roof_fractional.splitIntoParts())
                        {
                            // technically there is no guarantee that a drawn roof tip has lines. If it does not add walls
                            size_t roof_wall_count = 0;
                            if (TreeSupportUtils::generateSupportInfillLines(roof_area, config_, true, layer_idx, support_roof_line_distance_, nullptr, 0).empty())
                            {
                                roof_wall_count = config_.support_wall_count;
                            }
                            storage.support.supportLayers[layer_idx].fillRoofParts(roof_area, config_.support_roof_line_width, roof_wall_count, false);
                        }
                    }
                    else
                    {
                        storage.support.supportLayers[layer_idx].fillRoofParts(all_roof_fractional, config_.support_roof_line_width, config_.support_roof_wall_count, true);
                    }
                }

                if (config_.support_wall_count > 0 && config_.support_roof_wall_count == 0)
                {
                    for (auto roof_area : support_roof_drawn_[layer_idx].splitIntoParts()) // Need to check every area whether it has lines
                    {
                        // technically there is no guarantee that a drawn roof tip has lines. If it does not add walls
                        size_t roof_wall_count = 0;
                        if (TreeSupportUtils::generateSupportInfillLines(roof_area, config_, true, layer_idx, support_roof_line_distance_, nullptr, 0).empty())
                        {
                            roof_wall_count = config_.support_wall_count;
                        }
                        storage.support.supportLayers[layer_idx].fillRoofParts(roof_area, config_.support_roof_line_width, roof_wall_count, false);
                    }
                }
                else
                {
                    storage.support.supportLayers[layer_idx].fillRoofParts(support_roof_drawn_[layer_idx], config_.support_roof_line_width, config_.support_roof_wall_count, false);
                }
            }
        });

    cradle_data_export.resize(cradle_data_.size());

    for (auto [layer_idx, cradles] : cradle_data_ | ranges::views::enumerate)
    {
        cradle_data_export[layer_idx] = cradles;
    }

    removeUselessAddedPoints(new_tips, storage);

    for (auto [layer_idx, tips_on_layer] : new_tips | ranges::views::enumerate)
    {
        move_bounds[layer_idx].insert(tips_on_layer.begin(), tips_on_layer.end());
    }

}

} // namespace cura
