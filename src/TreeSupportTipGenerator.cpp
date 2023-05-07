#include "TreeSupportTipGenerator.h"
#include "Application.h" //To get settings.
#include "infill/SierpinskiFillProvider.h"
#include "settings/EnumSettings.h"
#include "utils/algorithm.h"
#include "utils/Simplify.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/polygonUtils.h" //For moveInside.
#include "utils/ThreadPool.h"
#include "TreeSupportUtils.h"

#include <chrono>
#include <fstream>
#include <spdlog/spdlog.h>
#include <stdio.h>
#include <string>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/reverse.hpp>



namespace cura
{

TreeSupportTipGenerator::TreeSupportTipGenerator(const SliceDataStorage& storage, const SliceMeshStorage& mesh, TreeModelVolumes& volumes_s):
    config(mesh.settings),
    use_fake_roof(!mesh.settings.get<bool>("support_roof_enable")),
    minimum_roof_area(!use_fake_roof? mesh.settings.get<double>("minimum_roof_area"): SUPPORT_TREE_MINIMUM_FAKE_ROOF_AREA),
    minimum_support_area(mesh.settings.get<double>("minimum_support_area")),
    support_roof_layers(mesh.settings.get<bool>("support_roof_enable") ? round_divide(mesh.settings.get<coord_t>("support_roof_height"), config.layer_height) : use_fake_roof ? SUPPORT_TREE_MINIMUM_FAKE_ROOF_LAYERS : 0),
    connect_length((config.support_line_width * 100 / mesh.settings.get<double>("support_tree_top_rate")) + std::max(2 * config.min_radius - 1.0 * config.support_line_width, 0.0)),
    support_tree_branch_distance((config.support_pattern == EFillMethod::TRIANGLES ? 3 : (config.support_pattern == EFillMethod::GRID ? 2 : 1)) * connect_length),
    support_roof_line_distance(use_fake_roof ? (config.support_pattern == EFillMethod::TRIANGLES ? 3 : (config.support_pattern == EFillMethod::GRID ? 2 : 1)) * (config.support_line_width * 100 / mesh.settings.get<double>("support_tree_top_rate")) : mesh.settings.get<coord_t>("support_roof_line_distance")),
    support_outset(mesh.settings.get<coord_t>("support_offset")),
    roof_outset(use_fake_roof ? support_outset: mesh.settings.get<coord_t>("support_roof_offset")),
    force_tip_to_roof((config.min_radius * config.min_radius * M_PI > minimum_roof_area * (1000 * 1000)) && support_roof_layers && !use_fake_roof),
    support_tree_limit_branch_reach(mesh.settings.get<bool>("support_tree_limit_branch_reach")),
    support_tree_branch_reach_limit(support_tree_limit_branch_reach ? mesh.settings.get<coord_t>("support_tree_branch_reach_limit") : 0),
    z_distance_delta(std::min(config.z_distance_top_layers+1,mesh.overhang_areas.size())),
    xy_overrides(config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z),
    tip_roof_size(force_tip_to_roof?config.min_radius * config.min_radius * M_PI:0),
    already_inserted(mesh.overhang_areas.size()),
    support_roof_drawn(mesh.overhang_areas.size(),Polygons()),
    roof_tips_drawn(mesh.overhang_areas.size(),Polygons()),
    cradle_areas(mesh.overhang_areas.size(),Polygons()),
    cradle_base_areas(mesh.overhang_areas.size(),Polygons()),
    cradle_overhang(mesh.overhang_areas.size(),Polygons()),
    volumes_(volumes_s),
    force_minimum_roof_area(use_fake_roof || SUPPORT_TREE_MINIMUM_ROOF_AREA_HARD_LIMIT),
    cradle_layers(mesh.settings.get<coord_t>("support_tree_cradle_height") / config.layer_height),
    minimum_cradle_layers(mesh.settings.get<coord_t>("support_tree_cradle_min_height") / config.layer_height),
    cradle_lines(mesh.settings.get<size_t>("support_tree_cradle_line_count")),
    cradle_length(mesh.settings.get<coord_t>("support_tree_cradle_length")),
    cradle_line_width(mesh.settings.get<coord_t>("support_tree_cradle_line_width")),
    cradle_lines_roof(mesh.settings.get<std::string>("support_tree_roof_cradle") != "none"),
    cradle_base_roof(mesh.settings.get<std::string>("support_tree_roof_cradle") == "cradle_and_base" || mesh.settings.get<std::string>("support_tree_roof_cradle") == "large_cradle_and_base"),
    large_cradle_base(mesh.settings.get<std::string>("support_tree_roof_cradle") == "large_cradle_and_base"),
    cradle_area_threshold(1000 * 1000 * mesh.settings.get<double>("support_tree_maximum_pointy_area")),
    cradle_tip_dtt(config.tip_layers *  mesh.settings.get<double>("support_tree_cradle_base_tip_percentage") / 100.0),
    large_cradle_line_tips(mesh.settings.get<bool>("support_tree_large_cradle_line_tips"))
{

    const double support_overhang_angle = mesh.settings.get<AngleRadians>("support_angle");
    const coord_t max_overhang_speed = (support_overhang_angle < TAU / 4) ? (coord_t)(tan(support_overhang_angle) * config.layer_height) : std::numeric_limits<coord_t>::max();
    max_overhang_insert_lag = std::max((size_t)round_up_divide(config.xy_distance, max_overhang_speed / 2), 2 * config.z_distance_top_layers);
    // ^^^ Cap for how much layer below the overhang a new support point may be added, as other than with regular support every new inserted point may cause extra material and time cost.
    //     Could also be an user setting or differently calculated. Idea is that if an overhang does not turn valid in double the amount of layers a slope of support angle would take to travel xy_distance, nothing reasonable will come from it.
    //     The 2*z_distance_delta is only a catch for when the support angle is very high.

    cross_fill_provider = generateCrossFillProvider(mesh, support_tree_branch_distance, config.support_line_width);

    std::vector<coord_t> known_z(mesh.layers.size());

    for (auto [z, layer] : ranges::views::enumerate(mesh.layers))
    {
        known_z[z] = layer.printZ;
    }
    config.setActualZ(known_z);


    coord_t dtt_when_tips_can_merge = 1;

    if(config.branch_radius * config.diameter_angle_scale_factor < 2 * config.maximum_move_distance_slow)
    {
        while((2 * config.maximum_move_distance_slow * dtt_when_tips_can_merge - config.support_line_width)  < config.getRadius(dtt_when_tips_can_merge))
        {
            dtt_when_tips_can_merge++;
        }
    }
    else
    {
        dtt_when_tips_can_merge = config.tip_layers; // arbitrary default for when there is no guarantee that the while loop above will terminate
    }
    support_supporting_branch_distance = 2 * config.getRadius(dtt_when_tips_can_merge) + config.support_line_width + FUDGE_LENGTH;
}


std::vector<TreeSupportTipGenerator::LineInformation> TreeSupportTipGenerator::convertLinesToInternal(Polygons polylines, LayerIndex layer_idx)
{
    // NOTE: The volumes below (on which '.inside(p, true)' is called each time below) are the same each time. The values being calculated here are strictly local as well.
    //       So they could in theory be pre-calculated here (outside of the loop). However, when I refatored it to be that way, it seemed to cause deadlocks each time for some settings.
    // NOTE2: When refactoring ensure that avoidance to buildplate is only requested when support_rest_preference == RestPreference::BUILDPLATE as otherwise it has not been precalculated (causing long delays while it is calculated when requested here).

    std::vector<LineInformation> result;
    // Also checks if the position is valid, if it is NOT, it deletes that point
    for (const auto& line : polylines)
    {
        LineInformation res_line;
        for (const Point& p : line)
        {
            if (config.support_rest_preference == RestPreference::BUILDPLATE && ! volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST_SAFE, false, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_BP_SAFE);
            }
            else if (config.support_rest_preference == RestPreference::BUILDPLATE && ! volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST, false, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_BP);
            }
            else if (config.support_rests_on_model && ! volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST_SAFE, true, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS_SAFE);
            }
            else if (config.support_rests_on_model && ! volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST, true, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS);
            }
            else if (config.support_rests_on_model && ! volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::COLLISION, true, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL);
            }
            else if (! res_line.empty())
            {
                result.emplace_back(res_line);
                res_line.clear();
            }
        }
        if (!res_line.empty())
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

std::function<bool(std::pair<Point, TreeSupportTipGenerator::LineStatus>)> TreeSupportTipGenerator::getEvaluatePointForNextLayerFunction(size_t current_layer)
{
    std::function<bool(std::pair<Point, LineStatus>)> evaluatePoint =
        [=](std::pair<Point, LineStatus> p)
    {
        if (config.support_rest_preference != RestPreference::GRACEFUL && ! volumes_.getAvoidance(config.getRadius(0), current_layer - 1, p.second == LineStatus::TO_BP_SAFE ? AvoidanceType::FAST_SAFE : AvoidanceType::FAST, false, !xy_overrides).inside(p.first, true))
        {
            return true;
        }
        if (config.support_rests_on_model && (p.second != LineStatus::TO_BP && p.second != LineStatus::TO_BP_SAFE))
        {
            if (p.second == LineStatus::TO_MODEL_GRACIOUS || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE)
            {
                return ! volumes_.getAvoidance(config.getRadius(0), current_layer - 1, p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE ? AvoidanceType::FAST_SAFE : AvoidanceType::FAST, true, !xy_overrides).inside(p.first, true);
            }
            else
            {
                return ! volumes_.getAvoidance(config.getRadius(0), current_layer - 1, AvoidanceType::COLLISION, true, !xy_overrides).inside(p.first, true);
            }
        }
        return false;
    };
    return evaluatePoint;
}

std::pair<std::vector<TreeSupportTipGenerator::LineInformation>, std::vector<TreeSupportTipGenerator::LineInformation>> TreeSupportTipGenerator::splitLines(std::vector<TreeSupportTipGenerator::LineInformation> lines, std::function<bool(std::pair<Point, TreeSupportTipGenerator::LineStatus>)> evaluatePoint)
{
    // Assumes all Points on the current line are valid.

    constexpr bool KEEPING = true;
    constexpr bool FREEING = false;
    std::vector<LineInformation> keep(1);
    std::vector<LineInformation> set_free(1);

    for (const std::vector<std::pair<Point, LineStatus>>& line : lines)
    {
        auto current = KEEPING;
        LineInformation resulting_line;
        for (const std::pair<Point, LineStatus>& me : line)
        {
            if (evaluatePoint(me) == (current == FREEING))
            {
                if (! resulting_line.empty())
                {
                    (current == KEEPING ? keep : set_free).emplace_back(resulting_line);
                    resulting_line.clear();
                }
                current = !current;
            }
            resulting_line.emplace_back(me);
        }
        if (! resulting_line.empty())
        {
            (current == KEEPING ? keep : set_free).emplace_back(resulting_line);
        }
    }
    return std::pair<std::vector<std::vector<std::pair<Point, TreeSupportTipGenerator::LineStatus>>>, std::vector<std::vector<std::pair<Point, TreeSupportTipGenerator::LineStatus>>>>(keep, set_free);
}

Polygons TreeSupportTipGenerator::ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points,  bool enforce_distance) const
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
            line.add(middle_point.location);
        }
        else
        {
            size_t optimal_end_index = part.size() - 1;

            if (part.front() == part.back())
            {
                size_t optimal_start_index = 0;
                // If the polyline was a polygon, there is a high chance it was an overhang. Overhangs that are <60 degree tend to be very thin areas, so lets get the beginning and end of them and ensure that they are supported.
                // The first point of the line will always be supported, so rotate the order of points in this polyline that one of the two corresponding points that are furthest from each other is in the beginning.
                // The other will be manually added (optimal_end_index)
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
                Point current_point = part[0];
                line.add(part[0]);

                bool should_add_endpoint = min_points > 1 || vSize2(part[0] - part[optimal_end_index]) > (current_distance * current_distance);
                bool added_endpoint = !should_add_endpoint; // If no endpoint should be added all endpoints are already added.

                size_t current_index = 0;
                GivenDistPoint next_point;
                coord_t next_distance = current_distance;
                // Get points so that at least min_points are added and they each are current_distance away from each other. If that is impossible, decrease current_distance a bit.
                // (Regarding the while-loop) The input are lines, that means that the line from the last to the first vertex does not have to exist, so exclude all points that are on this line!
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
                    if(enforce_distance)
                    {
                        for (Point p : line)
                        {
                            min_distance_to_existing_point_sqd = std::min(min_distance_to_existing_point_sqd, vSize2(p - next_point.location));
                        }
                    }
                    if (!enforce_distance || min_distance_to_existing_point_sqd >= (current_distance * current_distance))
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
                            spdlog::warn("Tree Support: Encountered a fixpoint in getNextPointWithDistance. This is expected to happen if the distance (currently {}) is smaller than 100", next_distance);
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


SierpinskiFillProvider* TreeSupportTipGenerator::generateCrossFillProvider(const SliceMeshStorage& mesh, coord_t line_distance, coord_t line_width) const
{

    if (config.support_pattern == EFillMethod::CROSS || config.support_pattern == EFillMethod::CROSS_3D)
    {
        AABB3D aabb;
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            spdlog::warn("Tree support tried to generate a CrossFillProvider for a non model mesh.");
            return nullptr;
        }

        const coord_t aabb_expansion = mesh.settings.get<coord_t>("support_offset");
        AABB3D aabb_here(mesh.bounding_box);
        aabb_here.include(aabb_here.min - Point3(-aabb_expansion, -aabb_expansion, 0));
        aabb_here.include(aabb_here.max + Point3(-aabb_expansion, -aabb_expansion, 0));
        aabb.include(aabb_here);

        const std::string cross_subdisivion_spec_image_file = mesh.settings.get<std::string>("cross_support_density_image");
        std::ifstream cross_fs(cross_subdisivion_spec_image_file.c_str());
        if (cross_subdisivion_spec_image_file != "" && cross_fs.good())
        {
            return new SierpinskiFillProvider(aabb, line_distance, line_width, cross_subdisivion_spec_image_file);
        }
        else
        {
            return new SierpinskiFillProvider(aabb, line_distance, line_width);
        }
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
    max_layer = std::min(max_layer + cradle_layers, LayerIndex(mesh.overhang_areas.size() - 1));

    LayerIndex start_layer = 1;
    floating_parts_cache_.resize(max_layer+1);
    floating_parts_map_.resize(max_layer+1);

    Polygons completely_supported = volumes_.getCollision(0,0,true);
    Polygons layer_below = completely_supported;  //technically wrong, but the xy distance error on layer 1 should not matter
    for (size_t layer_idx = start_layer; layer_idx < max_layer; layer_idx++)
    {
        Polygons next_completely_supported;

        // As the collision contains z distance and xy distance it cant be used to clearly identify which parts of the model are connected to the buildplate.
        Polygons layer = mesh.layers[layer_idx].getOutlines();


        // todo rewrite to be able to parallelize it
        for (const Polygons part : layer.splitIntoParts())
        {

            AABB part_aabb(part);
            auto has_support_below = !PolygonUtils::clipPolygonWithAABB(layer_below,part_aabb).intersection(part).empty();

            if(! completely_supported.intersection(part).empty() || (! has_support_below && part.area() > cradle_area_threshold))
            {
                next_completely_supported.add(part);
                continue;
            }

            if (!has_support_below)
            {
                floating_parts_cache_[layer_idx].emplace_back(part,floating_parts_cache_[layer_idx].size(),0);
                floating_parts_map_ [layer_idx].emplace_back(std::vector<size_t>());
                continue;
            }



            size_t min_resting_on_layers = 0;
            bool add = false;
            for (auto [idx, floating] : floating_parts_cache_[layer_idx-1] | ranges::views::enumerate)
            {
                if(layer_idx > 1 && floating.height < cradle_layers - 1 && !floating.area.intersection(part).empty())
                {
                    floating_parts_map_[layer_idx-1][idx].emplace_back(floating_parts_cache_[layer_idx].size()) ;
                    min_resting_on_layers = std::max(min_resting_on_layers,floating.height);
                    add = true;
                }
            }

            if(min_resting_on_layers < cradle_layers && add)
            {
                floating_parts_map_ [layer_idx].emplace_back(std::vector<size_t>());
                floating_parts_cache_[layer_idx].emplace_back(part, floating_parts_cache_[layer_idx].size(),min_resting_on_layers+1);
            }
            else
            {
                next_completely_supported.add(part);
            }

        }
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
        spdlog::error("Requested not calculated unsupported area.", layer_idx);
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

void TreeSupportTipGenerator::generateCradle(const SliceMeshStorage& mesh, std::vector<Polygons>& support_free_areas)
{
    std::mutex critical_support_free_areas_and_cradle_areas;
    std::mutex critical_dedupe;
    std::vector<std::unordered_set<size_t>> dedupe(mesh.overhang_areas.size());

    calculateFloatingParts(mesh);

    cura::parallel_for<coord_t>(
        1,
        mesh.overhang_areas.size() - (z_distance_delta + 1),
        [&](const LayerIndex layer_idx)
        {
            if(mesh.overhang_areas[layer_idx + z_distance_delta].empty())
            {
                return;
            }
            for (auto pointy_info : getFullyUnsupportedArea(layer_idx + z_distance_delta))
            {
                AABB overhang_aabb(mesh.overhang_areas[layer_idx + z_distance_delta]);
                if (PolygonUtils::clipPolygonWithAABB(mesh.overhang_areas[layer_idx + z_distance_delta], overhang_aabb).intersection(pointy_info.area).empty())
                {
                    // It will be assumed that if it touches this mesh's overhang, it will be part of that mesh.
                    continue;
                }

                std::vector<Polygons> accumulated_model(std::min(cradle_layers + 1, mesh.overhang_areas.size() - layer_idx), Polygons());
                std::vector<size_t> all_pointy_idx{ pointy_info.index };

                Point assumed_center;

                Polygons shadow; // A combination of all outlines of the model that will be supported with a cradle.
                bool abort = false;
                bool contacted_other_pointy = false;
                for (size_t cradle_up_layer = 0; cradle_up_layer < accumulated_model.size(); cradle_up_layer++)
                {
                    Polygons relevant_forbidden =
                        volumes_.getAvoidance(0, layer_idx + cradle_up_layer, (only_gracious || ! config.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, true);

                    // shadow model up => not cradle where model
                    // then drop cradle down
                    // cut into parts => get close to original pointy that are far enough from each other.
                    std::vector<size_t> next_pointy_idx;
                    Polygons model_outline;
                    bool blocked_by_dedupe = false;
                    if (cradle_up_layer > 1) // The cradle base is below the bottommost unsupported and the first cradle layer is around it, so this will be needed only for the second one and up
                    {
                        if (cradle_up_layer == 2)
                        {
                            assumed_center = Polygon(shadow.getOutsidePolygons()[0]).centerOfMass();
                        }
                        else
                        {
                            Point next_center = Polygon(shadow.getOutsidePolygons()[0]).centerOfMass();
                            assumed_center = (2 * assumed_center + next_center) / 3;
                        }

                        for (size_t pointy_idx : all_pointy_idx)
                        {
                            for (auto next_pointy_data : getUnsupportedArea(layer_idx + cradle_up_layer - 1 + z_distance_delta, pointy_idx))
                            {
                                if (next_pointy_data.height != (cradle_up_layer - 1) + pointy_info.height) // If the area belongs to another pointy overhang stop and let this other overhang handle it
                                {
                                    contacted_other_pointy = true;
                                    continue;
                                }

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
                        if (cradle_up_layer < minimum_cradle_layers)
                        {
                            abort = true;
                            break;
                        }

                        if (! blocked_by_dedupe)
                        {
                            // The model is surrounded with cradle based on the area above (z distance).
                            // When an area that should have a cradle merges with a buildplate supported area above, it will no longer exist for a cradle.
                            // But if the cradle stops there will be z distance layer between the end of the cradle and said merge.
                            // To reduce the impact an area is estimated where the cradle should be for these areas.
                            Polygons previous_area = shadow;
                            for (size_t cradle_up_layer_z_delta = cradle_up_layer; cradle_up_layer_z_delta < std::min(cradle_up_layer + z_distance_delta - 1, accumulated_model.size()); cradle_up_layer_z_delta++)
                            {
                                Polygons possible_areas = shadow.offset(cradle_up_layer_z_delta * config.maximum_move_distance + config.xy_min_distance).intersection(mesh.layers[layer_idx + cradle_up_layer_z_delta - 1].getOutlines());
                                Polygons next_area;
                                for (auto part : possible_areas.splitIntoParts(true))
                                {
                                    if (! previous_area.intersection(part).empty())
                                    {
                                        next_area.add(part);
                                    }
                                }
                                previous_area = next_area;

                                accumulated_model[cradle_up_layer_z_delta] = next_area;
                            }
                        }
                        break;
                    }


                    model_outline = model_outline.unionPolygons();
                    shadow = shadow.unionPolygons(model_outline);

                    accumulated_model[cradle_up_layer] = shadow.offset(-config.maximum_move_distance).unionPolygons(model_outline);
                }

                if (abort)
                {
                    // If aborted remove all model information for the cradle generation except the pointy overhang, as it may be needed to cut a small hole in the large interface base.
                    for (size_t cradle_up_layer = 1; cradle_up_layer < accumulated_model.size(); cradle_up_layer++)
                    {
                        accumulated_model[cradle_up_layer].clear();
                    }
                }

                std::vector<Point> centers{ assumed_center };
                if (! contacted_other_pointy)
                {
                    for (int extra_points = 0; extra_points < 2; extra_points++)
                    {
                        coord_t candidate_min_distance2 = 0;
                        Point candidate = assumed_center;
                        for (auto line : shadow)
                        {
                            for (auto p : line)
                            {
                                coord_t smallest_distance2 = std::numeric_limits<coord_t>::max();
                                for (Point already_center : centers)
                                {
                                    smallest_distance2 = std::min(vSize2(p - already_center), smallest_distance2);
                                }
                                if (smallest_distance2 > candidate_min_distance2)
                                {
                                    candidate_min_distance2 = smallest_distance2;
                                    candidate = p;
                                }
                            }
                        }

                        if (candidate_min_distance2 > pow(5 * cradle_length, 2))
                        {
                            centers.emplace_back(candidate);
                        }
                    }
                }


                for (Point center : centers)
                {
                    std::vector<Polygons> cradle_areas_calc(accumulated_model.size());

                    for (auto [idx, model_shadow] : accumulated_model | ranges::views::enumerate)
                    {
                        if ((idx > 0 || (cradle_base_roof && !large_cradle_base && ! abort))
                            && ! model_shadow.empty()) // also calculate lines for the small cradle, idea is that the lines will touch the overhang and support it that way.
                        {
                            coord_t max_distance2 = 0;
                            Polygons relevant_forbidden = volumes_.getAvoidance(0, layer_idx + idx, (only_gracious || ! config.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, true);

                            for (auto line : model_shadow)
                            {
                                for (Point p : line)
                                {
                                    max_distance2 = std::max(max_distance2, vSize2(center - p));
                                }
                            }

                            Polygon max_outer_points = PolygonUtils::makeCircle(center, sqrt(max_distance2) + cradle_length * 2, (2 * M_PI) / cradle_lines);

                            // create lines that go from the furthest possible location to the center
                            Polygons lines_to_center;
                            for (Point p : max_outer_points)
                            {
                                lines_to_center.addLine(p, center);
                            }

                            // Subtract the model shadow up until this layer from the lines.
                            if (idx > 0)
                            {
                                lines_to_center = model_shadow.offset(config.xy_min_distance).unionPolygons().differencePolyLines(lines_to_center, false);
                            }
                            // Store valid distances from the center in relation to the direction of the line.
                            // Used to detect if a line may be intersecting another model part.
                            std::vector<std::pair<Point, coord_t>> vector_distance_map;

                            // shorten lines to be at most SUPPORT_TREE_CRADLE_WIDTH long, with the location closest to the center not changing
                            Polygons shortened_lines_to_center;
                            for (auto [line_idx, line] : lines_to_center | ranges::views::enumerate)
                            {
                                bool front_closer = vSize2(line.front() - center) < vSize2(line.back() - center);
                                Point closer = front_closer ? line.front() : line.back();
                                Point further = front_closer ? line.back() : line.front();
                                if (Polygon(line).polylineLength() <= cradle_length)
                                {
                                    shortened_lines_to_center.add(line);
                                }
                                else
                                {
                                    double scale = (double(cradle_length) / double(vSize(further - closer)));
                                    Point correct_length = closer + (further - closer) * scale;
                                    shortened_lines_to_center.addLine(correct_length, closer);
                                }

                                if (further != closer)
                                {
                                    vector_distance_map.emplace_back((further - closer), vSize(center - closer));
                                }
                            }
                            // If a line is drawn, but half of it removed as it would collide with the collision, there may not actually be a print line. The offset should prevent this.
                            shortened_lines_to_center = relevant_forbidden.offset(config.support_line_width / 2).differencePolyLines(shortened_lines_to_center, false);

                            // Evaluate which lines are still valid after the avoidance was subtracted
                            for (auto [line_idx, line] : shortened_lines_to_center | ranges::views::enumerate)
                            {
                                bool front_closer = vSize2(line.front() - center) < vSize2(line.back() - center);
                                Point closer = front_closer ? line.front() : line.back();
                                Point further = front_closer ? line.back() : line.front();
                                Point current_direction = further - closer;
                                coord_t distance_from_center = vSize(closer - center);
                                bool keep_line = false;
                                bool found_candidate = false;
                                const bool too_short = (vSize(closer - further)) < std::min((cradle_lines_roof ? config.support_roof_line_width : config.support_line_width) * 2, cradle_length / 2);
                                if (! too_short)
                                {
                                    for (auto [direction_idx, direction] : vector_distance_map | ranges::views::enumerate)
                                    {
                                        double cosine = (dot(direction.first, current_direction)) / (vSize(current_direction) * vSize(direction.first));
                                        if (cosine > 0.99)
                                        {
                                            found_candidate = true;
                                            // found line, check if there is one that is closer, if not then check if distance to center is expected
                                            bool found_close_line = direction.second + config.xy_min_distance + config.min_feature_size > distance_from_center;
                                            if (found_close_line)
                                            {
                                                keep_line = true;
                                                break;
                                            }
                                        }
                                    }
                                }

                                if (too_short || (! keep_line && found_candidate))
                                {
                                    shortened_lines_to_center[line_idx].clear();
                                }
                            }
                            Polygons cradle = shortened_lines_to_center.offsetPolyLine(cradle_line_width / 2).unionPolygons().offset(FUDGE_LENGTH).unionPolygons().offset(-FUDGE_LENGTH).difference(relevant_forbidden);
                            cradle_areas_calc[idx] = cradle;
                        }
                    }

                    // The base has to include the pointy overhang that has to be supported.
                    if (cradle_areas_calc.size() > 1)
                    {
                        if (cradle_base_roof && ! large_cradle_base)
                        {
                            cradle_areas_calc[0] = cradle_areas_calc[0].unionPolygons(accumulated_model[0]);
                        }
                        else
                        {
                            cradle_areas_calc[0] = accumulated_model[0];
                        }
                    }

                    // Rooflines are more delicate, so ensure that they rest on as much other rooflines if possible.
                    if (! use_fake_roof && support_roof_layers)
                    {
                        for (auto [idx, cradle] : cradle_areas_calc | ranges::views::enumerate | ranges::views::reverse)
                        {
                            if (idx > 1)
                            {
                                Polygons relevant_forbidden =
                                    volumes_.getAvoidance(0, layer_idx + idx - 1, (only_gracious || ! config.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, true);

                                Polygons area_above = cradle.difference(relevant_forbidden);

                                // If a line would have some areas that are too small to be drawn, there can be minor issues. The idea is that the offsets will filter them out.
                                for (auto part : area_above.offset(-cradle_line_width / 4).splitIntoParts())
                                {
                                    if (part.area() > cradle_line_width * std::max(cradle_length / 4, config.support_roof_line_width))
                                    {
                                        cradle_areas_calc[idx - 1].add(part.offset(cradle_line_width / 4));
                                    }
                                }

                                cradle_areas_calc[idx - 1] = cradle_areas_calc[idx - 1].unionPolygons();
                            }
                        }
                    }
                    for (auto [idx, cradle] : cradle_areas_calc | ranges::views::enumerate | ranges::views::reverse)
                    {
                        Polygons relevant_forbidden = volumes_.getAvoidance(0, layer_idx + idx, (only_gracious || ! config.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, true);
                        cradle_areas_calc[idx] = cradle_areas_calc[idx].difference(relevant_forbidden);
                    }

                    const coord_t closing_dist = sqrt(cradle_area_threshold / M_PI) + cradle_length + FUDGE_LENGTH;
                    const coord_t small_hole_size = EPSILON + (config.fill_outline_gaps ? config.min_feature_size / 2 - 5 : config.min_wall_line_width / 2 - 5); // based on calculation in WallToolPath
                    for (auto [idx, cradle] : cradle_areas_calc | ranges::views::enumerate)
                    {
                        if (! cradle.empty())
                        {
                            if (idx == 0)
                            {
                                if (! use_fake_roof && support_roof_layers)
                                {
                                    Polygons cut_line_base;
                                    if (large_cradle_base)
                                    {
                                        // If a large cradle base is used there needs to be a small hole cut into it to ensure that there will be a line for the pointy overhang to rest on.
                                        // This line is just a diagonal though the original pointy overhang area (from min to max). Technically this line is covering more than just the overhang, but that should not cause any issues.
                                        Point min_p = cradle.min();
                                        Point max_p = cradle.max();
                                        Polygons rest_line;
                                        rest_line.addLine(min_p, max_p);
                                        cut_line_base = rest_line.offsetPolyLine(small_hole_size);
                                    }
                                    Polygons closed = cradle.unionPolygons(cradle_areas_calc[1].offset(closing_dist, ClipperLib::jtRound).unionPolygons().offset(-closing_dist, ClipperLib::jtRound));
                                    {
                                        std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                                        for (size_t interface_down = 0; interface_down < layer_idx && interface_down < support_roof_layers; interface_down++)
                                        {
                                            support_free_areas[layer_idx - interface_down].add(cut_line_base);
                                            volumes_.addAreaToAntiPreferred(closed, layer_idx - interface_down);
                                        }
                                    }
                                    if (large_cradle_base)
                                    {
                                        cradle = closed.difference(cut_line_base);
                                    }
                                }

                                if(large_cradle_line_tips)
                                {
                                    cradle.add(cradle_areas_calc[1]);
                                }

                                cradle = cradle.unionPolygons();
                                cradle_base_areas[layer_idx].add(cradle);
                            }
                            else
                            {
                                Polygons unsupported_cradle_parts = cradle.difference(cradle_areas_calc[idx - 1].offset(config.maximum_move_distance));

                                Polygons closed = cradle.unionPolygons(cradle.offset(closing_dist + 2 * config.branch_radius, ClipperLib::jtRound).unionPolygons().offset(-closing_dist, ClipperLib::jtRound));
                                volumes_.addAreaToAntiPreferred(closed, layer_idx + idx);

                                {
                                    std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                                    cradle_areas[layer_idx + idx].add(cradle);
                                }

                                // Check which cradle lines overhang so much that they need support. If possible add support one layer below said line.
                                const Polygons relevant_forbidden =
                                    volumes_.getAvoidance(0, layer_idx + idx - 1, (only_gracious || ! config.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, true);
                                if (! unsupported_cradle_parts.empty())
                                {
                                    for (auto unsupported_part : unsupported_cradle_parts.splitIntoParts())
                                    {
                                        if (unsupported_part.difference(relevant_forbidden).empty())
                                        {
                                            std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                                            cradle_overhang[layer_idx + idx].add(unsupported_cradle_parts);
                                        }
                                        else
                                        {
                                            std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                                            cradle_overhang[layer_idx + idx - 1].add(unsupported_cradle_parts);
                                        }
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
        mesh.layers.size(),
        [&](const LayerIndex layer_idx)
        {
            if(layer_idx < cradle_areas[layer_idx].size())
            {
                cradle_areas[layer_idx] = cradle_areas[layer_idx].unionPolygons();

            }
            if(layer_idx < cradle_overhang[layer_idx].size())
            {
                cradle_overhang[layer_idx] = cradle_overhang[layer_idx].unionPolygons();

            }
            if(layer_idx < cradle_base_areas[layer_idx].size())
            {
                cradle_base_areas[layer_idx] = cradle_base_areas[layer_idx].unionPolygons();

            }
            if(layer_idx < support_free_areas[layer_idx].size())
            {
                support_free_areas[layer_idx] = support_free_areas[layer_idx].unionPolygons();

            }
        });

}

void TreeSupportTipGenerator::dropOverhangAreas(const SliceMeshStorage& mesh, std::vector<Polygons>& result, bool roof )
{

    std::mutex critical;

    //this is ugly, but as far as i can see there is not way to ensure a parallel_for loop is calculated for each iteration up to a certain point before it continues;
    cura::parallel_for<coord_t>
        (
            1,
            mesh.overhang_areas.size() - z_distance_delta,
            [&](const LayerIndex layer_idx)
            {
                if (mesh.overhang_areas[layer_idx + z_distance_delta].empty() || result.size()<layer_idx)
                {
                    return; // This is a continue if imagined in a loop context.
                }

                Polygons relevant_forbidden = volumes_.getCollision(roof ? 0 : config.getRadius(0), layer_idx, ! xy_overrides);
                // ^^^ Take the least restrictive avoidance possible

                // Technically this also makes support blocker smaller, which is wrong as they do not have a xy_distance, but it should be good enough.
                Polygons model_outline = volumes_.getCollision(0, layer_idx, ! xy_overrides).offset(-config.xy_min_distance, ClipperLib::jtRound);

                //Use full_overhang to ensure dropped overhang will overlap with overhang further down. not leaving a small hole between model and roof where support could creep into.
                Polygons overhang_full =
                    TreeSupportUtils::safeOffsetInc(mesh.full_overhang_areas[layer_idx + z_distance_delta], roof ? roof_outset : support_outset, relevant_forbidden, config.min_radius * 1.75 + config.xy_min_distance, 0, 1, config.support_line_distance / 2, &config.simplifier);
                Polygons remaining_overhang =
                    mesh.full_overhang_areas[layer_idx + z_distance_delta].offset(roof ? roof_outset : support_outset).difference(overhang_full).intersection(relevant_forbidden).difference(model_outline);
                for (size_t lag_ctr = 1; lag_ctr <= max_overhang_insert_lag && layer_idx - coord_t(lag_ctr) >= 1 && !remaining_overhang.empty(); lag_ctr++)
                {
                    {
                        std::lock_guard<std::mutex> critical_section_storage(critical);
                        result[layer_idx-lag_ctr].add(remaining_overhang);
                    }

                    Polygons relevant_forbidden_below =
                        volumes_.getCollision(roof ? 0 : config.getRadius(0), layer_idx - lag_ctr, ! xy_overrides).offset(EPSILON);
                    remaining_overhang=remaining_overhang.intersection(relevant_forbidden_below).unionPolygons().difference(model_outline);
                }
            }
        );

    cura::parallel_for<coord_t>
        (
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
    std::vector<Polygons> potential_support_roofs(mesh.overhang_areas.size(),Polygons());
    std::mutex critical_potential_support_roofs;
    std::vector<Polygons> dropped_overhangs(mesh.overhang_areas.size(),Polygons());

    if (xy_overrides)
    {
        dropOverhangAreas(mesh, dropped_overhangs, true);
    }

    cura::parallel_for<coord_t>
        (
            0,
            mesh.overhang_areas.size() - z_distance_delta,
            [&](const LayerIndex layer_idx)
            {
                if (mesh.overhang_areas[layer_idx + z_distance_delta].empty())
                {
                    return; // This is a continue if imagined in a loop context.
                }

                // Roof does not have a radius, so remove it using offset. Note that there is no 0 radius avoidance, and it would not be identical with the avoidance offset with -radius.
                // This is intentional here, as support roof is still valid if only a part of the tip may reach it.
                Polygons forbidden_here =
                    volumes_.getAvoidance(0, layer_idx, (only_gracious||!config.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, ! xy_overrides);

                // todo Since arachnea the assumption that an area smaller then line_width is not printed is no longer true all such safeOffset should have config.support_line_width replaced with another setting. It should still work in most cases, but it should be possible to create a situation where a overhang outset lags though a wall.
                // I will take a look at this later.
                Polygons full_overhang_area  =  TreeSupportUtils::safeOffsetInc(mesh.full_overhang_areas[layer_idx + z_distance_delta].unionPolygons(dropped_overhangs[layer_idx]),roof_outset,forbidden_here, config.support_line_width, 0, 1, config.support_line_distance / 2, &config.simplifier);

                for (LayerIndex dtt_roof = 0; dtt_roof < support_roof_layers && layer_idx - dtt_roof >= 1; dtt_roof++)
                {
                    const Polygons forbidden_next =
                        volumes_.getAvoidance(0, layer_idx - (dtt_roof + 1), (only_gracious||!config.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, ! xy_overrides);

                    full_overhang_area = full_overhang_area.difference(forbidden_next);


                    if(force_minimum_roof_area)
                    {
                        full_overhang_area.removeSmallAreas(minimum_roof_area);
                    }

                    if (full_overhang_area.area()>EPSILON)
                    {
                        std::lock_guard<std::mutex> critical_section_potential_support_roofs(critical_potential_support_roofs);
                        potential_support_roofs[layer_idx-dtt_roof].add((full_overhang_area));
                    }
                    else
                    {
                        break;
                    }
                }

            }
        );

    cura::parallel_for<coord_t>
    (
        0,
        potential_support_roofs.size(),
        [&](const LayerIndex layer_idx)
        {
            // Now, because the avoidance/collision was subtracted above, the overhang parts that are of xy distance were removed, so to merge areas that should have been one offset by xy_min_distance and then undo it.
            // In a perfect world the offset here would be of a mode that makes sure that area.offset(config.xy_min_distance).unionPolygons().offset(-config.xy_min_distance) = area if there is only one polygon in said area.
            // I have not encountered issues with using the default mitered here. Could be that i just have not encountered an issue with it yet though.

            potential_support_roofs[layer_idx] = potential_support_roofs[layer_idx].unionPolygons().offset(config.xy_min_distance).unionPolygons().offset(-config.xy_min_distance).unionPolygons(potential_support_roofs[layer_idx]);
        }
    );

    std::vector<Polygons> additional_support_roofs(mesh.overhang_areas.size(),Polygons());

    cura::parallel_for<coord_t>
        (
            0,
            potential_support_roofs.size(),
            [&](const LayerIndex layer_idx)
            {
                if (!potential_support_roofs[layer_idx].empty())
                {
                    // Roof does not have a radius, so remove it using offset. Note that there is no 0 radius avoidance, and it would not be identical with the avoidance offset with -radius.
                    // This is intentional here, as support roof is still valid if only a part of the tip may reach it.
                    Polygons forbidden_here =
                        volumes_.getAvoidance(0, layer_idx, (only_gracious||!config.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, ! xy_overrides);

                    if (!force_minimum_roof_area)
                    {
                        Polygons fuzzy_area = Polygons();

                        // the roof will be combined with roof above and below, to see if a part of this roof may be part of a valid roof further up/down.
                        // This prevents the situation where a roof gets removed even tough its area would contribute to a (better) printable roof area further down.
                        for (const LayerIndex layer_offset : ranges::views::iota(-LayerIndex(std::min(size_t(layer_idx),support_roof_layers)), LayerIndex(std::min(size_t(potential_support_roofs.size()-layer_idx),support_roof_layers+1))))
                        {
                            fuzzy_area.add(support_roof_drawn[layer_idx+layer_offset]);
                            fuzzy_area.add(potential_support_roofs[layer_idx+layer_offset]);
                        }
                        fuzzy_area=fuzzy_area.unionPolygons();
                        fuzzy_area.removeSmallAreas(std::max(minimum_roof_area,tip_roof_size));

                        for (Polygons potential_roof:potential_support_roofs[layer_idx].difference(forbidden_here).splitIntoParts())
                        {

                            if (!potential_roof.intersection(fuzzy_area).empty())
                            {
                                additional_support_roofs[layer_idx].add(potential_roof);
                            }

                        }
                    }
                    else
                    {
                        Polygons valid_roof = potential_support_roofs[layer_idx].difference(forbidden_here);
                        valid_roof.removeSmallAreas(std::max(minimum_roof_area,tip_roof_size));
                        additional_support_roofs[layer_idx].add(valid_roof);
                    }
                }
            }
        );


    if(cradle_base_roof)
    {
        // The cradle has to be added after the closing operation above, as otherwise the holes in the cradle that ensure it can be removed will be closed.
        cura::parallel_for<coord_t>
            (
                1,
                cradle_areas.size(),
                [&](const LayerIndex layer_idx)
                {
                    if (cradle_areas[layer_idx].empty() && cradle_base_areas[layer_idx].empty())
                    {
                        return; // This is a continue if imagined in a loop context.
                    }

                    // Roof does not have a radius, so remove it using offset. Note that there is no 0 radius avoidance, and it would not be identical with the avoidance offset with -radius.
                    // This is intentional here, as support roof is still valid if only a part of the tip may reach it.
                    Polygons forbidden_here =
                        volumes_.getAvoidance(0, layer_idx, (only_gracious||!config.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, ! xy_overrides);

                    Polygons full_overhang_area;
                    full_overhang_area.add(cradle_base_areas[layer_idx]);
                    full_overhang_area=full_overhang_area.unionPolygons();

                    full_overhang_area  =  TreeSupportUtils::safeOffsetInc(full_overhang_area,roof_outset,forbidden_here, config.support_line_width, 0, 1, config.support_line_distance / 2, &config.simplifier);


                    for (LayerIndex dtt_roof = 0; dtt_roof < support_roof_layers && layer_idx - dtt_roof >= 1; dtt_roof++)
                    {
                        const Polygons forbidden_next =
                            volumes_.getAvoidance(0, layer_idx - (dtt_roof + 1), (only_gracious||!config.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, ! xy_overrides);

                        full_overhang_area = full_overhang_area.difference(forbidden_next);


                        if (full_overhang_area.area()>EPSILON)
                        {
                            std::lock_guard<std::mutex> critical_section_potential_support_roofs(critical_potential_support_roofs);
                            additional_support_roofs[layer_idx-dtt_roof].add((full_overhang_area));
                        }
                        else
                        {
                            break;
                        }
                    }

                }
            );
    }


    cura::parallel_for<coord_t>
        (
            0,
            additional_support_roofs.size(),
            [&](const LayerIndex layer_idx)
            {
                support_roof_drawn[layer_idx] = support_roof_drawn[layer_idx].unionPolygons(additional_support_roofs[layer_idx]);
            }
        );
}


void TreeSupportTipGenerator::addPointAsInfluenceArea(std::vector<std::set<TreeSupportElement *>>& move_bounds, std::pair<Point, TreeSupportTipGenerator::LineStatus> p, size_t dtt, LayerIndex insert_layer, size_t dont_move_until, bool roof, bool cradle, bool skip_ovalisation, std::vector<Point> additional_ovalization_targets)
{
    const bool to_bp = p.second == LineStatus::TO_BP || p.second == LineStatus::TO_BP_SAFE;
    const bool gracious = to_bp || p.second == LineStatus::TO_MODEL_GRACIOUS || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
    const bool safe_radius = p.second == LineStatus::TO_BP_SAFE || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
    if (! config.support_rests_on_model && !to_bp)
    {
        spdlog::warn("Tried to add an invalid support point");
        return;
    }
    Polygon circle;
    Polygon base_circle = TreeSupportBaseCircle::getBaseCircle();
    for (Point corner : base_circle)
    {
        circle.add(p.first + corner);
    }
    Polygons area = circle.offset(0);
    {
        std::lock_guard<std::mutex> critical_section_movebounds(critical_move_bounds);
        if (! already_inserted[insert_layer].count(p.first / ((config.min_radius + 1) / 10)))
        {
            // Normalize the point a bit to also catch points which are so close that inserting it would achieve nothing.
            already_inserted[insert_layer].emplace(p.first / ((config.min_radius + 1) / 10));
            TreeSupportElement* elem =
                new TreeSupportElement
                (
                    dtt,
                    insert_layer,
                    p.first,
                    to_bp,
                    gracious,
                    ! xy_overrides,
                    dont_move_until,
                    roof || cradle,
                    safe_radius,
                    force_tip_to_roof,
                    skip_ovalisation,
                    support_tree_limit_branch_reach,
                    support_tree_branch_reach_limit
                );
            elem->area = new Polygons(area);

            for (Point p : additional_ovalization_targets)
            {
                elem->additional_ovalization_targets.emplace_back(p);
            }

            move_bounds[insert_layer].emplace(elem);
        }
    }
}


void TreeSupportTipGenerator::addLinesAsInfluenceAreas(std::vector<std::set<TreeSupportElement *>>& move_bounds, std::vector<TreeSupportTipGenerator::LineInformation> lines, size_t roof_tip_layers, LayerIndex insert_layer_idx, bool supports_roof, bool supports_cradle, size_t dont_move_until, bool connect_points)
{
    // Add tip area as roof (happens when minimum roof area > minimum tip area) if possible. This is required as there is no guarantee that if support_roof_wall_count == 0 that a certain roof area will actually have lines.
    size_t dtt_roof_tip = 0;
    coord_t base_radius = config.getRadius(supports_cradle ? cradle_tip_dtt: 0);
    if (config.support_roof_wall_count == 0 && ! supports_cradle)
    {
        for (dtt_roof_tip = 0; dtt_roof_tip < roof_tip_layers && insert_layer_idx - dtt_roof_tip >= 1; dtt_roof_tip++)
        {


            std::function<bool(std::pair<Point, LineStatus>)> evaluateRoofWillGenerate = [&](std::pair<Point, LineStatus> p)
            {
                Polygon roof_circle;
                for (Point corner : TreeSupportBaseCircle::getBaseCircle())
                {
                    roof_circle.add(p.first + corner * std::max(base_radius / TreeSupportBaseCircle::base_radius, coord_t(1)));
                }
                Polygons area = roof_circle.offset(0);
                return !TreeSupportUtils::generateSupportInfillLines(area, config, true, insert_layer_idx - dtt_roof_tip,support_roof_line_distance,cross_fill_provider,true).empty();
            };

            std::pair<std::vector<TreeSupportTipGenerator::LineInformation>, std::vector<TreeSupportTipGenerator::LineInformation>> split =
                splitLines(lines, getEvaluatePointForNextLayerFunction(insert_layer_idx - dtt_roof_tip)); // Keep all lines that are still valid on the next layer.

            for (LineInformation line : split.second) // Add all points that would not be valid.
            {
                for (std::pair<Point, TreeSupportTipGenerator::LineStatus> point_data : line)
                {
                    addPointAsInfluenceArea(move_bounds, point_data, 0, insert_layer_idx - dtt_roof_tip, roof_tip_layers - dtt_roof_tip, dtt_roof_tip != 0, false, false);
                }
            }

            // Not all roofs are guaranteed to actually generate lines, so filter these out and add them as points.
            split = splitLines(split.first, evaluateRoofWillGenerate);
            lines = split.first;

            for (LineInformation line : split.second)
            {
                for (std::pair<Point, TreeSupportTipGenerator::LineStatus> point_data : line)
                {
                    addPointAsInfluenceArea(move_bounds, point_data, 0, insert_layer_idx - dtt_roof_tip, roof_tip_layers - dtt_roof_tip, dtt_roof_tip != 0, false, false);
                }
            }

            // Add all tips as roof to the roof storage.
            Polygons added_roofs;
            for (LineInformation line : lines)
            {
                for (std::pair<Point, TreeSupportTipGenerator::LineStatus> p : line)
                {
                    Polygon roof_circle;
                    for (Point corner : TreeSupportBaseCircle::getBaseCircle())
                    {
                        roof_circle.add(p.first + corner * std::max(base_radius / TreeSupportBaseCircle::base_radius, coord_t(1)));
                    }
                    added_roofs.add(roof_circle);
                }
            }
            added_roofs = added_roofs.unionPolygons();
            {
                std::lock_guard<std::mutex> critical_section_roof(critical_roof_tips);

                roof_tips_drawn[insert_layer_idx - dtt_roof_tip].add(added_roofs);
            }
        }
    }

    for (LineInformation line : lines)
    {
        // If a line consists of enough tips, the assumption is that it is not a single tip, but part of a simulated support pattern.
        // Ovalisation should be disabled if they may be placed close to each other to prevent tip-areas merging. If the tips has to turn into roof, the area is most likely not large enough for this to cause issues.
        const bool disable_ovalization =  !connect_points && config.min_radius < 3 * config.support_line_width && roof_tip_layers == 0 && dtt_roof_tip == 0;
        for (auto [idx, point_data] : line | ranges::views::enumerate)
        {
            std::vector<Point> additional_ovalization_targets;
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
            addPointAsInfluenceArea
                (
                    move_bounds,
                    point_data,
                    supports_cradle ? cradle_tip_dtt : 0,
                    insert_layer_idx - dtt_roof_tip,
                    dont_move_until > dtt_roof_tip ? dont_move_until - dtt_roof_tip : 0,
                    dtt_roof_tip != 0 || supports_roof, supports_cradle, disable_ovalization,
                    additional_ovalization_targets
                );
        }
    }
}


void TreeSupportTipGenerator::removeUselessAddedPoints(std::vector<std::set<TreeSupportElement *>>& move_bounds,SliceDataStorage& storage, std::vector<Polygons>& additional_support_areas)
{

    cura::parallel_for<coord_t>
        (
            0,
            move_bounds.size(),
            [&](const LayerIndex layer_idx)
            {

                if(layer_idx+1 < storage.support.supportLayers.size())
                {
                    std::vector<TreeSupportElement *> to_be_removed;
                    Polygons roof_on_layer_above = (use_fake_roof ? support_roof_drawn[layer_idx+1] : storage.support.supportLayers[layer_idx+1].support_roof).unionPolygons(additional_support_areas[layer_idx+1]);
                    Polygons roof_on_layer = (use_fake_roof ? support_roof_drawn[layer_idx] : storage.support.supportLayers[layer_idx].support_roof).unionPolygons(additional_support_areas[layer_idx]);
                    roof_on_layer_above.add(cradle_base_areas[layer_idx]);
                    roof_on_layer_above = roof_on_layer_above.unionPolygons(cradle_overhang[layer_idx]);

                    for (TreeSupportElement* elem : move_bounds[layer_idx])
                    {
                        if (roof_on_layer.inside(elem->result_on_layer)) // Remove branches that start inside of support interface
                        {
                            to_be_removed.emplace_back(elem);
                        }
                        else if(elem->supports_roof)
                        {
                            Point from = elem->result_on_layer;
                            PolygonUtils::moveInside(roof_on_layer_above,from);
                            // Remove branches should have interface above them, but don't. Should never happen.
                            if (roof_on_layer_above.empty()  ||
                                (!roof_on_layer_above.inside(elem->result_on_layer) && vSize2(from-elem->result_on_layer) > std::pow(config.getRadius(0) + FUDGE_LENGTH,2)))
                            {
                                to_be_removed.emplace_back(elem);
                                spdlog::warn("Removing already placed tip that should have roof above it. Distance from roof is {}.",vSize(from-elem->result_on_layer));
                            }
                        }
                    }

                    for (auto elem : to_be_removed)
                    {
                        move_bounds[layer_idx].erase(elem);
                        delete elem->area;
                        delete elem;
                    }
                }
            }
        );
}


void TreeSupportTipGenerator::generateTips(SliceDataStorage& storage,const SliceMeshStorage& mesh, std::vector<std::set<TreeSupportElement*>>& move_bounds, std::vector<Polygons>& additional_support_areas, std::vector<Polygons>& placed_support_lines_support_areas, std::vector<Polygons>& support_free_areas)
{
    std::vector<std::set<TreeSupportElement*>> new_tips(move_bounds.size());

    const coord_t circle_length_to_half_linewidth_change = config.min_radius < config.support_line_width ? config.min_radius / 2 : sqrt(square(config.min_radius) - square(config.min_radius - config.support_line_width / 2));
    // ^^^ As r*r=x*x+y*y (circle equation): If a circle with center at (0,0) the top most point is at (0,r) as in y=r. This calculates how far one has to move on the x-axis so that y=r-support_line_width/2.
    //     In other words how far does one need to move on the x-axis to be support_line_width/2 away from the circle line. As a circle is round this length is identical for every axis as long as the 90� angle between both remains.

    const coord_t extra_outset = std::max(coord_t(0), config.min_radius - config.support_line_width / 2) + (xy_overrides ? 0 : config.support_line_width / 2);
    // ^^^ Extra support offset to compensate for larger tip radiis. Also outset a bit more when z overwrites xy, because supporting something with a part of a support line is better than not supporting it at all.

    if(cradle_layers>0)
    {
        generateCradle(mesh,support_free_areas);
    }

    if (support_roof_layers)
    {
        calculateRoofAreas(mesh);
    }

    cura::parallel_for<coord_t>
        (
            1,
            mesh.overhang_areas.size() - z_distance_delta,
            [&](const LayerIndex layer_idx)
            {
                if (mesh.overhang_areas[layer_idx + z_distance_delta].empty() &&
                    (layer_idx+1 >= support_roof_drawn.size() || support_roof_drawn[layer_idx+1].empty()) &&
                    (layer_idx >= cradle_base_areas.size() || cradle_base_areas[layer_idx].empty()) &&
                    (layer_idx >= cradle_overhang.size() || cradle_overhang[layer_idx].empty())
                )
                {
                    return; // This is a continue if imagined in a loop context.
                }

                Polygons relevant_forbidden =
                    volumes_.getAvoidance(config.getRadius(0), layer_idx, (only_gracious || !config.support_rests_on_model)? AvoidanceType::FAST : AvoidanceType::COLLISION , config.support_rests_on_model, ! xy_overrides);
                // ^^^ Take the least restrictive avoidance possible
                relevant_forbidden = relevant_forbidden.offset(EPSILON).unionPolygons(); // Prevent rounding errors down the line, points placed directly on the line of the forbidden area may not be added otherwise.

                std::function<Polygons(const Polygons&, bool, LayerIndex)> generateLines =
                    [&](const Polygons& area, bool roof, LayerIndex layer_idx)
                {

                    coord_t upper_line_distance = support_supporting_branch_distance;
                    coord_t line_distance = std::max(roof ? support_roof_line_distance : support_tree_branch_distance, upper_line_distance);

                    bool use_grid = line_distance == upper_line_distance;

                    return TreeSupportUtils::generateSupportInfillLines(area, config, roof && !use_fake_roof, layer_idx, line_distance , cross_fill_provider, roof && !use_fake_roof, use_grid ? EFillMethod::GRID:EFillMethod::NONE);
                };

                enum class OverhangType {REGULAR, ROOF, CRADLE};

                std::vector<std::pair<Polygons, OverhangType>> overhang_processing;
                // ^^^ Every overhang has saved if a roof should be generated for it.
                //     This can NOT be done in the for loop as an area may NOT have a roof even if it is larger than the minimum_roof_area when it is only larger because of the support horizontal expansion and it would not have a roof if the overhang is offset by support roof horizontal expansion instead.
                //     (At least this is the current behavior of the regular support)

                Polygons core_overhang = mesh.overhang_areas[layer_idx + z_distance_delta];

                if(cradle_overhang.size() > layer_idx)
                {
                    core_overhang = core_overhang.unionPolygons(cradle_overhang[layer_idx]);
                }


                if (support_roof_layers && layer_idx+1 < support_roof_drawn.size())
                {
                    core_overhang = core_overhang.difference(support_roof_drawn[layer_idx]);
                    for (Polygons roof_part : support_roof_drawn[layer_idx+1].difference(support_roof_drawn[layer_idx].offset(config.maximum_move_distance_slow)).splitIntoParts(true)) //If there is a roof, the roof will be one layer above the tips.
                    {
                        //^^^Technically one should also subtract the avoidance of radius 0 (similarly how calculated in calculateRoofArea), as there can be some rounding errors introduced since then. But this does not fully prevent some rounding errors either way, so just handle the error later.
                        overhang_processing.emplace_back(roof_part, OverhangType::ROOF);
                    }
                }

                core_overhang = core_overhang.difference(TreeSupportUtils::safeOffsetInc(cradle_base_areas[layer_idx], support_outset + EPSILON, relevant_forbidden, config.min_radius * 1.75 + config.xy_min_distance, 0, 1, config.support_line_distance / 2, &config.simplifier));
                core_overhang = core_overhang.difference(support_free_areas[layer_idx]);
                for (Polygons cradle_base : cradle_base_areas[layer_idx].splitIntoParts(true))
                {
                    overhang_processing.emplace_back(cradle_base, OverhangType::CRADLE);
                }

                Polygons overhang_regular =
                    TreeSupportUtils::safeOffsetInc(core_overhang, support_outset, relevant_forbidden, config.min_radius * 1.75 + config.xy_min_distance, 0, 1, config.support_line_distance / 2, &config.simplifier);
                Polygons remaining_overhang =
                    core_overhang.offset(support_outset).difference(overhang_regular.offset(config.support_line_width * 0.5)).intersection(relevant_forbidden);


                // Offset ensures that areas that could be supported by a part of a support line, are not considered unsupported overhang
                coord_t extra_total_offset_acc = 0;

                // Offset the area to compensate for large tip radiis. Offset happens in multiple steps to ensure the tip is as close to the original overhang as possible.

                {
                    Polygons already_supported = support_roof_drawn[layer_idx];
                    already_supported.add(cradle_areas[layer_idx]);
                    already_supported.add(support_free_areas[layer_idx]); // While point there are not supported, there may be no support anyway.
                    already_supported=already_supported.unionPolygons();
                    while (extra_total_offset_acc + config.support_line_width / 8 < extra_outset) //+mesh_config.support_line_width / 8  to avoid calculating very small (useless) offsets because of rounding errors.
                    {
                        coord_t offset_current_step =
                            extra_total_offset_acc + 2 * config.support_line_width > config.min_radius ?
                                                                                                                 std::min(config.support_line_width / 8, extra_outset - extra_total_offset_acc) :
                                                                                                                 std::min(circle_length_to_half_linewidth_change, extra_outset - extra_total_offset_acc);
                        extra_total_offset_acc += offset_current_step;
                        Polygons overhang_offset = TreeSupportUtils::safeOffsetInc(overhang_regular, 1.5 * extra_total_offset_acc, volumes_.getCollision(0, layer_idx, true), config.xy_min_distance + config.support_line_width, 0, 1, config.support_line_distance / 2, &config.simplifier);
                        remaining_overhang = remaining_overhang.difference(overhang_offset.unionPolygons(already_supported.offset(1.5 * extra_total_offset_acc))).unionPolygons(); //overhang_offset is combined with roof, as all area that has a roof, is already supported by said roof.
                        Polygons next_overhang = TreeSupportUtils::safeOffsetInc(remaining_overhang, extra_total_offset_acc, volumes_.getCollision(0, layer_idx, true), config.xy_min_distance + config.support_line_width, 0, 1, config.support_line_distance / 2, &config.simplifier);
                        overhang_regular = overhang_regular.unionPolygons(next_overhang.difference(relevant_forbidden));
                    }
                }

                // If the xy distance overrides the z distance, some support needs to be inserted further down.
                //=> Analyze which support points do not fit on this layer and check if they will fit a few layers down
                //   (while adding them an infinite amount of layers down would technically be closer the setting description, it would not produce reasonable results. )
                if (xy_overrides)
                {

                    for (Polygons& remaining_overhang_part:remaining_overhang.splitIntoParts(false))
                    {

                        std::vector<LineInformation> overhang_lines;
                        Polygons polylines = ensureMaximumDistancePolyline(generateLines(remaining_overhang_part, false, layer_idx), config.min_radius, 1, false);
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
                            for (Point p : line)
                            {
                                res_line.emplace_back(p, LineStatus::INVALID);
                            }
                            overhang_lines.emplace_back(res_line);
                        }

                        for (size_t lag_ctr = 1; lag_ctr <= max_overhang_insert_lag && !overhang_lines.empty() && layer_idx - coord_t(lag_ctr) >= 1; lag_ctr++)
                        {
                            // get least restricted avoidance for layer_idx-lag_ctr
                            Polygons relevant_forbidden_below =
                                volumes_.getAvoidance(config.getRadius(0), layer_idx - lag_ctr, (only_gracious||!config.support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION, config.support_rests_on_model, ! xy_overrides);
                            // It is not required to offset the forbidden area here as the points won't change:
                            // If points here are not inside the forbidden area neither will they be later when placing these points, as these are the same points.
                            std::function<bool(std::pair<Point, LineStatus>)> evaluatePoint =
                                [&](std::pair<Point, LineStatus> p) { return relevant_forbidden_below.inside(p.first, true); };

                            Polygons already_supported = support_roof_drawn[layer_idx-lag_ctr];
                            already_supported.add(cradle_areas[layer_idx-lag_ctr]);
                            already_supported.add(support_free_areas[layer_idx-lag_ctr]); // While point there are not supported, there may be no support anyway.
                            already_supported=already_supported.unionPolygons();

                            //Remove all points that are for some reason are already supported
                            std::function<bool(std::pair<Point, LineStatus>)> evaluateAlreadySupported =
                                [&](std::pair<Point, LineStatus> p) { return already_supported.inside(p.first, true); };

                            overhang_lines = splitLines(overhang_lines, evaluateAlreadySupported).second;

                            std::pair<std::vector<TreeSupportTipGenerator::LineInformation>, std::vector<TreeSupportTipGenerator::LineInformation>> split = splitLines(overhang_lines, evaluatePoint); // Keep all lines that are invalid.
                            overhang_lines = split.first;
                            std::vector<LineInformation> fresh_valid_points = convertLinesToInternal(convertInternalToLines(split.second), layer_idx - lag_ctr);
                            // ^^^ Set all now valid lines to their correct LineStatus. Easiest way is to just discard Avoidance information for each point and evaluate them again.

                            addLinesAsInfluenceAreas(new_tips,fresh_valid_points, (force_tip_to_roof && lag_ctr <= support_roof_layers) ? support_roof_layers : 0, layer_idx - lag_ctr, false, false, support_roof_layers, false);
                        }
                    }
                }

                overhang_regular.removeSmallAreas(minimum_support_area);

                for (Polygons support_part : overhang_regular.splitIntoParts(true))
                {
                    overhang_processing.emplace_back(support_part, OverhangType::REGULAR);
                }

                for (std::pair<Polygons, OverhangType> overhang_pair : overhang_processing)
                {
                    const bool roof_allowed_for_this_part = overhang_pair.second == OverhangType::ROOF;
                    const bool supports_cradle = overhang_pair.second == OverhangType::CRADLE;

                    Polygons overhang_outset = overhang_pair.first;
                    const size_t min_support_points = std::max(coord_t(2), std::min(coord_t(EPSILON), overhang_outset.polygonLength() / connect_length));
                    std::vector<LineInformation> overhang_lines;

                    bool only_lines = true;

                    // The tip positions are determined here.
                    // todo can cause inconsistent support density if a line exactly aligns with the model
                    Polygons polylines =
                        ensureMaximumDistancePolyline
                        (
                            generateLines(overhang_outset, roof_allowed_for_this_part, layer_idx + roof_allowed_for_this_part), !roof_allowed_for_this_part ? config.min_radius * 2 : use_fake_roof ? support_supporting_branch_distance : connect_length , 1, false
                        );


                    // support_line_width to form a line here as otherwise most will be unsupported.
                    // Technically this violates branch distance, but not only is this the only reasonable choice,
                    //   but it ensures consistent behaviour as some infill patterns generate each line segment as its own polyline part causing a similar line forming behaviour.
                    // This is not done when a roof is above as the roof will support the model and the trees only need to support the roof

                    if (polylines.pointCount() <= min_support_points)
                    {
                        only_lines = false;
                        // Add the outer wall (of the overhang) to ensure it is correct supported instead.
                        // Try placing the support points in a way that they fully support the outer wall, instead of just the with half of the support line width.
                        Polygons reduced_overhang_outset = overhang_outset.offset(-config.support_line_width / 2.2);
                        // ^^^ It's assumed that even small overhangs are over one line width wide, so lets try to place the support points in a way that the full support area generated from them will support the overhang.
                        //     (If this is not done it may only be half). This WILL NOT be the case when supporting an angle of about < 60� so there is a fallback, as some support is better than none.)
                        if (! reduced_overhang_outset.empty() && overhang_outset.difference(reduced_overhang_outset.offset(std::max(config.support_line_width, connect_length))).area() < 1)
                        {
                            polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(reduced_overhang_outset), connect_length, min_support_points, true);
                        }
                        else
                        {
                            polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(overhang_outset), connect_length, min_support_points, true);
                        }
                    }

                    if (roof_allowed_for_this_part || supports_cradle) //Some roof may only be supported by a part of a tip
                    {
                        polylines = TreeSupportUtils::movePointsOutside(polylines,relevant_forbidden,config.getRadius(0)+FUDGE_LENGTH/2);
                    }

                    overhang_lines = convertLinesToInternal(polylines, layer_idx );

                    if(overhang_lines.empty()) //some error handling and logging
                    {
                        Polygons enlarged_overhang_outset = overhang_outset.offset(config.getRadius(0)+FUDGE_LENGTH/2,ClipperLib::jtRound).difference(relevant_forbidden);
                        polylines = ensureMaximumDistancePolyline(TreeSupportUtils::toPolylines(enlarged_overhang_outset), connect_length, min_support_points, true);
                        overhang_lines = convertLinesToInternal(polylines, layer_idx );

                        if(!overhang_lines.empty())
                        {
                            spdlog::debug("Compensated for overhang area that had no valid tips. Now has a tip.");
                        }
                        else
                        {
                            spdlog::warn("Overhang area has no valid tips! Was roof: {} Was Cradle {} On Layer: {}", roof_allowed_for_this_part, supports_cradle ,layer_idx);
                        }
                    }

                    size_t dont_move_for_layers = support_roof_layers ? (force_tip_to_roof ? support_roof_layers : (roof_allowed_for_this_part ? 0 : support_roof_layers)) : 0;
                    addLinesAsInfluenceAreas(new_tips, overhang_lines, force_tip_to_roof ? support_roof_layers : 0, layer_idx, roof_allowed_for_this_part, supports_cradle,dont_move_for_layers, only_lines);
                }
            }
        );

    cura::parallel_for<coord_t>
    (
        0,
        support_roof_drawn.size(),
        [&](const LayerIndex layer_idx)
        {
            // Sometimes roofs could be empty as the pattern does not generate lines if the area is narrow enough.
            // If there is a roof could have zero lines in its area (as it has no wall), rand a support area would very likely be printed (because there are walls for the support areas), replace non printable roofs with support
            if (! use_fake_roof && config.support_wall_count > 0 && config.support_roof_wall_count == 0)
            {
                for (auto roof_area : support_roof_drawn[layer_idx].unionPolygons(roof_tips_drawn[layer_idx]).splitIntoParts())
                {
                    // technically there is no guarantee that a drawn roof tip has lines, as it could be unioned with another roof area that has, but this has to be enough hopefully.
                    if (layer_idx < additional_support_areas.size() && TreeSupportUtils::generateSupportInfillLines(roof_area, config, true, layer_idx, support_roof_line_distance, cross_fill_provider, false).empty())
                    {
                        additional_support_areas[layer_idx].add(roof_area);
                    }
                    else
                    {
                        storage.support.supportLayers[layer_idx].support_roof.add(roof_area);
                    }
                }

                additional_support_areas[layer_idx] = additional_support_areas[layer_idx].unionPolygons();
                storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.unionPolygons();
            }
            else
            {
                if (use_fake_roof)
                {
                    for (auto part : support_roof_drawn[layer_idx].splitIntoParts())
                    {
                        storage.support.supportLayers[layer_idx].support_infill_parts.emplace_back(part, config.support_line_width, 0, support_roof_line_distance);
                    }
                    placed_support_lines_support_areas[layer_idx].add(
                        TreeSupportUtils::generateSupportInfillLines(support_roof_drawn[layer_idx], config, false, layer_idx, support_roof_line_distance, cross_fill_provider, false).offsetPolyLine(config.support_line_width / 2));
                }
                else
                {
                    storage.support.supportLayers[layer_idx].support_roof.add(support_roof_drawn[layer_idx]);
                    storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.unionPolygons(roof_tips_drawn[layer_idx]);
                }
            }
        });

    removeUselessAddedPoints(new_tips ,storage , additional_support_areas);

    if(support_roof_layers && !use_fake_roof && cradle_lines_roof)
    {
        for (auto [layer_idx, cradle] : cradle_areas | ranges::views::enumerate)
        {
            storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.unionPolygons(cradle);
        }
    }
    else
    {
        for (auto [layer_idx, cradle] : cradle_areas | ranges::views::enumerate)
        {
            additional_support_areas[layer_idx] = additional_support_areas[layer_idx].unionPolygons(cradle);
        }
    }


    for (auto [layer_idx, tips_on_layer] : new_tips | ranges::views::enumerate)
    {
        move_bounds[layer_idx].insert(tips_on_layer.begin(), tips_on_layer.end());
    }

}


}// namespace cura

