// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TreeSupport.h"

#include <chrono>
#include <fstream>
#include <optional>
#include <stdio.h>
#include <string>
#include <thread>

#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/reverse.hpp>
#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "TreeSupportTipGenerator.h"
#include "TreeSupportUtils.h"
#include "infill.h"
#include "infill/SierpinskiFillProvider.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h"
#include "support.h" //For precomputeCrossInfillTree
#include "utils/Simplify.h"
#include "utils/ThreadPool.h"
#include "utils/algorithm.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/polygonUtils.h" //For moveInside.
#include "utils/section_type.h"

namespace cura
{

TreeSupport::TreeSupport(const SliceDataStorage& storage)
{
    size_t largest_printed_mesh_idx = 0;

    for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        const auto& mesh = *mesh_ptr;
        TreeSupportSettings::some_model_contains_thick_roof |= mesh.settings.get<coord_t>("support_roof_height") >= 2 * mesh.settings.get<coord_t>("layer_height");
        TreeSupportSettings::has_to_rely_on_min_xy_dist_only |= mesh.settings.get<coord_t>("support_top_distance") == 0
                                                             || mesh.settings.get<coord_t>("support_bottom_distance") == 0
                                                             || mesh.settings.get<coord_t>("min_feature_size") < (FUDGE_LENGTH * 2);
    }

    // Group all meshes that can be processed together. NOTE this is different from mesh-groups!
    // Only one setting object is needed per group, as different settings in the same group may only occur in the tip, which uses the original settings objects from the meshes.
    for (auto [mesh_idx, mesh_ptr] : storage.meshes | ranges::views::enumerate)
    {
        SliceMeshStorage& mesh = *mesh_ptr;
        const bool non_supportable_mesh = mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh") || mesh.settings.get<bool>("support_mesh");
        if (mesh.settings.get<ESupportStructure>("support_structure") != ESupportStructure::TREE || ! mesh.settings.get<bool>("support_enable") || non_supportable_mesh)
        {
            continue;
        }

        bool added = false;

        TreeSupportSettings next_settings(mesh.settings);

        for (auto& grouped_mesh : grouped_meshes)
        {
            if (next_settings == grouped_mesh.first)
            {
                added = true;
                grouped_mesh.second.emplace_back(mesh_idx);
                // Handle some settings that are only used for performance reasons. This ensures that a horrible set setting intended to improve performance can not reduce it
                // drastically.
                grouped_mesh.first.performance_interface_skip_layers
                    = std::min(grouped_mesh.first.performance_interface_skip_layers, next_settings.performance_interface_skip_layers);
            }
        }
        if (! added)
        {
            grouped_meshes.emplace_back(next_settings, std::vector<size_t>{ mesh_idx });
        }

        // no need to do this per mesh group as adaptive layers and raft setting are not setable per mesh.
        if (storage.meshes[largest_printed_mesh_idx]->layers.back().printZ < mesh.layers.back().printZ)
        {
            largest_printed_mesh_idx = mesh_idx;
        }
    }
    std::vector<coord_t> known_z(storage.meshes[largest_printed_mesh_idx]->layers.size());

    for (auto [z, layer] : ranges::views::enumerate(storage.meshes[largest_printed_mesh_idx]->layers))
    {
        known_z[z] = layer.printZ;
    }

    for (auto& mesh : grouped_meshes)
    {
        mesh.first.setActualZ(known_z);
    }

    placed_support_lines_support_areas = std::vector<Polygons>(storage.support.supportLayers.size(), Polygons());
}

void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    if (grouped_meshes.empty())
    {
        return;
    }

    if (storage.support.cross_fill_provider == nullptr)
    {
        AreaSupport::precomputeCrossInfillTree(storage);
    }

    // Process every mesh group. These groups can not be processed parallel, as the processing in each group is parallelized, and nested parallelization is disables and slow.
    for (auto [counter, processing] : grouped_meshes | ranges::views::enumerate)
    {
        // process each combination of meshes
        std::vector<std::set<TreeSupportElement*>> move_bounds(
            storage.support.supportLayers
                .size()); // Value is the area where support may be placed. As this is calculated in CreateLayerPathing it is saved and reused in drawAreas.

        additional_required_support_area = std::vector<Polygons>(storage.support.supportLayers.size(), Polygons());


        spdlog::info("Processing support tree mesh group {} of {} containing {} meshes.", counter + 1, grouped_meshes.size(), grouped_meshes[counter].second.size());
        std::vector<Polygons> exclude(storage.support.supportLayers.size());
        auto t_start = std::chrono::high_resolution_clock::now();

        // get all already existing support areas and exclude them
        cura::parallel_for<coord_t>(
            LayerIndex(0),
            LayerIndex(storage.support.supportLayers.size()),
            [&](const LayerIndex layer_idx)
            {
                Polygons exlude_at_layer;
                exlude_at_layer.add(storage.support.supportLayers[layer_idx].support_bottom);
                exlude_at_layer.add(storage.support.supportLayers[layer_idx].support_roof);
                for (auto part : storage.support.supportLayers[layer_idx].support_infill_parts)
                {
                    exlude_at_layer.add(part.outline);
                }
                exclude[layer_idx] = exlude_at_layer.unionPolygons();
                scripta::log("tree_support_exclude", exclude[layer_idx], SectionType::SUPPORT, layer_idx);
            });
        config = processing.first; // This struct is used to easy retrieve setting. No other function except those in TreeModelVolumes and generateInitialAreas have knowledge of
                                   // the existence of multiple meshes being processed.
        progress_multiplier = 1.0 / double(grouped_meshes.size());
        progress_offset = counter == 0 ? 0 : TREE_PROGRESS_TOTAL * (double(counter) * progress_multiplier);
        volumes_ = TreeModelVolumes(
            storage,
            config.maximum_move_distance,
            config.maximum_move_distance_slow,
            config.support_line_width / 2,
            processing.second.front(),
            progress_multiplier,
            progress_offset,
            exclude);

        // ### Precalculate avoidances, collision etc.
        precalculate(storage, processing.second);
        const auto t_precalc = std::chrono::high_resolution_clock::now();

        // ### Place tips of the support tree
        for (size_t mesh_idx : processing.second)
        {
            generateInitialAreas(*storage.meshes[mesh_idx], move_bounds, storage);
        }
        const auto t_gen = std::chrono::high_resolution_clock::now();

        // ### Propagate the influence areas downwards.
        createLayerPathing(move_bounds);
        const auto t_path = std::chrono::high_resolution_clock::now();

        // ### Set a point in each influence area
        createNodesFromArea(move_bounds);
        const auto t_place = std::chrono::high_resolution_clock::now();

        // ### draw these points as circles
        drawAreas(move_bounds, storage);

        const auto t_draw = std::chrono::high_resolution_clock::now();
        const auto dur_pre_gen = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_precalc - t_start).count();
        const auto dur_gen = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_gen - t_precalc).count();
        const auto dur_path = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_path - t_gen).count();
        const auto dur_place = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_place - t_path).count();
        const auto dur_draw = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_draw - t_place).count();
        const auto dur_total = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_draw - t_start).count();
        spdlog::info(
            "Total time used creating Tree support for the currently grouped meshes: {} ms. Different subtasks:\n"
            "Calculating Avoidance: {} ms Creating inital influence areas: {} ms Influence area creation: {} ms Placement of Points in InfluenceAreas: {} ms Drawing result as "
            "support {} ms",
            dur_total,
            dur_pre_gen,
            dur_gen,
            dur_path,
            dur_place,
            dur_draw);


        for (auto& layer : move_bounds)
        {
            for (auto elem : layer)
            {
                delete elem->area;
                delete elem;
            }
        }
    }

    storage.support.generated = true;
}

void TreeSupport::precalculate(const SliceDataStorage& storage, std::vector<size_t> currently_processing_meshes)
{
    // Calculate top most layer that is relevant for support.
    LayerIndex max_layer = 0;
    for (size_t mesh_idx : currently_processing_meshes)
    {
        const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
        const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
        const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
        const size_t z_distance_top_layers = round_up_divide(z_distance_top,
                                                             layer_height) + 1; // Support must always be 1 layer below overhang.
        if (mesh.overhang_areas.size() <= z_distance_top_layers)
        {
            continue;
        }

        for (const auto layer_idx : ranges::views::iota(1UL, mesh.overhang_areas.size() - z_distance_top_layers) | ranges::views::reverse)
        {
            // Look for max relevant layer.
            const Polygons& overhang = mesh.overhang_areas[layer_idx + z_distance_top_layers];
            if (! overhang.empty())
            {
                if (layer_idx > max_layer) // iterates over multiple meshes
                {
                    max_layer = 1 + layer_idx; // plus one to avoid problems if something is of by one
                }
                break;
            }
        }
    }

    // ### The actual precalculation happens in TreeModelVolumes.
    volumes_.precalculate(max_layer);
}


void TreeSupport::generateInitialAreas(const SliceMeshStorage& mesh, std::vector<std::set<TreeSupportElement*>>& move_bounds, SliceDataStorage& storage)
{
    TreeSupportTipGenerator tip_gen(storage, mesh, volumes_);
    tip_gen.generateTips(storage, mesh, move_bounds, additional_required_support_area, placed_support_lines_support_areas);
}

void TreeSupport::mergeHelper(
    std::map<TreeSupportElement, AABB>& reduced_aabb,
    std::map<TreeSupportElement, AABB>& input_aabb,
    const PropertyAreasUnordered& to_bp_areas,
    const PropertyAreas& to_model_areas,
    const PropertyAreas& influence_areas,
    PropertyAreasUnordered& insert_bp_areas,
    PropertyAreasUnordered& insert_model_areas,
    PropertyAreasUnordered& insert_influence,
    std::vector<TreeSupportElement>& erase,
    const LayerIndex layer_idx)
{
    const bool first_merge_iteration = reduced_aabb.empty(); // If this is the first iteration, all elements in input have to be merged with each other
    const std::function<coord_t(size_t, double)> getRadiusFunction = [&](const size_t distance_to_top, const double buildplate_radius_increases)
    {
        return config.getRadius(distance_to_top, buildplate_radius_increases);
    };
    for (auto& influence : input_aabb)
    {
        bool merged = false;
        AABB influence_aabb = influence.second;
        for (const auto& reduced_check : reduced_aabb)
        {
            // As every area has to be checked for overlaps with other areas, some fast heuristic is needed to abort early if clearly possible
            // This is so performance critical that using a map lookup instead of the direct access of the cached AABBs can have a surprisingly large performance impact
            AABB aabb = reduced_check.second;
            if (aabb.hit(influence_aabb))
            {
                if (! first_merge_iteration && input_aabb.count(reduced_check.first))
                {
                    break; // Do not try to merge elements that already should have been merged. Done for potential performance improvement.
                }

                const bool merging_gracious_and_non_gracious = reduced_check.first.to_model_gracious != influence.first.to_model_gracious;
                // ^^^ We do not want to merge a gracious with a non gracious area as bad placement could negatively impact the dependability of the whole subtree.
                const bool merging_to_bp = reduced_check.first.to_buildplate && influence.first.to_buildplate;
                const bool merging_min_and_regular_xy = reduced_check.first.use_min_xy_dist != influence.first.use_min_xy_dist;
                // ^^^ Could cause some issues with the increase of one area, as it is assumed that if the smaller is increased by the delta to the larger it is engulfed by it
                // already.
                //     But because a different collision may be removed from the in drawArea generated circles, this assumption could be wrong.
                const bool merging_different_range_limits = reduced_check.first.influence_area_limit_active && influence.first.influence_area_limit_active
                                                         && influence.first.influence_area_limit_range != reduced_check.first.influence_area_limit_range;
                coord_t increased_to_model_radius = 0;
                size_t larger_to_model_dtt = 0;

                if (! merging_to_bp)
                {
                    const coord_t infl_radius = config.getRadius(influence.first); // Get the real radius increase as the user does not care for the collision model.
                    const coord_t redu_radius = config.getRadius(reduced_check.first);
                    if (reduced_check.first.to_buildplate != influence.first.to_buildplate)
                    {
                        if (reduced_check.first.to_buildplate)
                        {
                            if (infl_radius < redu_radius)
                            {
                                increased_to_model_radius = influence.first.increased_to_model_radius + redu_radius - infl_radius;
                            }
                        }
                        else
                        {
                            if (infl_radius > redu_radius)
                            {
                                increased_to_model_radius = reduced_check.first.increased_to_model_radius + infl_radius - redu_radius;
                            }
                        }
                    }
                    larger_to_model_dtt = std::max(influence.first.distance_to_top, reduced_check.first.distance_to_top);
                }

                // If a merge could place a stable branch on unstable ground, would be increasing the radius further than allowed to when merging to model and to_bp trees or
                //   would merge to model before it is known they will even been drawn the merge is skipped
                if (merging_min_and_regular_xy || merging_gracious_and_non_gracious || increased_to_model_radius > config.max_to_model_radius_increase
                    || (! merging_to_bp && larger_to_model_dtt < config.min_dtt_to_model && ! reduced_check.first.supports_roof && ! influence.first.supports_roof)
                    || merging_different_range_limits)
                {
                    continue;
                }

                Polygons relevant_infl;
                Polygons relevant_redu;
                if (merging_to_bp)
                {
                    relevant_infl = to_bp_areas.count(influence.first) ? to_bp_areas.at(influence.first)
                                                                       : Polygons(); // influence.first is a new element => not required to check if it was changed
                    relevant_redu = insert_bp_areas.count(reduced_check.first) ? insert_bp_areas[reduced_check.first]
                                                                               : (to_bp_areas.count(reduced_check.first) ? to_bp_areas.at(reduced_check.first) : Polygons());
                }
                else
                {
                    relevant_infl = to_model_areas.count(influence.first) ? to_model_areas.at(influence.first) : Polygons();
                    relevant_redu = insert_model_areas.count(reduced_check.first)
                                      ? insert_model_areas[reduced_check.first]
                                      : (to_model_areas.count(reduced_check.first) ? to_model_areas.at(reduced_check.first) : Polygons());
                }

                const bool red_bigger = config.getCollisionRadius(reduced_check.first) > config.getCollisionRadius(influence.first);
                std::pair<TreeSupportElement, Polygons> smaller_rad = red_bigger ? std::pair<TreeSupportElement, Polygons>(influence.first, relevant_infl)
                                                                                 : std::pair<TreeSupportElement, Polygons>(reduced_check.first, relevant_redu);
                std::pair<TreeSupportElement, Polygons> bigger_rad = red_bigger ? std::pair<TreeSupportElement, Polygons>(reduced_check.first, relevant_redu)
                                                                                : std::pair<TreeSupportElement, Polygons>(influence.first, relevant_infl);
                const coord_t real_radius_delta = std::abs(config.getRadius(bigger_rad.first) - config.getRadius(smaller_rad.first));
                const coord_t smaller_collision_radius = config.getCollisionRadius(smaller_rad.first);

                // the area of the bigger radius is used to ensure correct placement regarding the relevant avoidance, so if that would change an invalid area may be created
                if (! bigger_rad.first.can_use_safe_radius && smaller_rad.first.can_use_safe_radius)
                {
                    continue;
                }

                // The bigger radius is used to verify that the area is still valid after the increase with the delta.
                // If there were a point where the big influence area could be valid with can_use_safe_radius the element would already be can_use_safe_radius.
                // The smaller radius, which gets increased by delta may reach into the area where use_min_xy_dist is no longer required.
                bool use_min_radius = bigger_rad.first.use_min_xy_dist && smaller_rad.first.use_min_xy_dist;

                // The idea is that the influence area with the smaller collision radius is increased by the radius difference.
                // If this area has any intersections with the influence area of the larger collision radius,
                //   a branch (of the larger collision radius) placed in this intersection, has already engulfed the branch of the smaller collision radius.
                // Because of this a merge may happen even if the influence areas (that represent possible center points of branches) do not intersect yet.
                // Remember that collision radius <= real radius as otherwise this assumption would be false.
                const Polygons small_rad_increased_by_big_minus_small = TreeSupportUtils::safeOffsetInc(
                    smaller_rad.second,
                    real_radius_delta,
                    volumes_.getCollision(smaller_collision_radius, layer_idx - 1, use_min_radius),
                    2 * (config.xy_distance + smaller_collision_radius - EPSILON), // Epsilon avoids possible rounding errors
                    0,
                    0,
                    config.support_line_distance / 2,
                    &config.simplifier);
                Polygons intersect = small_rad_increased_by_big_minus_small.intersection(bigger_rad.second);

                if (intersect.area()
                    > 1) // dont use empty as a line is not empty, but for this use-case it very well may be (and would be one layer down as union does not keep lines)
                {
                    // Check if the overlap is large enough (Small ares tend to attract rounding errors in clipper). While 25 was guessed as enough, i did not have reason to change
                    // it.
                    if (intersect.offset(-FUDGE_LENGTH / 2).area() <= 1)
                    {
                        continue;
                    }

                    // Do the actual merge now that the branches are confirmed to be able to intersect.

                    // Calculate which point is closest to the point of the last merge (or tip center if no merge above it has happened)
                    // Used at the end to estimate where to best place the branch on the bottom most layer
                    // Could be replaced with a random point inside the new area
                    Point new_pos = reduced_check.first.next_position;
                    if (! intersect.inside(new_pos, true))
                    {
                        PolygonUtils::moveInside(intersect, new_pos);
                    }

                    if (increased_to_model_radius == 0)
                    {
                        increased_to_model_radius = std::max(reduced_check.first.increased_to_model_radius, influence.first.increased_to_model_radius);
                    }

                    const TreeSupportElement key(
                        reduced_check.first,
                        influence.first,
                        layer_idx - 1,
                        new_pos,
                        increased_to_model_radius,
                        getRadiusFunction,
                        config.diameter_scale_bp_radius,
                        config.branch_radius,
                        config.diameter_angle_scale_factor);

                    const auto getIntersectInfluence = [&](const PropertyAreasUnordered& insert_infl, const PropertyAreas& infl_areas)
                    {
                        const Polygons infl_small = insert_infl.count(smaller_rad.first) ? insert_infl.at(smaller_rad.first)
                                                                                         : (infl_areas.count(smaller_rad.first) ? infl_areas.at(smaller_rad.first) : Polygons());
                        const Polygons infl_big = insert_infl.count(bigger_rad.first) ? insert_infl.at(bigger_rad.first)
                                                                                      : (infl_areas.count(bigger_rad.first) ? infl_areas.at(bigger_rad.first) : Polygons());
                        const Polygons small_rad_increased_by_big_minus_small_infl = TreeSupportUtils::safeOffsetInc(
                            infl_small,
                            real_radius_delta,
                            volumes_.getCollision(smaller_collision_radius, layer_idx - 1, use_min_radius),
                            2 * (config.xy_distance + smaller_collision_radius - EPSILON),
                            0,
                            0,
                            config.support_line_distance / 2,
                            &config.simplifier);
                        return small_rad_increased_by_big_minus_small_infl.intersection(
                            infl_big); // If the one with the bigger radius with the lower radius removed overlaps we can merge.
                    };

                    Polygons intersect_influence;
                    intersect_influence
                        = TreeSupportUtils::safeUnion(intersect, getIntersectInfluence(insert_influence, influence_areas)); // Rounding errors again. Do not ask me where or why.

                    Polygons intersect_to_model;
                    if (merging_to_bp && config.support_rests_on_model)
                    {
                        intersect_to_model = getIntersectInfluence(insert_model_areas, to_model_areas);
                        intersect_influence = TreeSupportUtils::safeUnion(intersect_influence, intersect_to_model); // Still rounding errors.
                    }

                    // Remove the now merged elements from all buckets, as they do not exist anymore in their old form.
                    insert_bp_areas.erase(reduced_check.first);
                    insert_bp_areas.erase(influence.first);
                    insert_model_areas.erase(reduced_check.first);
                    insert_model_areas.erase(influence.first);
                    insert_influence.erase(reduced_check.first);
                    insert_influence.erase(influence.first);

                    (merging_to_bp ? insert_bp_areas : insert_model_areas).emplace(key, intersect);
                    if (merging_to_bp && config.support_rests_on_model)
                    {
                        insert_model_areas.emplace(key, intersect_to_model);
                    }
                    insert_influence.emplace(key, intersect_influence);

                    erase.emplace_back(reduced_check.first);
                    erase.emplace_back(influence.first);
                    const Polygons merge
                        = intersect.unionPolygons(intersect_to_model).offset(config.getRadius(key), ClipperLib::jtRound).difference(volumes_.getCollision(0, layer_idx - 1));
                    // ^^^ Regular union should be preferable here as Polygons tend to only become smaller through rounding errors (smaller!=has smaller area as holes have a
                    // negative area.).
                    //     And if this area disappears because of rounding errors, the only downside is that it can not merge again on this layer.

                    reduced_aabb.erase(reduced_check.first); // This invalidates reduced_check.
                    reduced_aabb.emplace(key, AABB(merge));

                    merged = true;
                    break;
                }
            }
        }

        if (! merged)
        {
            reduced_aabb[influence.first] = influence_aabb;
        }
    }
}

void TreeSupport::mergeInfluenceAreas(PropertyAreasUnordered& to_bp_areas, PropertyAreas& to_model_areas, PropertyAreas& influence_areas, LayerIndex layer_idx)
{
    /*
     * Idea behind this is that the calculation of merges can be accelerated a bit using divide and conquer:
     * If two groups of areas are already merged, only all elements in group 2 have to be merged into group one.
     * This can only accelerate by factor 2 (as half the work is merging the last two groups).
     * The actual merge logic is found in mergeHelper. This function only manages parallelization of different mergeHelper calls.
     */

    const size_t input_size = influence_areas.size();
    size_t num_threads = std::max(size_t(1), size_t(std::thread::hardware_concurrency())); // For some reason hardware concurrency can return 0;

    if (input_size == 0)
    {
        return;
    }
    constexpr int min_elements_per_bucket = 2;

    // max_bucket_count is input_size/min_elements_per_bucket round down to the next 2^n.
    // The rounding to 2^n is to ensure improved performance, as every iteration two buckets will be merged, halving the amount of buckets.
    // If halving would cause an uneven count, e.g. 3 Then bucket 0 and 1 would have to be merged, and in the next iteration the last remaining buckets. This is assumed to not be
    // optimal performance-wise.
    const size_t max_bucket_count = std::pow(2, std::floor(std::log(round_up_divide(input_size, min_elements_per_bucket))));
    int bucket_count = std::min(max_bucket_count, num_threads); // do not use more buckets than available threads.

    // To achieve that every element in a bucket is already correctly merged with other elements in this bucket
    // an extra empty bucket is created for each bucket, and the elements are merged into the empty one.
    // Each thread will then process two buckets by merging all elements in the second bucket into the first one as mergeHelper will disable not trying to merge elements from the
    // same bucket in this case.
    std::vector<PropertyAreas> buckets_area(2 * bucket_count);
    std::vector<std::map<TreeSupportElement, AABB>> buckets_aabb(2 * bucket_count);


    size_t position = 0, counter = 0;
    const size_t over_elements = input_size % bucket_count;
    const size_t elements_per_step = input_size / bucket_count;

    // Dplit the data in x parts to be able to divide and conquer.
    // The first "over_elements" of buckets gets elements_per_step+1 elements
    for (const auto& influence_area : influence_areas)
    {
        buckets_area[position * 2 + 1].emplace(influence_area.first, influence_area.second);
        // ^^^ Only use every second bucket beginning with 1 as this makes the parallel call later easier as we assume everything in a bucket i%2==0 is already processed.
        counter++;
        if ((counter == elements_per_step && position >= over_elements) || counter > elements_per_step)
        {
            position++;
            counter = 0;
        }
    }

    // Precalculate the AABBs from the influence areas.
    cura::parallel_for<size_t>(
        0,
        buckets_area.size() / 2,
        [&](size_t idx) // +=2 as in the beginning only uneven buckets will be filled
        {
            idx = idx * 2 + 1; // this is eqivalent to a parallel for(size_t idx=1;idx<buckets_area.size(),idx+=2)
            for (const std::pair<TreeSupportElement, Polygons>& input_pair : buckets_area[idx])
            {
                AABB outer_support_wall_aabb = AABB(input_pair.second);
                outer_support_wall_aabb.expand(config.getRadius(input_pair.first));
                buckets_aabb[idx].emplace(input_pair.first, outer_support_wall_aabb);
            }
        });

    while (buckets_area.size() > 1)
    {
        // Some temporary storage, of elements that have to be inserted or removed from the background storage. Only one per two buckets required
        std::vector<PropertyAreasUnordered> insert_main(buckets_area.size() / 2);
        std::vector<PropertyAreasUnordered> insert_secondary(buckets_area.size() / 2);
        std::vector<PropertyAreasUnordered> insert_influence(buckets_area.size() / 2);
        std::vector<std::vector<TreeSupportElement>> erase(buckets_area.size() / 2);

        cura::parallel_for<size_t>(
            0,
            (coord_t)buckets_area.size() / 2,
            [&](size_t bucket_pair_idx)
            {
                bucket_pair_idx *= 2; // this is eqivalent to a parallel for(size_t idx=0;idx<buckets_area.size()-1,idx+=2)
                // Merge bucket_count adjacent to each other, merging uneven bucket numbers into even buckets
                mergeHelper(
                    buckets_aabb[bucket_pair_idx],
                    buckets_aabb[bucket_pair_idx + 1],
                    to_bp_areas,
                    to_model_areas,
                    influence_areas,
                    insert_main[bucket_pair_idx / 2],
                    insert_secondary[bucket_pair_idx / 2],
                    insert_influence[bucket_pair_idx / 2],
                    erase[bucket_pair_idx / 2],
                    layer_idx);
                buckets_area[bucket_pair_idx + 1].clear(); // clear now irrelevant max_bucket_count, and delete them later
                buckets_aabb[bucket_pair_idx + 1].clear();
            });

        // Note the division in the limit of the loop!
        for (const coord_t i : ranges::views::iota(0UL, buckets_area.size() / 2))
        {
            for (TreeSupportElement& del : erase[i])
            {
                to_bp_areas.erase(del);
                to_model_areas.erase(del);
                influence_areas.erase(del);
            }

            for (const std::pair<TreeSupportElement, Polygons>& tup : insert_main[i])
            {
                to_bp_areas.emplace(tup);
            }

            for (const std::pair<TreeSupportElement, Polygons>& tup : insert_secondary[i])
            {
                to_model_areas.emplace(tup);
            }
            for (const std::pair<TreeSupportElement, Polygons>& tup : insert_influence[i])
            {
                influence_areas.emplace(tup);
            }
        }

        const auto position_rem = std::remove_if(
            buckets_area.begin(),
            buckets_area.end(),
            [&](const PropertyAreas x) mutable
            {
                return x.empty();
            });
        buckets_area.erase(position_rem, buckets_area.end());

        const auto position_aabb = std::remove_if(
            buckets_aabb.begin(),
            buckets_aabb.end(),
            [&](const std::map<TreeSupportElement, AABB> x) mutable
            {
                return x.empty();
            });
        buckets_aabb.erase(position_aabb, buckets_aabb.end());
    }
}

std::optional<TreeSupportElement> TreeSupport::increaseSingleArea(
    AreaIncreaseSettings settings,
    LayerIndex layer_idx,
    TreeSupportElement* parent,
    const Polygons& relevant_offset,
    Polygons& to_bp_data,
    Polygons& to_model_data,
    Polygons& increased,
    const coord_t overspeed,
    const bool mergelayer)
{
    TreeSupportElement current_elem(parent); // Also increases DTT by one.
    Polygons check_layer_data;
    if (settings.increase_radius)
    {
        current_elem.effective_radius_height += 1;
    }
    coord_t radius = config.getCollisionRadius(current_elem);

    if (settings.move)
    {
        increased = relevant_offset;
        if (overspeed > 0)
        {
            const coord_t safe_movement_distance = (current_elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance)
                                                 + (std::min(config.z_distance_top_layers, config.z_distance_bottom_layers) > 0 ? config.min_feature_size : 0);
            // The difference to ensure that the result not only conforms to wall_restriction, but collision/avoidance is done later.
            // The higher last_safe_step_movement_distance comes exactly from the fact that the collision will be subtracted later.
            increased = TreeSupportUtils::safeOffsetInc(
                increased,
                overspeed,
                volumes_.getWallRestriction(config.getCollisionRadius(*parent), layer_idx, parent->use_min_xy_dist),
                safe_movement_distance,
                safe_movement_distance + radius,
                1,
                config.support_line_distance / 2,
                nullptr);
        }
        if (settings.no_error && settings.move)
        {
            increased = config.simplifier.polygon(increased); // as ClipperLib::jtRound has to be used for offsets this simplify is VERY important for performance.
        }
    }
    else // if no movement is done the areas keep parent area as no move == offset(0)
    {
        increased = *parent->area;
    }

    if ((mergelayer || current_elem.to_buildplate) && config.support_rest_preference == RestPreference::BUILDPLATE)
    {
        to_bp_data = TreeSupportUtils::safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, false, settings.use_min_distance)));
        if (! current_elem.to_buildplate && to_bp_data.area() > 1) // mostly happening in the tip, but with merges one should check every time, just to be sure.
        {
            current_elem.to_buildplate = true; // sometimes nodes that can reach the buildplate are marked as cant reach, tainting subtrees. This corrects it.
            spdlog::debug("Corrected taint leading to a wrong to model value on layer {} targeting {} with radius {}", layer_idx - 1, current_elem.target_height, radius);
        }
    }
    if (config.support_rests_on_model)
    {
        if (mergelayer || current_elem.to_model_gracious)
        {
            to_model_data = TreeSupportUtils::safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, true, settings.use_min_distance)));
        }

        if (! current_elem.to_model_gracious)
        {
            if (mergelayer && to_model_data.area() >= 1)
            {
                current_elem.to_model_gracious = true;
                spdlog::debug("Corrected taint leading to a wrong non gracious value on layer {} targeting {} with radius {}", layer_idx - 1, current_elem.target_height, radius);
            }
            else
            {
                to_model_data
                    = TreeSupportUtils::safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, AvoidanceType::COLLISION, true, settings.use_min_distance)));
            }
        }
    }

    check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;

    if (settings.increase_radius && check_layer_data.area() > 1)
    {
        std::function<bool(coord_t)> validWithRadius = [&](coord_t next_radius)
        {
            if (volumes_.ceilRadius(next_radius, settings.use_min_distance) <= volumes_.ceilRadius(radius, settings.use_min_distance))
            {
                return true;
            }

            Polygons to_bp_data_2;
            if (current_elem.to_buildplate)
            {
                // Regular union as output will not be used later => this area should always be a subset of the safeUnion one.
                to_bp_data_2 = increased.difference(volumes_.getAvoidance(next_radius, layer_idx - 1, settings.type, false, settings.use_min_distance)).unionPolygons();
            }
            Polygons to_model_data_2;
            if (config.support_rests_on_model && ! current_elem.to_buildplate)
            {
                to_model_data_2 = increased
                                      .difference(volumes_.getAvoidance(
                                          next_radius,
                                          layer_idx - 1,
                                          current_elem.to_model_gracious ? settings.type : AvoidanceType::COLLISION,
                                          true,
                                          settings.use_min_distance))
                                      .unionPolygons();
            }
            Polygons check_layer_data_2 = current_elem.to_buildplate ? to_bp_data_2 : to_model_data_2;

            return check_layer_data_2.area() > 1;
        };
        coord_t ceil_radius_before = volumes_.ceilRadius(radius, settings.use_min_distance);

        // If the Collision Radius is smaller than the actual radius, check if it can catch up without violating the avoidance.
        if (config.getCollisionRadius(current_elem) < config.increase_radius_until_radius && config.getCollisionRadius(current_elem) < config.getRadius(current_elem))
        {
            coord_t target_radius = std::min(config.getRadius(current_elem), config.increase_radius_until_radius);
            coord_t current_ceil_radius = volumes_.getRadiusNextCeil(radius, settings.use_min_distance);

            while (current_ceil_radius < target_radius && validWithRadius(volumes_.getRadiusNextCeil(current_ceil_radius + 1, settings.use_min_distance)))
            {
                current_ceil_radius = volumes_.getRadiusNextCeil(current_ceil_radius + 1, settings.use_min_distance);
            }
            size_t resulting_eff_dtt = current_elem.effective_radius_height;
            while (resulting_eff_dtt + 1 < current_elem.distance_to_top && config.getRadius(resulting_eff_dtt + 1, current_elem.buildplate_radius_increases) <= current_ceil_radius
                   && config.getRadius(resulting_eff_dtt + 1, current_elem.buildplate_radius_increases) <= config.getRadius(current_elem))
            {
                resulting_eff_dtt++;
            }
            current_elem.effective_radius_height = resulting_eff_dtt;

            // If catchup is not possible, it is likely that there is a hole below. Assuming the branches are in some kind of bowl, the branches should still stay away from the
            // wall of the bowl if possible.
            if (config.getCollisionRadius(current_elem) < config.increase_radius_until_radius && config.getCollisionRadius(current_elem) < config.getRadius(current_elem))
            {
                Polygons new_to_bp_data;
                Polygons new_to_model_data;

                if (current_elem.to_buildplate)
                {
                    new_to_bp_data = to_bp_data.difference(volumes_.getCollision(config.getRadius(current_elem), layer_idx - 1, current_elem.use_min_xy_dist));
                    if (new_to_bp_data.area() > EPSILON)
                    {
                        to_bp_data = new_to_bp_data;
                    }
                }
                if (config.support_rests_on_model && (! current_elem.to_buildplate || mergelayer))
                {
                    new_to_model_data = to_model_data.difference(volumes_.getCollision(config.getRadius(current_elem), layer_idx - 1, current_elem.use_min_xy_dist));
                    if (new_to_model_data.area() > EPSILON)
                    {
                        to_model_data = new_to_model_data;
                    }
                }
            }
        }
        radius = config.getCollisionRadius(current_elem);

        const coord_t foot_radius_increase = config.branch_radius * (std::max(config.diameter_scale_bp_radius - config.diameter_angle_scale_factor, 0.0));
        const double planned_foot_increase = std::min(1.0, double(config.recommendedMinRadius(layer_idx - 1) - config.getRadius(current_elem)) / foot_radius_increase);
        // ^^^ Is nearly all of the time 1, but sometimes an increase of 1 could cause the radius to become bigger than recommendedMinRadius, which could cause the radius to become
        // bigger than precalculated.

        // If the support_rest_preference is GRACEFUL, increase buildplate_radius_increases anyway. This does ONLY affect the CollisionRadius, as the regular radius only includes
        // the buildplate_radius_increases when the SupportElement is to_buildplate (which it can not be when support_rest_preference is GRACEFUL). If the branch later rests on the
        // buildplate the to_buildplate flag will only need to be updated to ensure that the radius is also correctly increased. Downside is that the enlargement of the
        // CollisionRadius can cause branches, that could rest on the model if the radius was not increased, to instead rest on the buildplate. A better way could be changing
        // avoidance to model to not include the buildplate and then calculate avoidances by combining the to model avoidance without the radius increase with the to buildplate
        // avoidance with the larger radius. This would require ensuring all requests for the avoidance would have to ensure that the correct hybrid avoidance is requested (which
        // would only be relevant when support_rest_preference is GRACEFUL) Also unioning areas when an avoidance is requested may also have a relevant performance impact, so there
        // can be an argument made that the current workaround is preferable.
        const bool increase_bp_foot
            = planned_foot_increase > 0 && (current_elem.to_buildplate || (current_elem.to_model_gracious && config.support_rest_preference == RestPreference::GRACEFUL));


        if (increase_bp_foot && config.getRadius(current_elem) >= config.branch_radius && config.getRadius(current_elem) >= config.increase_radius_until_radius)
        {
            if (validWithRadius(config.getRadius(current_elem.effective_radius_height, current_elem.buildplate_radius_increases + planned_foot_increase)))
            {
                current_elem.buildplate_radius_increases += planned_foot_increase;
                radius = config.getCollisionRadius(current_elem);
            }
        }

        if (ceil_radius_before != volumes_.ceilRadius(radius, settings.use_min_distance))
        {
            if (current_elem.to_buildplate)
            {
                to_bp_data = TreeSupportUtils::safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, false, settings.use_min_distance)));
            }
            if (config.support_rests_on_model && (! current_elem.to_buildplate || mergelayer))
            {
                to_model_data = TreeSupportUtils::safeUnion(increased.difference(
                    volumes_.getAvoidance(radius, layer_idx - 1, current_elem.to_model_gracious ? settings.type : AvoidanceType::COLLISION, true, settings.use_min_distance)));
            }
            check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;
            if (check_layer_data.area() < 1)
            {
                spdlog::error(
                    "Lost area by doing catch up from {} to radius {}",
                    ceil_radius_before,
                    volumes_.ceilRadius(config.getCollisionRadius(current_elem), settings.use_min_distance));
            }
        }
    }

    if (current_elem.influence_area_limit_active && ! current_elem.use_min_xy_dist && check_layer_data.area() > 1
        && (current_elem.to_model_gracious || current_elem.distance_to_top <= config.min_dtt_to_model))
    {
        const coord_t max_radius_increase = std::max(
            static_cast<coord_t>((config.branch_radius - config.min_radius) / config.tip_layers),
            static_cast<coord_t>(
                (config.branch_radius * config.diameter_angle_scale_factor)
                + config.branch_radius * (std::max(config.diameter_scale_bp_radius - config.diameter_angle_scale_factor, 0.0))));
        bool limit_range_validated = false;
        // Rounding errors in a while loop can cause non-termination, so better safe than sorry. See https://github.com/Ultimaker/Cura/issues/14133 for an example.
        to_bp_data = TreeSupportUtils::safeUnion(to_bp_data);
        to_model_data = TreeSupportUtils::safeUnion(to_model_data);
        while (! limit_range_validated)
        {
            if (current_elem.to_buildplate)
            {
                Polygons limited_to_bp = to_bp_data.intersection((current_elem.influence_area_limit_area));
                if (limited_to_bp.area() > 1)
                {
                    to_bp_data = limited_to_bp;
                    to_model_data = to_model_data.intersection((current_elem.influence_area_limit_area));
                    limit_range_validated = true;
                }
            }
            else
            {
                Polygons limited_to_model_data = to_model_data.intersection((current_elem.influence_area_limit_area));
                if (limited_to_model_data.area() > 1)
                {
                    to_bp_data = to_bp_data.intersection((current_elem.influence_area_limit_area));
                    to_model_data = limited_to_model_data;
                    limit_range_validated = true;
                }
            }
            if (! limit_range_validated)
            {
                const coord_t reach_increase = std::max(current_elem.influence_area_limit_range / 4, (config.maximum_move_distance + max_radius_increase));
                current_elem.influence_area_limit_range += reach_increase;
                current_elem.RecreateInfluenceLimitArea();
            }
        }
    }

    return check_layer_data.area() > 1 ? std::optional<TreeSupportElement>(current_elem) : std::optional<TreeSupportElement>();
}

void TreeSupport::increaseAreas(
    PropertyAreasUnordered& to_bp_areas,
    PropertyAreas& to_model_areas,
    PropertyAreas& influence_areas,
    std::vector<TreeSupportElement*>& bypass_merge_areas,
    const std::vector<TreeSupportElement*>& last_layer,
    const LayerIndex layer_idx,
    const bool mergelayer)
{
    std::mutex critical_sections;
    cura::parallel_for<size_t>(
        0,
        last_layer.size(),
        [&](const size_t idx)
        {
            TreeSupportElement* parent = last_layer[idx];
            TreeSupportElement elem(parent); // Also increases dtt.
            // Abstract representation of the model outline. If an influence area would move through it, it could teleport through a wall.
            const Polygons wall_restriction = volumes_.getWallRestriction(config.getCollisionRadius(*parent), layer_idx, parent->use_min_xy_dist);

            Polygons to_bp_data;
            Polygons to_model_data;
            coord_t radius = config.getCollisionRadius(elem);

            // When the radius increases, the outer "support wall" of the branch will have been moved farther away from the center (as this is the definition of radius).
            // As it is not specified that the support_tree_angle has to be one of the center of the branch,
            //   it is here seen as the smaller angle of the outer wall of the branch, to the outer wall of the same branch one layer above.
            // As the branch may have become larger the distance between these 2 walls is smaller than the distance of the center points.
            // These extra distance is added to the movement distance possible for this layer.

            coord_t extra_speed = EPSILON; // The extra speed is added to both movement distances. Also move 5 microns faster than allowed to avoid rounding errors, this may cause
                                           // issues at VERY VERY small layer heights.
            coord_t extra_slow_speed = 0; // Only added to the slow movement distance.
            const coord_t ceiled_parent_radius = volumes_.ceilRadius(config.getCollisionRadius(*parent), parent->use_min_xy_dist);
            const coord_t projected_radius_increased = config.getRadius(parent->effective_radius_height + 1, parent->buildplate_radius_increases);
            const coord_t projected_radius_delta = projected_radius_increased - config.getCollisionRadius(*parent);

            // When z distance is more than one layer up and down the Collision used to calculate the wall restriction will always include the wall (and not just the
            // xy_min_distance) of the layer above and below like this (d = blocked area because of z distance):
            /*
             *  layer z+1:dddddiiiiiioooo
             *  layer z+0:xxxxxdddddddddd
             *  layer z-1:dddddxxxxxxxxxx
             *  For more detailed visualisation see calculateWallRestrictions
             */
            const coord_t safe_movement_distance = (elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance)
                                                 + (std::min(config.z_distance_top_layers, config.z_distance_bottom_layers) > 0 ? config.min_feature_size : 0);
            if (ceiled_parent_radius == volumes_.ceilRadius(projected_radius_increased, parent->use_min_xy_dist)
                || projected_radius_increased < config.increase_radius_until_radius)
            {
                // If it is guaranteed possible to increase the radius, the maximum movement speed can be increased, as it is assumed that the maximum movement speed is the one of
                // the slower moving wall
                extra_speed += projected_radius_delta;
            }
            else
            {
                // If a guaranteed radius increase is not possible, only increase the slow speed.
                // Ensure that the slow movement distance can not become larger than the fast one.
                extra_slow_speed += std::min(projected_radius_delta, (config.maximum_move_distance + extra_speed) - (config.maximum_move_distance_slow + extra_slow_speed));
            }

            if (config.layer_start_bp_radius > layer_idx
                && config.recommendedMinRadius(layer_idx - 1) < config.getRadius(elem.effective_radius_height + 1, elem.buildplate_radius_increases))
            {
                // Can guarantee elephant foot radius increase.
                if (ceiled_parent_radius
                    == volumes_.ceilRadius(config.getRadius(parent->effective_radius_height + 1, parent->buildplate_radius_increases + 1), parent->use_min_xy_dist))
                {
                    extra_speed += config.branch_radius * config.diameter_scale_bp_radius;
                }
                else
                {
                    extra_slow_speed += std::min(
                        coord_t(config.branch_radius * config.diameter_scale_bp_radius),
                        config.maximum_move_distance - (config.maximum_move_distance_slow + extra_slow_speed));
                }
            }

            const coord_t fast_speed = config.maximum_move_distance + extra_speed;
            const coord_t slow_speed = config.maximum_move_distance_slow + extra_speed + extra_slow_speed;

            Polygons offset_slow;
            Polygons offset_fast;

            bool add = false;
            bool bypass_merge = false;

            // Aliases for better readability.
            constexpr bool increase_radius = true;
            constexpr bool no_error = true;
            constexpr bool use_min_radius = true;
            constexpr bool move = true;

            // Determine in which order configurations are checked if they result in a valid influence area. Check will stop if a valid area is found
            std::deque<AreaIncreaseSettings> order;
            std::function<void(const AreaIncreaseSettings&, bool)> insertSetting = [&](const AreaIncreaseSettings& settings, bool back)
            {
                if (std::find(order.begin(), order.end(), settings) == order.end())
                {
                    if (back)
                    {
                        order.emplace_back(settings);
                    }
                    else
                    {
                        order.emplace_front(settings);
                    }
                }
            };

            const bool parent_moved_slow = elem.last_area_increase.increase_speed < config.maximum_move_distance;
            const bool avoidance_speed_mismatch = parent_moved_slow && elem.last_area_increase.type != AvoidanceType::SLOW;
            if (elem.last_area_increase.move && elem.last_area_increase.no_error && elem.can_use_safe_radius && ! mergelayer && ! avoidance_speed_mismatch
                && (elem.distance_to_top >= config.tip_layers || parent_moved_slow))
            {
                // Assume that the avoidance type that was best for the parent is best for me. Makes this function about 7% faster.
                const auto slow_or_fast = elem.last_area_increase.increase_speed < config.maximum_move_distance ? slow_speed : fast_speed;
                insertSetting(
                    AreaIncreaseSettings(
                        elem.last_area_increase.type,
                        slow_or_fast,
                        increase_radius,
                        elem.last_area_increase.no_error,
                        ! use_min_radius,
                        elem.last_area_increase.move),
                    true);
                insertSetting(
                    AreaIncreaseSettings(
                        elem.last_area_increase.type,
                        slow_or_fast,
                        ! increase_radius,
                        elem.last_area_increase.no_error,
                        ! use_min_radius,
                        elem.last_area_increase.move),
                    true);
            }
            // Branch may still go though a hole, so a check has to be done whether the hole was already passed, and the regular avoidance can be used.
            if (! elem.can_use_safe_radius)
            {
                // If the radius until which it is always increased can not be guaranteed, move fast. This is to avoid holes smaller than the real branch radius.
                // This does not guarantee the avoidance of such holes, but ensures they are avoided if possible.
                insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, slow_speed, increase_radius, no_error, ! use_min_radius, ! move), true); // Did we go through the hole.
                // In many cases the definition of hole is overly restrictive, so to avoid unnecessary fast movement in the tip, it is ignored there for a bit.
                // This CAN cause a branch to go though a hole it otherwise may have avoided.
                if (elem.distance_to_top < round_up_divide(config.tip_layers, 2))
                {
                    insertSetting(AreaIncreaseSettings(AvoidanceType::FAST, slow_speed, increase_radius, no_error, ! use_min_radius, ! move), true);
                }
                insertSetting(
                    AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, increase_radius, no_error, ! use_min_radius, ! move),
                    true); // Did we manage to avoid the hole,
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, ! increase_radius, no_error, ! use_min_radius, move), true);
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST, fast_speed, ! increase_radius, no_error, ! use_min_radius, move), true);
            }
            else
            {
                insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, slow_speed, increase_radius, no_error, ! use_min_radius, move), true);
                // While moving fast to be able to increase the radius (b) may seems preferable (over a) this can cause the a sudden skip in movement, which looks similar to a
                // layer shift and can reduce stability. As such idx have chosen to only use the user setting for radius increases as a friendly recommendation.
                insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, slow_speed, ! increase_radius, no_error, ! use_min_radius, move), true); // a (See above.)
                if (elem.distance_to_top < config.tip_layers)
                {
                    insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, slow_speed, increase_radius, no_error, ! use_min_radius, move), true);
                }
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, increase_radius, no_error, ! use_min_radius, move), true); // b (See above.)
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, ! increase_radius, no_error, ! use_min_radius, move), true);
            }

            if (elem.use_min_xy_dist)
            {
                std::deque<AreaIncreaseSettings> new_order;
                // If the branch currently has to use min_xy_dist check if the configuration would also be valid with the regular xy_distance before checking with use_min_radius.
                // (Only happens when Support Distance priority is z overrides xy )
                for (AreaIncreaseSettings settings : order)
                {
                    new_order.emplace_back(settings);
                    new_order.emplace_back(settings.type, settings.increase_speed, settings.increase_radius, settings.no_error, use_min_radius, settings.move);
                }
                order = new_order;
            }

            insertSetting(
                AreaIncreaseSettings(AvoidanceType::FAST, fast_speed, ! increase_radius, ! no_error, elem.use_min_xy_dist, move),
                true); // simplifying is very important for performance, but before an error is compensated by moving faster it makes sense to check to see if the simplifying has
                       // caused issues

            // The getAccumulatedPlaceable0 intersection is just a quick and dirty check to see that at least a part of the branch would correctly rest on the model.
            // Proper way would be to offset getAccumulatedPlaceable0 by -radius first, but the small benefit to maybe detect an error, that should not be happening anyway is not
            // worth the performance impact in the expected case when a branch rests on the model.
            if (elem.to_buildplate || (elem.to_model_gracious && (parent->area->intersection(volumes_.getPlaceableAreas(radius, layer_idx)).empty()))
                || (! elem.to_model_gracious && (parent->area->intersection(volumes_.getAccumulatedPlaceable0(layer_idx)).empty()))) // Error case.
            {
                // It is normal that we won't be able to find a new area at some point in time if we won't be able to reach layer 0 aka have to connect with the model.
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST, fast_speed * 1.5, ! increase_radius, ! no_error, elem.use_min_xy_dist, move), true);
            }
            if (elem.distance_to_top < elem.dont_move_until && elem.can_use_safe_radius) // Only do not move when holes would be avoided in every case.
            {
                insertSetting(
                    AreaIncreaseSettings(AvoidanceType::SLOW, 0, increase_radius, no_error, ! use_min_radius, ! move),
                    false); // Only do not move when already in a no hole avoidance with the regular xy distance.
            }

            Polygons inc_wo_collision;
            // Check whether it is faster to calculate the area increased with the fast speed independently from the slow area, or time could be saved by reusing the slow area to
            // calculate the fast one. Calculated by comparing the steps saved when calculating independently with the saved steps when not.
            const bool offset_independent_faster = (radius / safe_movement_distance - (((config.maximum_move_distance + extra_speed) < (radius + safe_movement_distance)) ? 1 : 0))
                                                 > (round_up_divide((extra_speed + extra_slow_speed + config.maximum_move_distance_slow), safe_movement_distance));
            for (AreaIncreaseSettings settings : order)
            {
                if (settings.move)
                {
                    if (offset_slow.empty() && (settings.increase_speed == slow_speed || ! offset_independent_faster))
                    {
                        offset_slow = TreeSupportUtils::safeOffsetInc(
                                          *parent->area,
                                          extra_speed + extra_slow_speed + config.maximum_move_distance_slow,
                                          wall_restriction,
                                          safe_movement_distance,
                                          offset_independent_faster ? safe_movement_distance + radius : 0,
                                          2, // Offsetting in 2 steps makes our offsetted area rounder preventing (rounding) errors created by to pointy areas.
                                          config.support_line_distance / 2,
                                          &config.simplifier)
                                          .unionPolygons();
                        // At this point one can see that the Polygons class was never made for precision in the single digit micron range.
                    }

                    if ((settings.increase_speed != slow_speed) && offset_fast.empty())
                    {
                        if (offset_independent_faster)
                        {
                            offset_fast = TreeSupportUtils::safeOffsetInc(
                                              *parent->area,
                                              extra_speed + config.maximum_move_distance,
                                              wall_restriction,
                                              safe_movement_distance,
                                              offset_independent_faster ? safe_movement_distance + radius : 0,
                                              1,
                                              config.support_line_distance / 2,
                                              &config.simplifier)
                                              .unionPolygons();
                        }
                        else
                        {
                            const coord_t delta_slow_fast = config.maximum_move_distance - (config.maximum_move_distance_slow + extra_slow_speed);
                            offset_fast = TreeSupportUtils::safeOffsetInc(
                                              offset_slow,
                                              delta_slow_fast,
                                              wall_restriction,
                                              safe_movement_distance,
                                              safe_movement_distance + radius,
                                              offset_independent_faster ? 2 : 1,
                                              config.support_line_distance / 2,
                                              &config.simplifier)
                                              .unionPolygons();
                        }
                    }
                }
                std::optional<TreeSupportElement> result;

                // Check for errors!
                if (! settings.no_error)
                {
                    // If the area becomes for whatever reason something that clipper sees as a line, offset would stop working, so ensure that even if if wrongly would be a line,
                    // it still actually has an area that can be increased
                    Polygons lines_offset = TreeSupportUtils::toPolylines(*parent->area).offsetPolyLine(EPSILON);
                    Polygons base_error_area = parent->area->unionPolygons(lines_offset);
                    result = increaseSingleArea(settings, layer_idx, parent, base_error_area, to_bp_data, to_model_data, inc_wo_collision, settings.increase_speed, mergelayer);

                    if (fast_speed < settings.increase_speed)
                    {
                        spdlog::warn(
                            "Influence area could not be increased! Data about the Influence area: "
                            "Radius: {} at layer: {} NextTarget: {} Distance to top: {} Elephant foot increases {}  use_min_xy_dist {} to buildplate {} gracious {} safe {} until "
                            "move {} \n "
                            "Parent {}: Radius: {} at layer: {} NextTarget: {} Distance to top: {} Elephant foot increases {}  use_min_xy_dist {} to buildplate {} gracious {} "
                            "safe {} until move {}",
                            radius,
                            layer_idx - 1,
                            elem.next_height,
                            elem.distance_to_top,
                            elem.buildplate_radius_increases,
                            elem.use_min_xy_dist,
                            elem.to_buildplate,
                            elem.to_model_gracious,
                            elem.can_use_safe_radius,
                            elem.dont_move_until,
                            fmt::ptr(parent),
                            config.getCollisionRadius(*parent),
                            layer_idx,
                            parent->next_height,
                            parent->distance_to_top,
                            parent->buildplate_radius_increases,
                            parent->use_min_xy_dist,
                            parent->to_buildplate,
                            parent->to_model_gracious,
                            parent->can_use_safe_radius,
                            parent->dont_move_until);
                    }
                }
                else
                {
                    result = increaseSingleArea(
                        settings,
                        layer_idx,
                        parent,
                        settings.increase_speed == slow_speed ? offset_slow : offset_fast,
                        to_bp_data,
                        to_model_data,
                        inc_wo_collision,
                        std::max(settings.increase_speed - fast_speed, coord_t(0)),
                        mergelayer);
                }

                if (result)
                {
                    elem = result.value();
                    radius = config.getCollisionRadius(elem);
                    elem.last_area_increase = settings;
                    add = true;
                    bypass_merge
                        = ! settings.move
                       || (settings.use_min_distance
                           && elem.distance_to_top < config.tip_layers); // Do not merge if the branch should not move or the priority has to be to get farther away from the model.
                    if (settings.move)
                    {
                        elem.dont_move_until = 0;
                    }
                    else
                    {
                        elem.result_on_layer = parent->result_on_layer;
                    }

                    elem.can_use_safe_radius = settings.type != AvoidanceType::FAST;

                    if (! settings.use_min_distance)
                    {
                        elem.use_min_xy_dist = false;
                    }
                    if (! settings.no_error && fast_speed < settings.increase_speed)
                    {
                        spdlog::warn("Trying to keep area by moving faster than intended: Success.");
                    }
                    break;
                }
                else if (! settings.no_error && fast_speed < settings.increase_speed)
                {
                    spdlog::error("Trying to keep area by moving faster than intended: FAILURE! Wrong branch likely!");
                }
            }

            if (add)
            {
                Polygons max_influence_area = TreeSupportUtils::safeUnion(
                    inc_wo_collision.difference(volumes_.getCollision(radius, layer_idx - 1, elem.use_min_xy_dist)),
                    TreeSupportUtils::safeUnion(to_bp_data, to_model_data));
                // ^^^ Note: union seems useless, but some rounding errors somewhere can cause to_bp_data to be slightly bigger than it should be

                {
                    std::lock_guard<std::mutex> critical_section_newLayer(critical_sections);
                    if (bypass_merge)
                    {
                        Polygons* new_area = new Polygons(max_influence_area);
                        TreeSupportElement* next = new TreeSupportElement(elem, new_area);
                        bypass_merge_areas.emplace_back(next);
                    }
                    else
                    {
                        influence_areas.emplace(elem, max_influence_area);
                        if (elem.to_buildplate)
                        {
                            to_bp_areas.emplace(elem, to_bp_data);
                        }
                        if (config.support_rests_on_model)
                        {
                            to_model_areas.emplace(elem, to_model_data);
                        }
                    }
                }
            }
            else
            {
                // If the bottom most point of a branch is set, later functions will assume that the position is valid, and ignore it.
                // But as branches connecting with the model that are to small have to be culled, the bottom most point has to be not set.
                // A point can be set on the top most tip layer (maybe more if it should not move for a few layers).
                parent->result_on_layer = Point(-1, -1);
            }
        });
}

void TreeSupport::createLayerPathing(std::vector<std::set<TreeSupportElement*>>& move_bounds)
{
    const double data_size_inverse = 1 / double(move_bounds.size());
    double progress_total = TREE_PROGRESS_PRECALC_AVO + TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_GENERATE_NODES;

    auto dur_inc = std::chrono::duration_values<std::chrono::nanoseconds>::zero();
    auto dur_merge = std::chrono::duration_values<std::chrono::nanoseconds>::zero();

    const auto dur_inc_recent = std::chrono::duration_values<std::chrono::nanoseconds>::zero();
    const auto dur_merge_recent = std::chrono::duration_values<std::chrono::nanoseconds>::zero();

    LayerIndex last_merge = move_bounds.size();
    bool new_element = false;

    // Ensure at least one merge operation per 3mm height, 50 layers, 1 mm movement of slow speed or 5mm movement of fast speed (whatever is lowest). Values were guessed.
    size_t max_merge_every_x_layers = std::min(
        std::min(5000 / (std::max(config.maximum_move_distance, static_cast<coord_t>(100))), 1000 / std::max(config.maximum_move_distance_slow, static_cast<coord_t>(20))),
        3000 / config.layer_height);

    size_t merge_every_x_layers = 1;
    // Calculate the influence areas for each layer below (Top down)
    // This is done by first increasing the influence area by the allowed movement distance, and merging them with other influence areas if possible
    for (const auto layer_idx : ranges::views::iota(1UL, move_bounds.size()) | ranges::views::reverse)
    {
        // Merging is expensive and only parallelized to a max speedup of 2. As such it may be useful in some cases to only merge every few layers to improve performance.
        bool merge_this_layer = size_t(last_merge - layer_idx) >= merge_every_x_layers;
        if (new_element)
        {
            merge_this_layer = true;
            merge_every_x_layers = 1;
        }

        PropertyAreas influence_areas; // Over this map will be iterated when merging, as such it has to be ordered to ensure deterministic results.
        PropertyAreas to_model_areas; // The area of these SupportElement is not set, to avoid to much allocation and deallocation on the heap.
        PropertyAreasUnordered to_bp_areas; // Same.
        std::vector<TreeSupportElement*>
            bypass_merge_areas; // Different to the other maps of SupportElements as these here have the area already set, as they are already to be inserted into move_bounds.

        const auto time_a = std::chrono::high_resolution_clock::now();

        std::vector<TreeSupportElement*> last_layer;
        last_layer.insert(last_layer.begin(), move_bounds[layer_idx].begin(), move_bounds[layer_idx].end());

        // ### Increase the influence areas by the allowed movement distance
        increaseAreas(to_bp_areas, to_model_areas, influence_areas, bypass_merge_areas, last_layer, layer_idx, merge_this_layer);

        const auto time_b = std::chrono::high_resolution_clock::now();
        if (merge_this_layer)
        {
            bool reduced_by_merging = false;
            size_t count_before_merge = influence_areas.size();
            // ### Calculate which influence areas overlap, and merge them into a new influence area (simplified: an intersection of influence areas that have such an intersection)
            mergeInfluenceAreas(to_bp_areas, to_model_areas, influence_areas, layer_idx);

            last_merge = layer_idx;
            reduced_by_merging = count_before_merge > influence_areas.size();
            if (! reduced_by_merging && ! new_element)
            {
                merge_every_x_layers = std::min(max_merge_every_x_layers, merge_every_x_layers + 1);
            }
        }
        const auto time_c = std::chrono::high_resolution_clock::now();

        dur_inc += time_b - time_a;
        dur_merge += time_c - time_b;

        new_element = ! move_bounds[layer_idx - 1].empty();

        // Save calculated elements to output, and allocate Polygons on heap, as they will not be changed again.
        for (std::pair<TreeSupportElement, Polygons> tup : influence_areas)
        {
            const TreeSupportElement elem = tup.first;
            Polygons* new_area = new Polygons(TreeSupportUtils::safeUnion(tup.second));
            TreeSupportElement* next = new TreeSupportElement(elem, new_area);
            move_bounds[layer_idx - 1].emplace(next);

            if (new_area->area() < 1)
            {
                spdlog::error("Insert Error of Influence area on layer {}. Origin of {} areas. Was to bp {}", layer_idx - 1, elem.parents.size(), elem.to_buildplate);
            }
        }

        // Place already fully constructed elements in the output.
        for (TreeSupportElement* elem : bypass_merge_areas)
        {
            if (elem->area->area() < 1)
            {
                spdlog::error("Insert Error of Influence area bypass on layer {}.", layer_idx - 1);
            }
            move_bounds[layer_idx - 1].emplace(elem);
        }

        progress_total += data_size_inverse * TREE_PROGRESS_AREA_CALC;
        Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
    }

    spdlog::info("Time spent with creating influence areas' subtasks: Increasing areas {} ms merging areas: {} ms", dur_inc.count() / 1000000, dur_merge.count() / 1000000);
}

void TreeSupport::setPointsOnAreas(const TreeSupportElement* elem)
{
    // Based on the branch center point of the current layer, the point on the next (further up) layer is calculated.

    if (elem->result_on_layer == Point(-1, -1))
    {
        spdlog::error("Uninitialized support element");
        return;
    }

    for (TreeSupportElement* next_elem : elem->parents)
    {
        if (next_elem->result_on_layer
            != Point(-1, -1)) // If the value was set somewhere else it it kept. This happens when a branch tries not to move after being unable to create a roof.
        {
            continue;
        }

        Point from = elem->result_on_layer;
        if (! (next_elem->area->inside(from, true)))
        {
            PolygonUtils::moveInside(
                *next_elem->area,
                from,
                0); // Move inside has edgecases (see tests) so DONT use Polygons.inside to confirm correct move, Error with distance 0 is <= 1
            // It is not required to check if how far this move moved a point as is can be larger than maximum_movement_distance. While this seems like a problem it may for example
            // occur after merges.
        }
        next_elem->result_on_layer = from;
        // Do not call recursive because then amount of layers would be restricted by the stack size.
    }
}

bool TreeSupport::setToModelContact(std::vector<std::set<TreeSupportElement*>>& move_bounds, TreeSupportElement* first_elem, const LayerIndex layer_idx)
{
    if (first_elem->to_model_gracious)
    {
        TreeSupportElement* check = first_elem;

        std::vector<TreeSupportElement*> checked;
        LayerIndex last_successfull_layer = layer_idx;

        bool set = false;
        if (config.support_rest_preference != RestPreference::BUILDPLATE && layer_idx == 0)
        {
            set = true;
        }

        Polygons valid_place_area;

        // Check for every layer upwards, up to the point where this influence area was created (either by initial insert or merge) if the branch could be placed on it, and highest
        // up layer index.
        for (LayerIndex layer_check = layer_idx; check->next_height >= layer_check; layer_check++)
        {
            Polygons check_valid_place_area = check->area->intersection(volumes_.getPlaceableAreas(config.getCollisionRadius(*check), layer_check));

            if (! check_valid_place_area.empty())
            {
                set = true;
                last_successfull_layer = layer_check;
                valid_place_area = check_valid_place_area;
            }
            checked.emplace_back(check);
            if (check->parents.size() == 1)
            {
                check = check->parents[0];
            }
            else
            {
                break; // reached merge point
            }
        }

        // Could not find valid placement, even though it should exist => error handling
        if (! set)
        {
            if (SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL)
            {
                spdlog::warn("No valid placement found for to model gracious element on layer {}: REMOVING BRANCH", layer_idx);
                for (LayerIndex layer = layer_idx; layer <= first_elem->next_height; layer++)
                {
                    move_bounds[layer].erase(checked[layer - layer_idx]);
                    delete checked[layer - layer_idx]->area;
                    delete checked[layer - layer_idx];
                }
                return true;
            }
            else
            {
                spdlog::warn("No valid placement found for to model gracious element on layer {}", layer_idx);
                first_elem->to_model_gracious = false;
                return setToModelContact(move_bounds, first_elem, layer_idx);
            }
        }

        for (LayerIndex layer = layer_idx + 1; layer < last_successfull_layer - 1;
             ++layer) // NOTE: Use of 'itoa' will make this crash in the loop, even though the operation should be equivalent.
        {
            move_bounds[layer].erase(checked[layer - layer_idx]);
            delete checked[layer - layer_idx]->area;
            delete checked[layer - layer_idx];
        }

        // If resting on the buildplate keep bp location
        if (config.support_rest_preference != RestPreference::BUILDPLATE && last_successfull_layer == 0)
        {
            return false;
        }

        // Guess a point inside the influence area, in which the branch will be placed in.
        Point best = checked[last_successfull_layer - layer_idx]->next_position;

        if (! valid_place_area.inside(best, true))
        {
            PolygonUtils::moveInside(valid_place_area, best);
        }

        checked[last_successfull_layer - layer_idx]->result_on_layer = best;
        spdlog::debug("Added gracious Support On Model Point ({},{}). The current layer is {}", best.X, best.Y, last_successfull_layer);

        return last_successfull_layer != layer_idx;
    }
    else // can not add graceful => just place it here and hope for the best
    {
        Point best = first_elem->next_position;
        Polygons valid_place_area
            = first_elem->area->difference(volumes_.getAvoidance(config.getCollisionRadius(first_elem), layer_idx, AvoidanceType::COLLISION, first_elem->use_min_xy_dist));

        if (! valid_place_area.inside(best, true))
        {
            if (! valid_place_area.empty())
            {
                PolygonUtils::moveInside(valid_place_area, best);
            }
            else
            {
                bool found_partial_placement;
                for (coord_t radius_offset : { -config.getCollisionRadius(first_elem),
                                               -config.getCollisionRadius(first_elem) / 2,
                                               coord_t(0) }) // Interestingly the first radius is working most of the time, even though it seems like it shouldn't.
                {
                    valid_place_area = first_elem->area->intersection(volumes_.getAccumulatedPlaceable0(layer_idx).offset(radius_offset));
                    if (! valid_place_area.empty())
                    {
                        PolygonUtils::moveInside(valid_place_area, best);
                        spdlog::warn(
                            "Not able to place branch fully on non support blocker at layer {} using offset {} for radius {}",
                            layer_idx,
                            radius_offset,
                            config.getCollisionRadius(first_elem));
                        found_partial_placement = true;
                        break;
                    }
                }
                if (! found_partial_placement)
                {
                    PolygonUtils::moveInside(*first_elem->area, best);
                    spdlog::warn("Not able to place branch on non support blocker at layer {}", layer_idx);
                }
            }
        }
        first_elem->result_on_layer = best;
        first_elem->to_model_gracious = false;
        spdlog::debug("Added NON gracious Support On Model Point ({},{}). The current layer is {}", best.X, best.Y, layer_idx);
        return false;
    }
}

void TreeSupport::createNodesFromArea(std::vector<std::set<TreeSupportElement*>>& move_bounds)
{
    // Initialize points on layer 0, with a "random" point in the influence area. Point is chosen based on an inaccurate estimate where the branches will split into two, but every
    // point inside the influence area would produce a valid result.
    std::unordered_set<TreeSupportElement*> remove;
    for (TreeSupportElement* init : move_bounds[0])
    {
        Point p = init->next_position;
        if (! (init->area->inside(p, true)))
        {
            PolygonUtils::moveInside(*init->area, p, 0);
        }
        init->result_on_layer = p;

        setPointsOnAreas(init); // Also set the parent nodes, as these will be required for the first iteration of the loop below.

        if (config.support_rest_preference != RestPreference::BUILDPLATE)
        {
            if (setToModelContact(move_bounds, init, 0))
            {
                remove.emplace(init);
            }
            else
            {
                // If the support_rest_preference is GRACEFUL the collision radius is increased, but the radius will only be increased if the element is to_buildplate, so if the
                // branch rests on the buildplate, the element will have to be updated to include this information.
                init->setToBuildplateForAllParents(true);
            }
        }
    }

    for (TreeSupportElement* del : remove)
    {
        move_bounds[0].erase(del);
        delete del->area;
        delete del;
    }
    remove.clear();

    for (const auto layer_idx : ranges::views::iota(1UL, move_bounds.size()))
    {
        for (TreeSupportElement* elem : move_bounds[layer_idx])
        {
            bool removed = false;
            if (elem->result_on_layer == Point(-1, -1)) // Check if the resulting center point is not yet set.
            {
                if (elem->to_buildplate || (! elem->to_buildplate && elem->distance_to_top < config.min_dtt_to_model && ! elem->supports_roof))
                {
                    if (elem->to_buildplate)
                    {
                        spdlog::error(
                            "Uninitialized Influence area targeting ({},{}) at target_height: {} layer: {}",
                            elem->target_position.X,
                            elem->target_position.Y,
                            elem->target_height,
                            layer_idx);
                    }
                    remove.emplace(elem); // We dont need to remove yet the parents as they will have a lower dtt and also no result_on_layer set.
                    removed = true;
                    for (TreeSupportElement* parent : elem->parents)
                    {
                        // When the roof was not able to generate downwards enough, the top elements may have not moved, and have result_on_layer already set. As this branch needs
                        // to be removed => all parents result_on_layer have to be invalidated.
                        parent->result_on_layer = Point(-1, -1);
                    }
                    continue;
                }
                else
                {
                    // Set the point where the branch will be placed on the model.
                    removed = setToModelContact(move_bounds, elem, layer_idx);
                    if (removed)
                    {
                        remove.emplace(elem);
                    }
                }
            }

            if (! removed)
            {
                setPointsOnAreas(elem); // Element is valid now setting points in the layer above.
            }
        }

        // Delete all not needed support elements.
        for (TreeSupportElement* del : remove)
        {
            move_bounds[layer_idx].erase(del);
            delete del->area;
            delete del;
        }
        remove.clear();
    }
}

void TreeSupport::generateBranchAreas(
    std::vector<std::pair<LayerIndex, TreeSupportElement*>>& linear_data,
    std::vector<std::unordered_map<TreeSupportElement*, Polygons>>& layer_tree_polygons,
    const std::map<TreeSupportElement*, TreeSupportElement*>& inverse_tree_order)
{
    double progress_total = TREE_PROGRESS_PRECALC_AVO + TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_GENERATE_NODES + TREE_PROGRESS_AREA_CALC;
    constexpr int progress_report_steps = 10;
    Polygon branch_circle; // Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.

    {
        Polygon base_circle = TreeSupportBaseCircle::getBaseCircle();
        for (Point vertex : base_circle)
        {
            vertex = Point(vertex.X * config.branch_radius / TreeSupportBaseCircle::base_radius, vertex.Y * config.branch_radius / TreeSupportBaseCircle::base_radius);
            branch_circle.add(vertex);
        }
    }

    std::vector<Polygons> linear_inserts(linear_data.size());
    const size_t progress_inserts_check_interval = linear_data.size() / progress_report_steps;

    std::mutex critical_sections;
    cura::parallel_for<size_t>(
        0,
        linear_data.size(),
        [&](const size_t idx)
        {
            TreeSupportElement* elem = linear_data[idx].second;
            coord_t radius = config.getRadius(*elem);
            bool parent_uses_min = false;
            TreeSupportElement* child_elem = inverse_tree_order.count(elem) ? inverse_tree_order.at(elem) : nullptr;

            // Calculate multiple ovalized circles, to connect with every parent and child. Also generate regular circle for the current layer. Merge all these into one area.
            std::vector<std::pair<Point, coord_t>> movement_directions{ std::pair<Point, coord_t>(Point(0, 0), radius) };
            if (! elem->skip_ovalisation)
            {
                if (child_elem != nullptr)
                {
                    Point movement = (child_elem->result_on_layer - elem->result_on_layer);
                    movement_directions.emplace_back(movement, radius);
                }
                for (TreeSupportElement* parent : elem->parents)
                {
                    Point movement = (parent->result_on_layer - elem->result_on_layer);
                    movement_directions.emplace_back(movement, std::max(config.getRadius(parent), config.support_line_width));
                    parent_uses_min |= parent->use_min_xy_dist;
                }

                for (Point target : elem->additional_ovalization_targets)
                {
                    Point movement = (target - elem->result_on_layer);
                    movement_directions.emplace_back(movement, std::max(radius, config.support_line_width));
                }
            }

            coord_t max_speed_sqd = 0;
            std::function<Polygons(coord_t)> generateArea = [&](coord_t offset)
            {
                Polygons poly;

                for (std::pair<Point, coord_t> movement : movement_directions)
                {
                    max_speed_sqd = std::max(max_speed_sqd, vSize2(movement.first));

                    // Visualization: https://jsfiddle.net/0zvcq39L/2/
                    // Ovalizes the circle to an ellipse, that contains both old center and new target position.
                    double used_scale = (movement.second + offset) / (1.0 * config.branch_radius);
                    Point center_position = elem->result_on_layer + movement.first / 2;
                    const double moveX = movement.first.X / (used_scale * config.branch_radius);
                    const double moveY = movement.first.Y / (used_scale * config.branch_radius);
                    const double vsize_inv = 0.5 / (0.01 + std::sqrt(moveX * moveX + moveY * moveY));

                    std::array<double, 4> matrix = {
                        used_scale * (1 + moveX * moveX * vsize_inv),
                        used_scale * (0 + moveX * moveY * vsize_inv),
                        used_scale * (0 + moveX * moveY * vsize_inv),
                        used_scale * (1 + moveY * moveY * vsize_inv),
                    };
                    Polygon circle;
                    for (Point vertex : branch_circle)
                    {
                        vertex = Point(matrix[0] * vertex.X + matrix[1] * vertex.Y, matrix[2] * vertex.X + matrix[3] * vertex.Y);
                        circle.add(center_position + vertex);
                    }
                    poly.add(circle.offset(0));
                }

                poly = poly.unionPolygons()
                           .offset(std::min(static_cast<coord_t>(FUDGE_LENGTH), config.support_line_width / 4))
                           .difference(volumes_.getCollision(0, linear_data[idx].first, parent_uses_min || elem->use_min_xy_dist));
                // ^^^ There seem to be some rounding errors, causing a branch to be a tiny bit further away from the model that it has to be. This can cause the tip to be slightly
                // further away front the overhang (x/y wise) than optimal.
                //     This fixes it, and for every other part, 0.05mm will not be noticed.
                return poly;
            };

            constexpr auto three_quarters_sqd = 0.75 * 0.75;
            const bool fast_relative_movement = max_speed_sqd > (radius * radius * three_quarters_sqd);

            // Ensure branch area will not overlap with model/collision. This can happen because of e.g. ovalization or increase_until_radius.
            linear_inserts[idx] = generateArea(0);

            if (fast_relative_movement || config.getRadius(*elem) - config.getCollisionRadius(*elem) > config.support_line_width)
            {
                // Simulate the path the nozzle will take on the outermost wall.
                // If multiple parts exist, the outer line will not go all around the support part potentially causing support material to be printed mid air.
                Polygons nozzle_path = linear_inserts[idx].offset(-config.support_line_width / 2);
                if (nozzle_path.splitIntoParts(false).size() > 1)
                {
                    // Just try to make the area a tiny bit larger.
                    linear_inserts[idx] = generateArea(config.support_line_width / 2);
                    nozzle_path = linear_inserts[idx].offset(-config.support_line_width / 2);

                    // if larger area did not fix the problem, all parts off the nozzle path that do not contain the center point are removed, hoping for the best
                    if (nozzle_path.splitIntoParts(false).size() > 1)
                    {
                        Polygons polygons_with_correct_center;
                        for (PolygonsPart part : nozzle_path.splitIntoParts(false))
                        {
                            if (part.inside(elem->result_on_layer, true))
                            {
                                polygons_with_correct_center = polygons_with_correct_center.unionPolygons(part);
                            }
                            else
                            {
                                // Try a fuzzy inside as sometimes the point should be on the border, but is not because of rounding errors...
                                Point from = elem->result_on_layer;
                                PolygonUtils::moveInside(part, from, 0);
                                if (vSize2(elem->result_on_layer - from) < (FUDGE_LENGTH * FUDGE_LENGTH) / 4)
                                {
                                    polygons_with_correct_center = polygons_with_correct_center.unionPolygons(part);
                                }
                            }
                        }
                        // Increase the area again, to ensure the nozzle path when calculated later is very similar to the one assumed above.
                        linear_inserts[idx] = polygons_with_correct_center.offset(config.support_line_width / 2).unionPolygons();
                        linear_inserts[idx]
                            = linear_inserts[idx].difference(volumes_.getCollision(0, linear_data[idx].first, parent_uses_min || elem->use_min_xy_dist)).unionPolygons();
                    }
                }
            }

            if (idx % progress_inserts_check_interval == 0)
            {
                {
                    std::lock_guard<std::mutex> critical_section_progress(critical_sections);
                    progress_total += TREE_PROGRESS_GENERATE_BRANCH_AREAS / progress_report_steps;
                    Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
                }
            }
        });

    // Single threaded combining all elements to the right layers. Only copies data!
    for (const coord_t i : ranges::views::iota(0UL, linear_data.size()))
    {
        layer_tree_polygons[linear_data[i].first].emplace(linear_data[i].second, linear_inserts[i]);
    }
}

void TreeSupport::smoothBranchAreas(std::vector<std::unordered_map<TreeSupportElement*, Polygons>>& layer_tree_polygons)
{
    double progress_total = TREE_PROGRESS_PRECALC_AVO + TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_GENERATE_NODES + TREE_PROGRESS_AREA_CALC + TREE_PROGRESS_GENERATE_BRANCH_AREAS;
    const coord_t max_radius_change_per_layer = 1 + config.support_line_width / 2; // This is the upper limit a radius may change per layer. +1 to avoid rounding errors.

    // Smooth upward.
    for (const auto layer_idx : ranges::views::iota(0UL, std::max<size_t>(layer_tree_polygons.size(), 1UL) - 1UL))
    {
        std::vector<std::pair<TreeSupportElement*, Polygons>> processing;
        processing.insert(processing.end(), layer_tree_polygons[layer_idx].begin(), layer_tree_polygons[layer_idx].end());
        std::vector<std::vector<std::pair<TreeSupportElement*, Polygons>>> update_next(processing.size()); // With this a lock can be avoided.
        cura::parallel_for<size_t>(
            0,
            processing.size(),
            [&](const size_t processing_idx)
            {
                std::pair<TreeSupportElement*, Polygons> data_pair = processing[processing_idx];

                coord_t max_outer_wall_distance = 0;
                bool do_something = false;
                for (TreeSupportElement* parent : data_pair.first->parents)
                {
                    if (config.getRadius(*parent) != config.getCollisionRadius(*parent))
                    {
                        do_something = true;
                        max_outer_wall_distance = std::max(
                            max_outer_wall_distance,
                            vSize(data_pair.first->result_on_layer - parent->result_on_layer) - (config.getRadius(*data_pair.first) - config.getRadius(*parent)));
                    }
                }
                max_outer_wall_distance
                    += max_radius_change_per_layer; // As this change is a bit larger than what usually appears, lost radius can be slowly reclaimed over the layers.
                if (do_something)
                {
                    Polygons max_allowed_area = data_pair.second.offset(max_outer_wall_distance);
                    for (TreeSupportElement* parent : data_pair.first->parents)
                    {
                        if (config.getRadius(*parent) != config.getCollisionRadius(*parent))
                        {
                            update_next[processing_idx].emplace_back(
                                std::pair<TreeSupportElement*, Polygons>(parent, layer_tree_polygons[layer_idx + 1][parent].intersection(max_allowed_area)));
                        }
                    }
                }
            });

        for (std::vector<std::pair<TreeSupportElement*, Polygons>> data_vector : update_next)
        {
            for (std::pair<TreeSupportElement*, Polygons> data_pair : data_vector)
            {
                layer_tree_polygons[layer_idx + 1][data_pair.first] = data_pair.second;
            }
        }
    }

    progress_total += TREE_PROGRESS_SMOOTH_BRANCH_AREAS / 2;
    Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
    // ^^^ It is just assumed that both smoothing loops together are one third of the time spent in this function. This was guessed.
    //     As the whole function is only 10%, and the smoothing is hard to predict a progress report in the loop may be not useful.

    // Smooth downwards.
    std::unordered_set<TreeSupportElement*> updated_last_iteration;
    for (const auto layer_idx : ranges::views::iota(0UL, std::max<size_t>(layer_tree_polygons.size(), 1UL) - 1UL) | ranges::views::reverse)
    {
        std::vector<std::pair<TreeSupportElement*, Polygons>> processing;
        processing.insert(processing.end(), layer_tree_polygons[layer_idx].begin(), layer_tree_polygons[layer_idx].end());
        std::vector<std::pair<TreeSupportElement*, Polygons>> update_next(
            processing.size(),
            std::pair<TreeSupportElement*, Polygons>(nullptr, Polygons())); // With this a lock can be avoided.

        cura::parallel_for<size_t>(
            0,
            processing.size(),
            [&](const size_t processing_idx)
            {
                std::pair<TreeSupportElement*, Polygons> data_pair = processing[processing_idx];
                bool do_something = false;
                Polygons max_allowed_area;
                for (size_t idx = 0; idx < data_pair.first->parents.size(); idx++)
                {
                    TreeSupportElement* parent = data_pair.first->parents[idx];
                    const coord_t max_outer_line_increase = max_radius_change_per_layer;
                    Polygons result = layer_tree_polygons[layer_idx + 1][parent].offset(max_outer_line_increase);
                    const Point direction = data_pair.first->result_on_layer - parent->result_on_layer;
                    // Move the polygons object.
                    for (auto& outer : result)
                    {
                        for (Point& p : outer)
                        {
                            p += direction;
                        }
                    }
                    max_allowed_area.add(result);
                    do_something = do_something || updated_last_iteration.count(parent) || config.getCollisionRadius(*parent) != config.getRadius(*parent);
                }

                if (do_something)
                {
                    const Polygons result = max_allowed_area.unionPolygons().intersection(data_pair.second);
                    if (result.area() < data_pair.second.area())
                    {
                        update_next[processing_idx] = std::pair<TreeSupportElement*, Polygons>(data_pair.first, result);
                    }
                }
            });

        updated_last_iteration.clear();
        for (std::pair<TreeSupportElement*, Polygons> data_pair : update_next)
        {
            if (data_pair.first != nullptr)
            {
                updated_last_iteration.emplace(data_pair.first);
                layer_tree_polygons[layer_idx][data_pair.first] = data_pair.second;
            }
        }
    }

    progress_total += TREE_PROGRESS_SMOOTH_BRANCH_AREAS / 2;
    Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
}

void TreeSupport::dropNonGraciousAreas(
    std::vector<std::unordered_map<TreeSupportElement*, Polygons>>& layer_tree_polygons,
    const std::vector<std::pair<LayerIndex, TreeSupportElement*>>& linear_data,
    std::vector<std::vector<std::pair<LayerIndex, Polygons>>>& dropped_down_areas,
    const std::map<TreeSupportElement*, TreeSupportElement*>& inverse_tree_order)
{
    cura::parallel_for<size_t>(
        0,
        linear_data.size(),
        [&](const size_t idx)
        {
            TreeSupportElement* elem = linear_data[idx].second;
            bool non_gracious_model_contact
                = ! elem->to_model_gracious
               && ! inverse_tree_order.count(elem); // If an element has no child, it connects to whatever is below as no support further down for it will exist.
            if (non_gracious_model_contact)
            {
                Polygons rest_support = layer_tree_polygons[linear_data[idx].first][elem].intersection(volumes_.getAccumulatedPlaceable0(linear_data[idx].first));
                for (LayerIndex counter = 1; rest_support.area() > 1 && counter < linear_data[idx].first; ++counter)
                {
                    rest_support = rest_support.difference(volumes_.getCollision(0, linear_data[idx].first - counter));
                    dropped_down_areas[idx].emplace_back(linear_data[idx].first - counter, rest_support);
                }
            }
        });
}


void TreeSupport::filterFloatingLines(std::vector<Polygons>& support_layer_storage)
{
    const auto t_start = std::chrono::high_resolution_clock::now();

    const coord_t closing_dist = config.support_line_width * config.support_wall_count;
    const coord_t open_close_distance = config.fill_outline_gaps ? config.min_feature_size / 2 - 5 : config.min_wall_line_width / 2 - 5; // based on calculation in WallToolPath
    const double small_area_length = INT2MM(static_cast<double>(config.support_line_width) / 2);

    std::function<void(Polygons&)> reversePolygon = [&](Polygons& poly)
    {
        for (size_t idx = 0; idx < poly.size(); idx++)
        {
            poly[idx].reverse();
        }
    };


    std::vector<Polygons> support_holes(support_layer_storage.size(), Polygons());
    // Extract all holes as polygon objects
    cura::parallel_for<coord_t>(
        0,
        support_layer_storage.size(),
        [&](const LayerIndex layer_idx)
        {
            support_layer_storage[layer_idx] = config.simplifier.polygon(PolygonUtils::unionManySmall(support_layer_storage[layer_idx].smooth(FUDGE_LENGTH)))
                                                   .offset(-open_close_distance)
                                                   .offset(open_close_distance * 2)
                                                   .offset(-open_close_distance);
            support_layer_storage[layer_idx].removeSmallAreas(small_area_length * small_area_length, false);

            std::vector<Polygons> parts = support_layer_storage[layer_idx].sortByNesting();

            if (parts.size() <= 1)
            {
                return;
            }

            Polygons holes_original;
            for (const size_t idx : ranges::views::iota(1UL, parts.size()))
            {
                Polygons area = parts[idx];
                reversePolygon(area);
                holes_original.add(area);
            }
            support_holes[layer_idx] = holes_original;
        });

    const auto t_union = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<Polygons>> holeparts(support_layer_storage.size());
    // Split all holes into parts
    cura::parallel_for<coord_t>(
        0,
        support_layer_storage.size(),
        [&](const LayerIndex layer_idx)
        {
            for (Polygons hole : support_holes[layer_idx].splitIntoParts())
            {
                holeparts[layer_idx].emplace_back(hole);
            }
        });
    std::vector<std::map<size_t, std::vector<size_t>>> hole_rest_map(holeparts.size());
    std::vector<std::set<size_t>> holes_resting_outside(holeparts.size());


    // Figure out which hole rests on which other hole
    cura::parallel_for<coord_t>(
        1,
        support_layer_storage.size(),
        [&](const LayerIndex layer_idx)
        {
            if (holeparts[layer_idx].empty())
            {
                return;
            }

            Polygons outer_walls
                = TreeSupportUtils::toPolylines(support_layer_storage[layer_idx - 1].getOutsidePolygons())
                      .tubeShape(closing_dist, 0); //.unionPolygons(volumes_.getCollision(0, layer_idx - 1, true).offset(-(config.support_line_width+config.xy_min_distance)));

            Polygons holes_below;

            for (auto poly : holeparts[layer_idx - 1])
            {
                holes_below.add(poly);
            }

            for (auto [idx, hole] : holeparts[layer_idx] | ranges::views::enumerate)
            {
                AABB hole_aabb = AABB(hole);
                hole_aabb.expand(EPSILON);
                if (! hole.intersection(PolygonUtils::clipPolygonWithAABB(outer_walls, hole_aabb)).empty())
                {
                    holes_resting_outside[layer_idx].emplace(idx);
                }
                else
                {
                    for (auto [idx2, hole2] : holeparts[layer_idx - 1] | ranges::views::enumerate)
                    {
                        if (hole_aabb.hit(AABB(hole2))
                            && ! hole.intersection(hole2).empty()) // TODO should technically be outline: Check if this is fine either way as it would save an offset
                        {
                            hole_rest_map[layer_idx][idx].emplace_back(idx2);
                        }
                    }
                }
            }
        });

    const auto t_hole_rest_ordering = std::chrono::high_resolution_clock::now();

    std::unordered_set<size_t> removed_holes_by_idx;
    std::vector<Polygons> valid_holes(support_holes.size(), Polygons());
    // Check which holes have to be removed as they do not rest on anything. Only keep holes that have to be removed
    for (const size_t layer_idx : ranges::views::iota(1UL, support_holes.size()))
    {
        std::unordered_set<size_t> next_removed_holes_by_idx;

        for (auto [idx, hole] : holeparts[layer_idx] | ranges::views::enumerate)
        {
            bool found = false;
            if (holes_resting_outside[layer_idx].contains(idx))
            {
                found = true;
            }
            else
            {
                if (hole_rest_map[layer_idx].contains(idx))
                {
                    for (size_t resting_idx : hole_rest_map[layer_idx][idx])
                    {
                        if (! removed_holes_by_idx.contains(resting_idx))
                        {
                            found = true;
                            break;
                        }
                    }
                }
            }
            if (! found)
            {
                next_removed_holes_by_idx.emplace(idx);
            }
            else
            {
                valid_holes[layer_idx].add(hole);
                holeparts[layer_idx][idx] = Polygons(); // all remaining holes will have to be removed later, so removing the hole means it is confirmed valid!
            }
        }
        removed_holes_by_idx = next_removed_holes_by_idx;
    }
    const auto t_hole_removal_tagging = std::chrono::high_resolution_clock::now();

    // Check if holes are so close to each other that two lines will be printed directly next to each other, which is assumed stable (as otherwise the simulated support pattern
    // will not work correctly) and remove all remaining, invalid holes
    cura::parallel_for<coord_t>(
        1,
        support_layer_storage.size(),
        [&](const LayerIndex layer_idx)
        {
            if (holeparts[layer_idx].empty())
            {
                return;
            }

            support_layer_storage[layer_idx] = support_layer_storage[layer_idx].getOutsidePolygons();
            reversePolygon(valid_holes[layer_idx]);
            support_layer_storage[layer_idx].add(valid_holes[layer_idx]);
        });


    const auto t_end = std::chrono::high_resolution_clock::now();

    const auto dur_union = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_union - t_start).count();
    const auto dur_hole_rest_ordering = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_hole_rest_ordering - t_union).count();
    const auto dur_hole_removal_tagging = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_hole_removal_tagging - t_hole_rest_ordering).count();

    const auto dur_hole_removal = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_hole_removal_tagging).count();
    spdlog::debug(
        "Time to union areas: {} ms Time to evaluate which hole rest on which other hole: {} ms Time to see which holes are not resting on anything valid: {} ms remove all holes "
        "that are invalid and not close enough to a valid hole: {} ms",
        dur_union,
        dur_hole_rest_ordering,
        dur_hole_removal_tagging,
        dur_hole_removal);
}

void TreeSupport::finalizeInterfaceAndSupportAreas(std::vector<Polygons>& support_layer_storage, std::vector<Polygons>& support_roof_storage, SliceDataStorage& storage)
{
    InterfacePreference interface_pref = config.interface_preference; // InterfacePreference::SUPPORT_LINES_OVERWRITE_INTERFACE;
    double progress_total = TREE_PROGRESS_PRECALC_AVO + TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_GENERATE_NODES + TREE_PROGRESS_AREA_CALC + TREE_PROGRESS_GENERATE_BRANCH_AREAS
                          + TREE_PROGRESS_SMOOTH_BRANCH_AREAS;

    // Iterate over the generated circles in parallel and clean them up. Also add support floor.
    std::mutex critical_sections;
    cura::parallel_for<coord_t>(
        0,
        support_layer_storage.size(),
        [&](const LayerIndex layer_idx)
        {
            support_layer_storage[layer_idx] = support_layer_storage[layer_idx].difference(placed_support_lines_support_areas[layer_idx]);

            // Subtract support lines of the branches from the roof
            storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.unionPolygons(support_roof_storage[layer_idx]);
            if (! storage.support.supportLayers[layer_idx].support_roof.empty()
                && support_layer_storage[layer_idx].intersection(storage.support.supportLayers[layer_idx].support_roof).area() > 1)
            {
                switch (interface_pref)
                {
                case InterfacePreference::INTERFACE_AREA_OVERWRITES_SUPPORT:
                    support_layer_storage[layer_idx] = support_layer_storage[layer_idx].difference(storage.support.supportLayers[layer_idx].support_roof);
                    break;

                case InterfacePreference::SUPPORT_AREA_OVERWRITES_INTERFACE:
                    storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.difference(support_layer_storage[layer_idx]);
                    break;

                case InterfacePreference::INTERFACE_LINES_OVERWRITE_SUPPORT:
                {
                    Polygons interface_lines = TreeSupportUtils::generateSupportInfillLines(
                                                   storage.support.supportLayers[layer_idx].support_roof,
                                                   config,
                                                   true,
                                                   layer_idx,
                                                   config.support_roof_line_distance,
                                                   storage.support.cross_fill_provider,
                                                   true)
                                                   .offsetPolyLine(config.support_roof_line_width / 2);
                    support_layer_storage[layer_idx] = support_layer_storage[layer_idx].difference(interface_lines);
                }
                break;

                case InterfacePreference::SUPPORT_LINES_OVERWRITE_INTERFACE:
                {
                    Polygons tree_lines;
                    tree_lines = tree_lines.unionPolygons(TreeSupportUtils::generateSupportInfillLines(
                                                              support_layer_storage[layer_idx],
                                                              config,
                                                              false,
                                                              layer_idx,
                                                              config.support_line_distance,
                                                              storage.support.cross_fill_provider,
                                                              true)
                                                              .offsetPolyLine(config.support_line_width / 2));
                    storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.difference(tree_lines);
                    // Do not draw roof where the tree is. I prefer it this way as otherwise the roof may cut of a branch from its support below.
                }
                break;

                case InterfacePreference::NOTHING:
                    break;
                }
            }

            // Subtract support floors from the support area and add them to the support floor instead.
            if (config.support_bottom_layers > 0 && ! support_layer_storage[layer_idx].empty())
            {
                Polygons floor_layer = storage.support.supportLayers[layer_idx].support_bottom;
                Polygons layer_outset = support_layer_storage[layer_idx].offset(config.support_bottom_offset).difference(volumes_.getCollision(0, layer_idx, false));
                size_t layers_below = 0;
                while (layers_below <= config.support_bottom_layers)
                {
                    // One sample at 0 layers below, another at config.support_bottom_layers. In-between samples at config.performance_interface_skip_layers distance from each
                    // other.
                    const size_t sample_layer
                        = static_cast<size_t>(std::max(0, (static_cast<int>(layer_idx) - static_cast<int>(layers_below)) - static_cast<int>(config.z_distance_bottom_layers)));
                    constexpr bool no_support = false;
                    constexpr bool no_prime_tower = false;
                    floor_layer.add(layer_outset.intersection(storage.getLayerOutlines(sample_layer, no_support, no_prime_tower)));
                    if (layers_below < config.support_bottom_layers)
                    {
                        layers_below = std::min(layers_below + config.performance_interface_skip_layers, config.support_bottom_layers);
                    }
                    else
                    {
                        break;
                    }
                }
                floor_layer = floor_layer.unionPolygons();
                storage.support.supportLayers[layer_idx].support_bottom = storage.support.supportLayers[layer_idx].support_bottom.unionPolygons(floor_layer);
                support_layer_storage[layer_idx] = support_layer_storage[layer_idx].difference(floor_layer.offset(10)); // Subtract the support floor from the normal support.
            }

            constexpr bool convert_every_part = true; // Convert every part into a PolygonsPart for the support.
            storage.support.supportLayers[layer_idx]
                .fillInfillParts(layer_idx, support_layer_storage, config.support_line_width, config.support_wall_count, config.maximum_move_distance, convert_every_part);

            {
                std::lock_guard<std::mutex> critical_section_progress(critical_sections);
                progress_total += TREE_PROGRESS_FINALIZE_BRANCH_AREAS / support_layer_storage.size();
                Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
            }

            {
                std::lock_guard<std::mutex> critical_section_storage(critical_sections);
                if (! storage.support.supportLayers[layer_idx].support_infill_parts.empty() || ! storage.support.supportLayers[layer_idx].support_roof.empty())
                {
                    storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, static_cast<int>(layer_idx));
                }
            }
        });
}

void TreeSupport::drawAreas(std::vector<std::set<TreeSupportElement*>>& move_bounds, SliceDataStorage& storage)
{
    std::vector<Polygons> support_layer_storage(move_bounds.size());
    std::vector<Polygons> support_roof_storage(move_bounds.size());
    std::map<TreeSupportElement*, TreeSupportElement*>
        inverse_tree_order; // In the tree structure only the parents can be accessed. Inverse this to be able to access the children.
    std::vector<std::pair<LayerIndex, TreeSupportElement*>>
        linear_data; // All SupportElements are put into a layer independent storage to improve parallelization. Was added at a point in time where this function had performance
                     // issues. These were fixed by creating less initial points, but i do not see a good reason to remove a working performance optimization.
    for (const auto layer_idx : ranges::views::iota(0UL, move_bounds.size()))
    {
        for (TreeSupportElement* elem : move_bounds[layer_idx])
        {
            // (Check if) We either come from nowhere at the final layer or we had invalid parents 2. should never happen but just to be sure:
            if ((layer_idx > 0
                 && ((! inverse_tree_order.count(elem) && elem->target_height == layer_idx && config.min_dtt_to_model > 0 && ! elem->to_buildplate)
                     || (inverse_tree_order.count(elem) && inverse_tree_order[elem]->result_on_layer == Point(-1, -1)))))
            {
                continue;
            }

            for (TreeSupportElement* par : elem->parents)
            {
                if (par->result_on_layer == Point(-1, -1))
                {
                    continue;
                }
                inverse_tree_order.emplace(par, elem);
            }
            linear_data.emplace_back(layer_idx, elem);
        }
    }


    // Reorder the processed data by layers again. The map also could be a vector<pair<SupportElement*,Polygons>>:
    std::vector<std::unordered_map<TreeSupportElement*, Polygons>> layer_tree_polygons(move_bounds.size());
    const auto t_start = std::chrono::high_resolution_clock::now();

    // Generate the circles that will be the branches.
    generateBranchAreas(linear_data, layer_tree_polygons, inverse_tree_order);
    const auto t_generate = std::chrono::high_resolution_clock::now();

    // In some edge-cases a branch may go through a hole, where the regular radius does not fit. This can result in an apparent jump in branch radius. As such this cases need to be
    // caught and smoothed out.
    smoothBranchAreas(layer_tree_polygons);
    const auto t_smooth = std::chrono::high_resolution_clock::now();

    // Drop down all trees that connect non gracefully with the model.
    std::vector<std::vector<std::pair<LayerIndex, Polygons>>> dropped_down_areas(linear_data.size());
    dropNonGraciousAreas(layer_tree_polygons, linear_data, dropped_down_areas, inverse_tree_order);
    const auto t_drop = std::chrono::high_resolution_clock::now();

    // single threaded combining all dropped down support areas to the right layers. ONLY COPYS DATA!
    for (const coord_t i : ranges::views::iota(0UL, dropped_down_areas.size()))
    {
        for (std::pair<LayerIndex, Polygons> pair : dropped_down_areas[i])
        {
            support_layer_storage[pair.first].add(pair.second);
        }
    }

    // ensure all branch areas added as roof actually cause a roofline to generate. Else disable turning the branch to roof going down
    cura::parallel_for<size_t>(
        0,
        layer_tree_polygons.size(),
        [&](const size_t layer_idx)
        {
            for (std::pair<TreeSupportElement*, Polygons> data_pair : layer_tree_polygons[layer_idx])
            {
                if (data_pair.first->missing_roof_layers > data_pair.first->distance_to_top
                    && TreeSupportUtils::generateSupportInfillLines(data_pair.second, config, true, layer_idx, config.support_roof_line_distance, nullptr, true).empty())
                {
                    std::vector<TreeSupportElement*> to_disable_roofs;
                    to_disable_roofs.emplace_back(data_pair.first);
                    while (! to_disable_roofs.empty())
                    {
                        std::vector<TreeSupportElement*> to_disable_roofs_next;
                        for (TreeSupportElement* elem : to_disable_roofs)
                        {
                            elem->missing_roof_layers = 0;
                            if (data_pair.first->missing_roof_layers > data_pair.first->distance_to_top + 1)
                            {
                                to_disable_roofs_next.emplace_back(inverse_tree_order[elem]);
                            }
                        }
                        to_disable_roofs = to_disable_roofs_next;
                    }
                }
            }
        });

    // Single threaded combining all support areas to the right layers.
    // Only copies data!
    for (const auto layer_idx : ranges::views::iota(0UL, layer_tree_polygons.size()))
    {
        for (std::pair<TreeSupportElement*, Polygons> data_pair : layer_tree_polygons[layer_idx])
        {
            ((data_pair.first->missing_roof_layers > data_pair.first->distance_to_top) ? support_roof_storage : support_layer_storage)[layer_idx].add(data_pair.second);
        }
    }

    for (const auto layer_idx : ranges::views::iota(0UL, additional_required_support_area.size()))
    {
        if (support_layer_storage.size() > layer_idx)
        {
            support_layer_storage[layer_idx].add(additional_required_support_area[layer_idx]);
        }
        scripta::log("tree_support_layer_storage", support_layer_storage[layer_idx], SectionType::SUPPORT, layer_idx);
    }

    filterFloatingLines(support_layer_storage);
    const auto t_filter = std::chrono::high_resolution_clock::now();

    finalizeInterfaceAndSupportAreas(support_layer_storage, support_roof_storage, storage);
    const auto t_end = std::chrono::high_resolution_clock::now();

    const auto dur_gen_tips = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_generate - t_start).count();
    const auto dur_smooth = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_smooth - t_generate).count();
    const auto dur_drop = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_drop - t_smooth).count();
    const auto dur_filter = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_filter - t_drop).count();
    const auto dur_finalize = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_filter).count();
    spdlog::info(
        "Time used for drawing subfuctions: generateBranchAreas: {} ms smoothBranchAreas: {} ms dropNonGraciousAreas: {} ms filterFloatingLines: {} ms "
        "finalizeInterfaceAndSupportAreas {} ms",
        dur_gen_tips,
        dur_smooth,
        dur_drop,
        dur_filter,
        dur_finalize);
}

} // namespace cura
