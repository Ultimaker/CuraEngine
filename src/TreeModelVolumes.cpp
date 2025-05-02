// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TreeModelVolumes.h"

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/reverse.hpp>
#include <spdlog/spdlog.h>

#include "PrimeTower/PrimeTower.h"
#include "TreeSupport.h"
#include "TreeSupportEnums.h"
#include "progress/Progress.h"
#include "sliceDataStorage.h"
#include "utils/ThreadPool.h"
#include "utils/algorithm.h"

namespace cura
{

TreeModelVolumes::TreeModelVolumes(
    const SliceDataStorage& storage,
    const coord_t max_move,
    const coord_t max_move_slow,
    const coord_t min_offset_per_step,
    const coord_t min_radius,
    size_t current_mesh_idx,
    double progress_multiplier,
    double progress_offset,
    const std::vector<Shape>& additional_excluded_areas)
    : max_move_{ std::max(max_move - 2, coord_t(0)) }
    , // -2 to avoid rounding errors
    max_move_slow_{ std::max(max_move_slow - 2, coord_t(0)) }
    , // -2 to avoid rounding errors
    min_offset_per_step_{ min_offset_per_step }
    , radius_0_(min_radius)
    , progress_multiplier_{ progress_multiplier }
    , progress_offset_{ progress_offset }
    , machine_border_{ calculateMachineBorderCollision(storage.getMachineBorder()) }
    , machine_area_{ storage.getMachineBorder() }
    , max_layer_idx_without_anti_preferred_{ storage.support.supportLayers.size() - 1 }
{
    anti_overhang_ = std::vector<Shape>(storage.support.supportLayers.size(), Shape());
    std::unordered_map<size_t, size_t> mesh_to_layeroutline_idx;

    // Get, for all participating meshes, simplification settings, and support settings that can be set per mesh.
    // NOTE: The setting 'support_type' (used here for 'support_rests_on_model' is not settable per mesh, if this stays that way, we could simplify the code a bit.
    //       Currently in the middle of rethinking support, so this stays.

    coord_t min_maximum_resolution = std::numeric_limits<coord_t>::max();
    coord_t min_maximum_deviation = std::numeric_limits<coord_t>::max();
    coord_t min_maximum_area_deviation = std::numeric_limits<coord_t>::max();

    support_rests_on_model_ = false;
    for (auto [mesh_idx, mesh_ptr] : storage.meshes | ranges::views::enumerate)
    {
        auto& mesh = *mesh_ptr;
        bool added = false;
        for (auto [idx, layer_outline] : layer_outlines_ | ranges::views::enumerate)
        {
            if (checkSettingsEquality(layer_outline.first, mesh.settings))
            {
                added = true;
                mesh_to_layeroutline_idx[mesh_idx] = idx;
            }
        }
        if (! added)
        {
            mesh_to_layeroutline_idx[mesh_idx] = layer_outlines_.size();
            layer_outlines_.emplace_back(mesh.settings, std::vector<Shape>(storage.support.supportLayers.size(), Shape()));
        }
    }

    // Figure out the rest of the setting(-like variable)s relevant to the class a whole.
    current_outline_idx_ = mesh_to_layeroutline_idx[current_mesh_idx];
    const TreeSupportSettings config(layer_outlines_[current_outline_idx_].first);

    for (const auto data_pair : layer_outlines_)
    {
        support_rests_on_model_ |= data_pair.first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
        min_maximum_deviation = std::min(min_maximum_deviation, data_pair.first.get<coord_t>("meshfix_maximum_deviation"));
        min_maximum_resolution = std::min(min_maximum_resolution, data_pair.first.get<coord_t>("meshfix_maximum_resolution"));
        min_maximum_area_deviation = std::min(min_maximum_area_deviation, data_pair.first.get<coord_t>("meshfix_maximum_extrusion_area_deviation"));
        max_cradle_dtt = std::max(max_cradle_dtt, config.tip_layers); // todo better estimation
    }

    // Figure out the rest of the setting(-like variable)s relevant to the class a whole.
    current_outline_idx_ = mesh_to_layeroutline_idx[current_mesh_idx];

    if (config.support_overrides == SupportDistPriority::Z_OVERRIDES_XY)
    {
        current_min_xy_dist_ = config.xy_min_distance;

        if (TreeSupportSettings::has_to_rely_on_min_xy_dist_only)
        {
            current_min_xy_dist_ = std::max(current_min_xy_dist_, coord_t(FUDGE_LENGTH * 2));
        }

        current_min_xy_dist_delta_ = std::max(config.xy_distance - current_min_xy_dist_, coord_t(0));
    }
    else
    {
        current_min_xy_dist_ = config.xy_distance;
        current_min_xy_dist_delta_ = 0;
    }
    increase_until_radius_ = config.increase_radius_until_radius;

    // Retrieve all layer outlines. Done in this way because normally we don't do this per mesh, but for the whole buildplate.
    // (So we can handle some settings on a per-mesh basis.)
    for (auto [mesh_idx, mesh] : storage.meshes | ranges::views::enumerate)
    {
        // Workaround for compiler bug on apple-clang -- Closure won't properly capture variables in capture lists in outer scope.
        const auto& mesh_idx_l = mesh_idx;
        const auto& mesh_l = *mesh;
        // ^^^ Remove when fixed (and rename accordingly in the below parallel-for).

        cura::parallel_for<coord_t>(
            0,
            LayerIndex(layer_outlines_[mesh_to_layeroutline_idx[mesh_idx_l]].second.size()),
            [&](const LayerIndex layer_idx)
            {
                if (mesh_l.layer_nr_max_filled_layer < layer_idx)
                {
                    return; // Can't break as parallel_for wont allow it, this is equivalent to a continue.
                }
                Shape outline = extractOutlineFromMesh(mesh_l, layer_idx);
                layer_outlines_[mesh_to_layeroutline_idx[mesh_idx_l]].second[layer_idx].push_back(outline);
            });
    }
    // Merge all the layer outlines together.
    for (auto& layer_outline : layer_outlines_)
    {
        cura::parallel_for<coord_t>(
            0,
            LayerIndex(anti_overhang_.size()),
            [&](const LayerIndex layer_idx)
            {
                layer_outline.second[layer_idx] = layer_outline.second[layer_idx].unionPolygons();
            });
    }

    // Gather all excluded areas, like support-blockers and trees that where already generated.
    cura::parallel_for<coord_t>(
        0,
        LayerIndex(anti_overhang_.size()),
        [&](const LayerIndex layer_idx)
        {
            if (layer_idx < coord_t(additional_excluded_areas.size()))
            {
                anti_overhang_[layer_idx].push_back(additional_excluded_areas[layer_idx]);
            }
            for (const SupportGenerationModifier& support_modifier : storage.support.supportGenerationModifiers)
            {
                if (support_modifier.isAntiSupport() && layer_idx < support_modifier.areas_.size())
                {
                    anti_overhang_[layer_idx].push_back(support_modifier.areas_[layer_idx]);
                }
            }

            if (storage.prime_tower_)
            {
                anti_overhang_[layer_idx].push_back(storage.prime_tower_->getOccupiedOutline(layer_idx));
            }
            anti_overhang_[layer_idx] = anti_overhang_[layer_idx].unionPolygons();
        });

    for (max_layer_idx_without_blocker_ = -1; max_layer_idx_without_blocker_ + 1 < anti_overhang_.size(); max_layer_idx_without_blocker_++)
    {
        if (! anti_overhang_[max_layer_idx_without_blocker_ + 1].empty())
        {
            break;
        }
    }

    // Cache some handy settings in the object itself.
    support_rest_preference_ = config.support_rest_preference;
    simplifier_ = Simplify(min_maximum_resolution, min_maximum_deviation, min_maximum_area_deviation);
}

void TreeModelVolumes::precalculate(LayerIndex max_layer)
{
    const auto t_start = std::chrono::high_resolution_clock::now();
    precalculated_ = true;

    // Get the config corresponding to one mesh that is in the current group. Which one has to be irrelevant.
    // Not the prettiest way to do this, but it ensures some calculations that may be a bit more complex like initial layer diameter are only done in once.
    const TreeSupportSettings config(layer_outlines_[current_outline_idx_].first);

    // Calculate which radius each layer in the tip may have.
    std::unordered_set<coord_t> possible_tip_radiis;
    for (const auto dtt : ranges::views::iota(0UL, config.tip_layers + 1))
    {
        possible_tip_radiis.emplace(ceilRadius(config.getRadius(dtt)));
        possible_tip_radiis.emplace(ceilRadius(config.getRadius(dtt) + current_min_xy_dist_delta_));
    }
    // It theoretically may happen in the tip, that the radius can change so much in-between 2 layers, that a ceil step is skipped (as in there is a radius r so that
    // ceilRadius(radius(dtt))<ceilRadius(r)<ceilRadius(radius(dtt+1))). As such a radius will not reasonable happen in the tree and it will most likely not be requested, there is
    // no need to calculate them. So just skip these.
    for (coord_t radius_eval = ceilRadius(1); radius_eval <= config.branch_radius; radius_eval = ceilRadius(radius_eval + 1))
    {
        if (! possible_tip_radiis.count(radius_eval))
        {
            ignorable_radii_.emplace(radius_eval);
        }
    }

    // Since we possibly have a required max/min size branches can be on the build-plate, and also of course a restricted rate at wich a radius normally is altered,
    //   (also) pre-calculate the restriction(s) on the radius at each layer which maximum these restrictions impose.

    // It may seem that the required avoidance can be of a smaller radius when going to model (no initial layer diameter for to model branches)
    // but as for every branch going towards the bp, the to model avoidance is required to check for possible merges with to model branches, this assumption is in-fact wrong.
    std::unordered_map<coord_t, LayerIndex> radius_until_layer;
    // while it is possible to calculate, up to which layer the avoidance should be calculated, this simulation is easier to understand, and does not need to be adjusted if
    // something of the radius calculation is changed. Tested overhead was negligible (milliseconds for thousands of layers).
    for (LayerIndex simulated_dtt = 0; simulated_dtt <= max_layer; simulated_dtt++)
    {
        const LayerIndex current_layer = max_layer - simulated_dtt;
        const coord_t max_regular_radius = ceilRadius(config.getRadius(simulated_dtt, 0) + current_min_xy_dist_delta_);
        const coord_t max_min_radius = ceilRadius(config.getRadius(simulated_dtt, 0)); // the maximal radius that the radius with the min_xy_dist can achieve
        const coord_t max_initial_layer_diameter_radius = ceilRadius(config.recommendedMinRadius(current_layer) + current_min_xy_dist_delta_);
        if (! radius_until_layer.count(max_regular_radius))
        {
            radius_until_layer[max_regular_radius] = std::min(current_layer + max_cradle_dtt, max_layer);
        }
        if (! radius_until_layer.count(max_min_radius))
        {
            radius_until_layer[max_min_radius] = std::min(current_layer + max_cradle_dtt, max_layer);
        }

        // all radiis between max_min_radius and max_initial_layer_diameter_radius can also occur
        coord_t current_ceil_radius = max_min_radius;
        while (current_ceil_radius < max_initial_layer_diameter_radius)
        {
            current_ceil_radius = ceilRadius(current_ceil_radius + 1);
            if (! radius_until_layer.count(current_ceil_radius))
            {
                radius_until_layer[current_ceil_radius] = std::min(current_layer + max_cradle_dtt, max_layer);
            }
        }
    }

    // Copy these deques, as the methods we provide them to will loop over them using parallel-for.
    // NOTE: While it might seem that one of these could be removed, they are here in case the parallel loop becomes no-wait.
    std::deque<RadiusLayerPair> relevant_avoidance_radiis;
    std::deque<RadiusLayerPair> relevant_avoidance_radiis_to_model;
    relevant_avoidance_radiis.insert(relevant_avoidance_radiis.end(), radius_until_layer.begin(), radius_until_layer.end());
    relevant_avoidance_radiis_to_model.insert(relevant_avoidance_radiis_to_model.end(), radius_until_layer.begin(), radius_until_layer.end());
    precalculated_avoidance_radii.insert(precalculated_avoidance_radii.end(), radius_until_layer.begin(), radius_until_layer.end());

    // Append additional radiis needed for collision.

    radius_until_layer[ceilRadius(increase_until_radius_, false)]
        = max_layer; // To calculate collision holefree for every radius, the collision of radius increase_until_radius will be required.
    // Collision for radius 0 needs to be calculated everywhere, as it will be used to ensure valid xy_distance in drawAreas.
    radius_until_layer[0] = max_layer;
    if (current_min_xy_dist_delta_ != 0)
    {
        radius_until_layer[current_min_xy_dist_delta_] = max_layer;
    }

    std::deque<RadiusLayerPair> relevant_collision_radiis;
    relevant_collision_radiis.insert(
        relevant_collision_radiis.end(),
        radius_until_layer.begin(),
        radius_until_layer.end()); // Now that required_avoidance_limit contains the maximum of old and regular required radius just copy.

    // ### Calculate the relevant collisions
    calculateCollision(relevant_collision_radiis);

    // Calculate a separate Collisions with all holes removed. These are relevant for some avoidances that try to avoid holes (called safe).
    std::deque<RadiusLayerPair> relevant_hole_collision_radiis;
    for (RadiusLayerPair key : relevant_avoidance_radiis)
    {
        spdlog::debug("Calculating avoidance of radius {} up to layer {}", key.first, key.second);
        if (key.first < increase_until_radius_ + current_min_xy_dist_delta_)
        {
            relevant_hole_collision_radiis.emplace_back(key);
        }
    }

    // ### Calculate collisions without holes, build from regular collision
    calculateCollisionHolefree(relevant_hole_collision_radiis);

    const auto t_coll = std::chrono::high_resolution_clock::now();
    auto t_acc = std::chrono::high_resolution_clock::now();


    if (max_layer_idx_without_blocker_ < max_layer && support_rests_on_model_)
    {
        calculateAccumulatedPlaceable0(max_layer);
        t_acc = std::chrono::high_resolution_clock::now();
    }

    // ### Calculate the relevant avoidances in parallel as far as possible
    {
        std::future<void> placeable_waiter;
        std::future<void> avoidance_waiter;

        if (support_rests_on_model_)
        {
            calculatePlaceables(relevant_avoidance_radiis_to_model);
        }
        if (support_rest_preference_ == RestPreference::BUILDPLATE)
        {
            calculateAvoidance(relevant_avoidance_radiis);
        }

        calculateWallRestrictions(relevant_avoidance_radiis);
        if (support_rests_on_model_)
        {
            // FIXME: When nowait (parellel-for) is implemented, ensure here the following is calculated: calculatePlaceables.
            calculateAvoidanceToModel(relevant_avoidance_radiis_to_model);
            // FIXME: When nowait (parellel-for) is implemented, ensure here the following is calculated: calculateAvoidanceToModel.
        }
        if (support_rest_preference_ == RestPreference::BUILDPLATE)
        {
            // FIXME: When nowait (parellel-for) is implemented, ensure here the following is calculated: calculateAvoidance.
        }
        // FIXME: When nowait (parellel-for) is implemented, ensure here the following is calculated: calculateWallRestrictions.
    }
    const auto t_avo = std::chrono::high_resolution_clock::now();

    auto t_colAvo = std::chrono::high_resolution_clock::now();
    if (max_layer_idx_without_blocker_ < max_layer && support_rests_on_model_)
    {
        // FIXME: When nowait (parellel-for) is implemented, ensure here the following is calculated: calculateAccumulatedPlaceable0.
        calculateCollisionAvoidance(relevant_avoidance_radiis);
        t_colAvo = std::chrono::high_resolution_clock::now();
    }

    calculateFake0Avoidances(max_layer);

    precalculation_finished_ = true;
    const auto dur_col = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_coll - t_start).count();
    const auto dur_acc = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_acc - t_coll).count();
    const auto dur_avo = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_avo - t_acc).count();
    const auto dur_col_avo = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_colAvo - t_avo).count();


    spdlog::info(
        "Pre-calculating collision took {} ms. Pre-calculating avoidance took {} ms. Pre-calculating accumulated Placeables with radius 0 took {} ms. Pre-calculating "
        "collision-avoidance took {} ms. ",
        dur_col,
        dur_avo,
        dur_acc,
        dur_col_avo);
}

const Shape& TreeModelVolumes::getCollision(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    const coord_t orig_radius = radius;
    std::optional<std::reference_wrapper<const Shape>> result;
    if (! min_xy_dist)
    {
        radius += current_min_xy_dist_delta_;
    }

    // special case as if a radius 0 is requested it could be to ensure correct xy distance. As such it is beneficial if the collision is as close to the configured values as
    // possible.
    if (orig_radius != 0)
    {
        radius = ceilRadius(radius);
    }
    RadiusLayerPair key{ radius, layer_idx };

    {
        std::lock_guard<std::mutex> critical_critical_collision_cache_(*critical_collision_cache_);
        result = getArea(collision_cache_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated_)
    {
        spdlog::warn("Had to calculate collision at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
    }

    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::COLLISION));
        calculateCollision(key);
    }
    return getCollision(orig_radius, layer_idx, min_xy_dist);
}

const Shape& TreeModelVolumes::getCollisionHolefree(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    const coord_t orig_radius = radius;
    std::optional<std::reference_wrapper<const Shape>> result;
    if (! min_xy_dist)
    {
        radius += current_min_xy_dist_delta_;
    }
    if (radius >= increase_until_radius_ + current_min_xy_dist_delta_)
    {
        return getCollision(orig_radius, layer_idx, min_xy_dist);
    }
    RadiusLayerPair key{ radius, layer_idx };

    {
        std::lock_guard<std::mutex> critical_section_support_max_layer_nr(*critical_collision_cache_holefree_);
        result = getArea(collision_cache_holefree_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated_)
    {
        spdlog::warn("Had to calculate collision holefree at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
    }
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::COLLISION_HOLEFREE));
        calculateCollisionHolefree(key);
    }
    return getCollisionHolefree(orig_radius, layer_idx, min_xy_dist);
}

const Shape& TreeModelVolumes::getAccumulatedPlaceable0(LayerIndex layer_idx)
{
    {
        std::lock_guard<std::mutex> critical_section_support_max_layer_nr(*critical_accumulated_placeables_cache_radius_0_);
        if (accumulated_placeables_cache_radius_0_.count(layer_idx))
        {
            return accumulated_placeables_cache_radius_0_[layer_idx];
        }
    }
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(0, CalculationType::PLACEABLE_ACCUMULATED));
        calculateAccumulatedPlaceable0(layer_idx);
    }
    return getAccumulatedPlaceable0(layer_idx);
}

const Shape& TreeModelVolumes::getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist)
{
    if (layer_idx == 0) // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
    {
        return getCollision(radius, layer_idx, min_xy_dist);
    }

    const coord_t orig_radius = radius;

    std::optional<std::reference_wrapper<const Shape>> result;

    radius += (min_xy_dist ? 0 : current_min_xy_dist_delta_);
    radius = ceilRadius(radius);

    if (radius >= increase_until_radius_ + current_min_xy_dist_delta_ && type == AvoidanceType::FAST_SAFE) // no holes anymore by definition at this request
    {
        type = AvoidanceType::FAST;
    }

    const RadiusLayerPair key{ radius, layer_idx };

    std::unordered_map<RadiusLayerPair, Shape>* cache_ptr = nullptr;
    std::mutex* mutex_ptr = nullptr;
    switch (type)
    {
    case AvoidanceType::FAST:
        cache_ptr = to_model ? &avoidance_cache_to_model_ : &avoidance_cache_;
        mutex_ptr = to_model ? critical_avoidance_cache_to_model_.get() : critical_avoidance_cache_.get();
        break;
    case AvoidanceType::SLOW:
        cache_ptr = to_model ? &avoidance_cache_to_model_slow_ : &avoidance_cache_slow_;
        mutex_ptr = to_model ? critical_avoidance_cache_to_model_slow_.get() : critical_avoidance_cache_slow_.get();
        break;
    case AvoidanceType::FAST_SAFE:
        cache_ptr = to_model ? &avoidance_cache_hole_to_model_ : &avoidance_cache_hole_;
        mutex_ptr = to_model ? critical_avoidance_cache_holefree_to_model_.get() : critical_avoidance_cache_holefree_.get();
        break;
    case AvoidanceType::COLLISION:
        if (layer_idx <= max_layer_idx_without_blocker_)
        {
            return getCollision(radius, layer_idx, true);
        }
        else
        {
            cache_ptr = &avoidance_cache_collision_;
            mutex_ptr = critical_avoidance_cache_collision_.get();
        }
        break;
    default:
        spdlog::error("Invalid Avoidance Request");
        break;
    }

    {
        std::lock_guard<std::mutex> critical_section(*mutex_ptr);
        result = getArea(*cache_ptr, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated_)
    {
        spdlog::warn(
            "Had to calculate Avoidance (to model-bool: {}) at radius {} and layer {} and type {}, but precalculate was called. Performance may suffer!",
            to_model,
            key.first,
            key.second,
            coord_t(type));
    }

    if (orig_radius == 0)
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::AVOIDANCE_0));
        calculateFake0Avoidances(layer_idx);
    }
    else if (type == AvoidanceType::COLLISION)
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::AVOIDANCE_COLLISION));
        calculateCollisionAvoidance(key);
    }
    else if (to_model)
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::AVOIDANCE_TO_MODEL));
        calculateAvoidanceToModel(key);
    }
    else
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::AVOIDANCE));
        calculateAvoidance(key);
    }
    return getAvoidance(orig_radius, layer_idx, type, to_model, min_xy_dist); // retrive failed and correct result was calculated. Now it has to be retrived.
}

const Shape& TreeModelVolumes::getPlaceableAreas(coord_t radius, LayerIndex layer_idx)
{
    std::optional<std::reference_wrapper<const Shape>> result;
    const coord_t orig_radius = radius;
    radius = ceilRadius(radius);
    RadiusLayerPair key{ radius, layer_idx };
    {
        std::lock_guard<std::mutex> critical_section(*critical_placeable_areas_cache_);
        result = getArea(placeable_areas_cache_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated_)
    {
        spdlog::warn("Had to calculate Placeable Areas at radius {} and layer {}, but precalculate was called. Performance may suffer!", radius, layer_idx);
    }
    if (radius != 0)
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::PLACEABLE));
        calculatePlaceables(key);
    }
    else
    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::COLLISION));
        calculateCollision(key);
    }
    return getPlaceableAreas(orig_radius, layer_idx);
}


const Shape& TreeModelVolumes::getWallRestriction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    if (layer_idx == 0) // Should never be requested as there will be no going below layer 0 ..., but just to be sure some semi-sane catch. Alternative would be empty Polygon.
    {
        return getCollision(radius, layer_idx, min_xy_dist);
    }

    const coord_t orig_radius = radius;
    min_xy_dist = min_xy_dist && current_min_xy_dist_delta_ > 0;

    std::optional<std::reference_wrapper<const Shape>> result;

    radius = ceilRadius(radius);
    const RadiusLayerPair key{ radius, layer_idx };

    std::unordered_map<RadiusLayerPair, Shape>* cache_ptr = min_xy_dist ? &wall_restrictions_cache_min_ : &wall_restrictions_cache_;
    {
        std::lock_guard<std::mutex> critical_section(min_xy_dist ? *critical_wall_restrictions_cache_min_ : *critical_wall_restrictions_cache_);
        result = getArea(*cache_ptr, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated_)
    {
        spdlog::warn("Had to calculate Wall restrictions at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
    }

    {
        std::lock_guard<std::mutex> critical_section(*getLockForCalculation(radius, CalculationType::WALL_RESTRICTION));
        calculateWallRestrictions(key);
    }
    return getWallRestriction(orig_radius, layer_idx, min_xy_dist); // Retrieve failed and correct result was calculated. Now it has to be retrieved.
}


void TreeModelVolumes::addAreaToAntiPreferred(const Shape area, LayerIndex layer_idx)
{
    RadiusLayerPair key(0, layer_idx);
    std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_);
    anti_preferred_[key] = anti_preferred_[key].unionPolygons(area);
    max_layer_idx_without_anti_preferred_ = std::min(max_layer_idx_without_anti_preferred_, layer_idx - 1);
}


void TreeModelVolumes::precalculateAntiPreferred()
{
    const TreeSupportSettings config(layer_outlines_[current_outline_idx_].first);

    cura::parallel_for<size_t>(
        0,
        precalculated_avoidance_radii.size(),
        [&, precalculated_avoidance_radiis = precalculated_avoidance_radii](const size_t key_idx)
        {
            const coord_t radius = precalculated_avoidance_radiis[key_idx].first;
            const LayerIndex max_required_layer = precalculated_avoidance_radiis[key_idx].second;

            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist_ * 1.9);
            RadiusLayerPair key(radius, 0);

            Shape latest_avoidance;
            Shape latest_avoidance_to_model;
            Shape latest_avoidance_collision;

            LayerIndex start_layer = 0;
            std::vector<std::pair<RadiusLayerPair, Shape>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));
            std::vector<std::pair<RadiusLayerPair, Shape>> data_to_model(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));
            std::vector<std::pair<RadiusLayerPair, Shape>> data_collision(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));
            std::vector<std::pair<RadiusLayerPair, Shape>> data_raw_anti(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));

            bool encountered_anti = false;
            // ### main loop doing the calculation
            for (const LayerIndex layer : ranges::views::iota(static_cast<size_t>(start_layer), max_required_layer + 1UL))
            {
                key.second = layer;
                RadiusLayerPair key_0(0, layer);
                Shape anti;
                {
                    std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_);
                    anti = anti_preferred_[key_0];
                }

                if (! encountered_anti && ! anti.empty())
                {
                    encountered_anti = true;
                    if (support_rest_preference_ == RestPreference::BUILDPLATE)
                    {
                        latest_avoidance = getAvoidance(radius, layer, AvoidanceType::FAST_SAFE, false, true);
                    }
                    if (support_rests_on_model_)
                    {
                        latest_avoidance_to_model = getAvoidance(radius, layer, AvoidanceType::FAST_SAFE, true, true);
                    }
                    if (max_layer_idx_without_blocker_ <= layer && support_rests_on_model_)
                    {
                        latest_avoidance_collision = getAvoidance(radius, layer, AvoidanceType::COLLISION, true, true);
                    }

                    std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_);
                    max_layer_idx_without_anti_preferred_ = layer - 1;
                }
                if (! encountered_anti)
                {
                    continue;
                }

                Shape col = getCollisionHolefree(radius, layer, true);
                anti = anti.unionPolygons().offset(std::max(radius, config.branch_radius)).unionPolygons();
                data_raw_anti[layer] = std::pair<RadiusLayerPair, Shape>(key, anti);

                if (support_rest_preference_ == RestPreference::BUILDPLATE)
                {
                    latest_avoidance = safeOffset(latest_avoidance, -max_move_, ClipperLib::jtRound, -max_step_move, col.unionPolygons(anti));
                    Shape next_latest_avoidance = simplifier_.polygon(latest_avoidance);
                    latest_avoidance = next_latest_avoidance.unionPolygons(latest_avoidance);
                    latest_avoidance = latest_avoidance.unionPolygons(getAvoidance(radius, layer, AvoidanceType::FAST_SAFE, false, true));
                    data[layer] = std::pair<RadiusLayerPair, Shape>(key, latest_avoidance);
                }

                if (support_rests_on_model_)
                {
                    latest_avoidance_to_model = safeOffset(latest_avoidance_to_model, -max_move_, ClipperLib::jtRound, -max_step_move, col.unionPolygons(anti));
                    Shape next_latest_avoidance_to_model = simplifier_.polygon(latest_avoidance_to_model);
                    latest_avoidance_to_model = next_latest_avoidance_to_model.unionPolygons(latest_avoidance_to_model);
                    latest_avoidance_to_model = latest_avoidance_to_model.difference(getPlaceableAreas(radius, layer));
                    latest_avoidance_to_model = latest_avoidance_to_model.unionPolygons(getAvoidance(radius, layer, AvoidanceType::FAST_SAFE, true, true));
                    data_to_model[layer] = std::pair<RadiusLayerPair, Shape>(key, latest_avoidance_to_model);
                }

                if (max_layer_idx_without_blocker_ <= layer && support_rests_on_model_)
                {
                    latest_avoidance_collision = safeOffset(latest_avoidance_collision, -max_move_, ClipperLib::jtRound, -max_step_move, col.unionPolygons(anti));
                    Shape placeable0RadiusCompensated = getAccumulatedPlaceable0(layer).offset(-std::max(radius, increase_until_radius_), ClipperLib::jtRound);
                    latest_avoidance_collision = latest_avoidance_collision.difference(placeable0RadiusCompensated).unionPolygons(getCollision(radius, layer, true));
                    latest_avoidance_collision = latest_avoidance_collision.unionPolygons(getAvoidance(radius, layer, AvoidanceType::COLLISION, true, true));
                    data_collision[layer] = std::pair<RadiusLayerPair, Shape>(key, latest_avoidance_collision);
                }
            }
            {
                std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_caches);
                anti_preferred_cache_.insert(data.begin(), data.end());
                anti_preferred_cache_to_model_.insert(data_to_model.begin(), data_to_model.end());
                anti_preferred_cache_collision.insert(data_collision.begin(), data_collision.end());
            }
            {
                std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_);
                anti_preferred_.insert(data_raw_anti.begin(), data_raw_anti.end());
            }
        });
}

const Shape& TreeModelVolumes::getAntiPreferredAreas(LayerIndex layer_idx, coord_t radius)
{
    coord_t ceiled_radius = ceilRadius(radius);
    RadiusLayerPair key(ceilRadius(ceiled_radius), layer_idx);
    std::optional<std::reference_wrapper<const Shape>> result;

    std::unordered_map<RadiusLayerPair, Shape>* cache_ptr = &anti_preferred_;
    {
        std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_caches);
        result = getArea(*cache_ptr, key);
    }

    if (result)
    {
        return result.value().get();
    }

    {
        key.first = 0;
        std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_);
        result = getArea(anti_preferred_, key);
    }

    if (! result || result.value().get().empty()) // todo where and why are empty areas inserted?
    {
        return empty_polygon;
    }

    if (precalculated_)
    {
        spdlog::warn("Missing anti preferred area at radius {} and layer {}  Returning Empty! Result had area of {}", ceiled_radius, key.second, result.value().get().area());
    }
    return empty_polygon;
}


const Shape& TreeModelVolumes::getAntiPreferredAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist)
{
    coord_t ceiled_radius = ceilRadius(radius, min_xy_dist);
    RadiusLayerPair key(ceilRadius(ceiled_radius), layer_idx);
    std::optional<std::reference_wrapper<const Shape>> result;

    std::unordered_map<RadiusLayerPair, Shape>* cache_ptr = nullptr;

    if (type == AvoidanceType::COLLISION)
    {
        if (max_layer_idx_without_blocker_ <= layer_idx && to_model)
        {
            cache_ptr = &anti_preferred_cache_collision;
        }
        else
        {
            cache_ptr = &anti_preferred_;
        }
    }
    else if (to_model)
    {
        cache_ptr = &anti_preferred_cache_to_model_;
    }
    else
    {
        cache_ptr = &anti_preferred_cache_;
    }

    {
        std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_caches);
        result = getArea(*cache_ptr, key);
    }
    if (result)
    {
        return result.value().get();
    }

    {
        key.first = 0;
        std::lock_guard<std::mutex> critical_section(*critical_anti_preferred_);
        result = getArea(anti_preferred_, key);
    }

    if (! result || result.value().get().empty())
    {
        return getAvoidance(radius, layer_idx, type, to_model, min_xy_dist);
    }

    if (precalculated_)
    {
        spdlog::warn(
            "Missing anti preferred calculated at radius {} and layer {} and type {} to model {}, but precalculate was called. Returning Empty!",
            ceiled_radius,
            key.second,
            type == AvoidanceType::COLLISION,
            to_model);
    }
    return getAvoidance(radius, layer_idx, type, to_model, min_xy_dist);
}

const Shape& TreeModelVolumes::getSupportBlocker(LayerIndex layer_idx)
{
    if (layer_idx < anti_overhang_.size())
    {
        return anti_overhang_[layer_idx];
    }
    else
    {
        return empty_polygon;
    }
}


coord_t TreeModelVolumes::ceilRadius(coord_t radius, bool min_xy_dist) const
{
    return ceilRadius(radius + (min_xy_dist ? 0 : current_min_xy_dist_delta_));
}

coord_t TreeModelVolumes::getRadiusNextCeil(coord_t radius, bool min_xy_dist) const
{
    return ceilRadius(radius, min_xy_dist) - (min_xy_dist ? 0 : current_min_xy_dist_delta_);
}

LayerIndex TreeModelVolumes::getMaxLayerIdxWithoutAntiPreferred()
{
    return max_layer_idx_without_anti_preferred_;
}

LayerIndex TreeModelVolumes::getMaxLayerIdxWithoutBlocker()
{
    return max_layer_idx_without_blocker_;
}

bool TreeModelVolumes::checkSettingsEquality(const Settings& me, const Settings& other) const
{
    return TreeSupportSettings(me) == TreeSupportSettings(other);
}

Shape TreeModelVolumes::extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx) const
{
    // Similar to SliceDataStorage.getLayerOutlines but only for one mesh instead of for all of them.

    constexpr bool external_polys_only = false;
    Shape total;

    if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
    {
        return Shape();
    }
    const SliceLayer& layer = mesh.layers[layer_idx];

    layer.getOutlines(total, external_polys_only);
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
    {
        total = total.unionPolygons(layer.open_polylines.offset(FUDGE_LENGTH * 2));
    }
    const coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t maximum_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    const coord_t maximum_area_deviation = mesh.settings.get<coord_t>("meshfix_maximum_extrusion_area_deviation");
    return Simplify(maximum_resolution, maximum_deviation, maximum_area_deviation).polygon(total);
}

LayerIndex TreeModelVolumes::getMaxCalculatedLayer(coord_t radius, const std::unordered_map<RadiusLayerPair, Shape>& map) const
{
    LayerIndex max_layer = -1;

    // the placeable on model areas do not exist on layer 0, as there can not be model below it. As such it may be possible that layer 1 is available, but layer 0 does not exist.
    const RadiusLayerPair key_layer_1(radius, 1);
    if (getArea(map, key_layer_1))
    {
        max_layer = 1;
    }

    while (map.count(RadiusLayerPair(radius, max_layer + 1)))
    {
        max_layer++;
    }

    return max_layer;
}

void TreeModelVolumes::calculateCollision(const std::deque<RadiusLayerPair>& keys)
{
    cura::parallel_for<size_t>(
        0,
        keys.size(),
        [&](const size_t i)
        {
            const coord_t radius = keys[i].first;
            RadiusLayerPair key(radius, 0);
            std::unordered_map<RadiusLayerPair, Shape> data_outer;
            std::unordered_map<RadiusLayerPair, Shape> data_placeable_outer;
            for (const auto outline_idx : ranges::views::iota(0UL, layer_outlines_.size()))
            {
                std::unordered_map<RadiusLayerPair, Shape> data;
                std::unordered_map<RadiusLayerPair, Shape> data_placeable;

                const coord_t layer_height = layer_outlines_[outline_idx].first.get<coord_t>("layer_height");
                const bool support_rests_on_this_model = layer_outlines_[outline_idx].first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
                const coord_t z_distance_bottom = layer_outlines_[outline_idx].first.get<coord_t>("support_bottom_distance");
                const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom, layer_height);
                const coord_t z_distance_top_layers = round_up_divide(layer_outlines_[outline_idx].first.get<coord_t>("support_top_distance"), layer_height);
                const LayerIndex max_anti_overhang_layer = anti_overhang_.size() - 1;
                const LayerIndex max_required_layer = keys[i].second + std::max(coord_t(1), z_distance_top_layers);
                const coord_t xy_distance = outline_idx == current_outline_idx_ ? current_min_xy_dist_ : layer_outlines_[outline_idx].first.get<coord_t>("support_xy_distance");
                // Technically this causes collision for the normal xy_distance to be larger by current_min_xy_dist_delta for all not currently processing meshes as this delta will
                // be added at request time. Avoiding this would require saving each collision for each outline_idx separately,
                // and later for each avoidance... But avoidance calculation has to be for the whole scene and can NOT be done for each outline_idx separately and combined later.
                // So avoiding this inaccuracy seems infeasible as it would require 2x the avoidance calculations => 0.5x the performance.
                coord_t min_layer_bottom;
                coord_t min_layer_insert;
                {
                    std::lock_guard<std::mutex> critical_section(*critical_collision_cache_);
                    min_layer_insert = getMaxCalculatedLayer(radius, collision_cache_) + 1;
                    min_layer_bottom = min_layer_insert - z_distance_bottom_layers;
                }

                if (min_layer_bottom < 0)
                {
                    min_layer_bottom = 0;
                }

                if (max_required_layer < min_layer_bottom)
                {
                    continue;
                }

                for (const auto layer_idx : ranges::views::iota(min_layer_bottom, max_required_layer + 1))
                {
                    key.second = layer_idx;
                    Shape collision_areas = machine_border_;
                    if (size_t(layer_idx) < layer_outlines_[outline_idx].second.size())
                    {
                        collision_areas.push_back(layer_outlines_[outline_idx].second[layer_idx]);
                    }
                    collision_areas = collision_areas.offset(
                        radius
                        + xy_distance); // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                    data[key].push_back(collision_areas); // if a key does not exist when it is accessed it is added!
                }

                // Add layers below, to ensure correct support_bottom_distance. Also save placeable areas of radius 0, if required for this mesh.
                for (const auto layer_idx : ranges::views::iota(min_layer_bottom, max_required_layer + 1) | ranges::views::reverse)
                {
                    key.second = layer_idx;
                    for (size_t layer_offset = 1; layer_offset <= z_distance_bottom_layers && layer_idx - coord_t(layer_offset) > min_layer_bottom; layer_offset++)
                    {
                        data[key].push_back(data[RadiusLayerPair(radius, layer_idx - layer_offset)]);
                    }
                    // Placeable areas also have to be calculated when a collision has to be calculated if called outside of precalculate to prevent an infinite loop when they are
                    // invalidly requested...
                    if ((support_rests_on_this_model || precalculation_finished_ || ! precalculated_) && radius == 0 && layer_idx < coord_t(1 + keys[i].second))
                    {
                        data[key] = data[key].unionPolygons();
                        Shape above = data[RadiusLayerPair(radius, layer_idx + 1)];
                        above = above.unionPolygons(max_anti_overhang_layer >= layer_idx + 1 ? anti_overhang_[layer_idx] : Shape());
                        // Empty polygons on condition: Just to be sure the area is correctly unioned as otherwise difference may behave unexpectedly.

                        Shape placeable = data[key].unionPolygons().difference(above);
                        data_placeable[RadiusLayerPair(radius, layer_idx + 1)] = data_placeable[RadiusLayerPair(radius, layer_idx + 1)].unionPolygons(placeable);
                    }
                }

                // Add collision layers above to ensure correct support_top_distance.
                for (const auto layer_idx : ranges::views::iota(min_layer_bottom, max_required_layer + 1))
                {
                    key.second = layer_idx;
                    for (coord_t layer_offset = 1; layer_offset <= z_distance_top_layers
                                                   && layer_offset + layer_idx < std::min(coord_t(layer_outlines_[outline_idx].second.size()), coord_t(max_required_layer + 1));
                         layer_offset++)
                    {
                        // If just the collision (including the xy distance) of the layers above is accumulated, it leads to the following issue:
                        // Example: assuming the z distance is 2 layer
                        // + = xy_distance
                        // - = model
                        // o = overhang of the area two layers above that should result in tips on this layer
                        //
                        //  +-----+
                        //   +-----+
                        //    +-----+
                        //   o +-----+
                        // If just the collision above is accumulated the overhang will get overwritten by the xy_distance of the layer below the overhang...
                        //
                        // This only causes issues if the overhang area is thinner than xy_distance
                        // Just accumulating areas of the model above without the xy distance is also problematic, as then support may get closer to the model (on the diagonal
                        // downwards) than the user intended. Example (s = support):
                        //  +-----+
                        //   +-----+
                        //   +-----+
                        //   s+-----+

                        // Technically the calculation below is off by one layer, as the actual distance between plastic one layer down is 0 not layer height, as this layer is
                        // filled with said plastic. But otherwise a part of the overhang that is expected to be supported is overwritten by the remaining part of the xy distance
                        // of the layer below the to be supported area.
                        const coord_t required_range_x = coord_t(xy_distance - ((layer_offset - (z_distance_top_layers == 1 ? 0.5 : 0)) * xy_distance / z_distance_top_layers));
                        // ^^^ The conditional -0.5 ensures that plastic can never touch on the diagonal downward when the z_distance_top_layers = 1.
                        //      It is assumed to be better to not support an overhang<90� than to risk fusing to it.
                        data[key].push_back(layer_outlines_[outline_idx].second[layer_idx + layer_offset].offset(radius + required_range_x));
                    }
                    data[key] = data[key].unionPolygons(max_anti_overhang_layer >= layer_idx ? anti_overhang_[layer_idx].offset(radius) : Shape());
                }

                for (const auto layer_idx : ranges::views::iota(static_cast<size_t>(keys[i].second) + 1UL, max_required_layer + 1UL))
                {
                    data.erase(RadiusLayerPair(radius, layer_idx)); // all these dont have the correct z_distance_top_layers as they can still have areas above them
                }
                for (const auto layer_idx : ranges::views::iota(min_layer_bottom, min_layer_insert))
                {
                    // Those were already inserted
                    data.erase(RadiusLayerPair(radius, layer_idx));
                    data_placeable.erase(RadiusLayerPair(radius, layer_idx));
                }

                for (auto pair : data)
                {
                    pair.second = simplifier_.polygon(pair.second);
                    data_outer[pair.first] = data_outer[pair.first].unionPolygons(pair.second);
                }
                if (radius == 0)
                {
                    for (auto pair : data_placeable)
                    {
                        pair.second = simplifier_.polygon(pair.second);
                        data_placeable_outer[pair.first] = data_placeable_outer[pair.first].unionPolygons(pair.second);
                    }
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_progress_);

                if (precalculated_ && precalculation_progress_ < TREE_PROGRESS_PRECALC_COLL)
                {
                    precalculation_progress_ += TREE_PROGRESS_PRECALC_COLL / keys.size();
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress_ * progress_multiplier_ + progress_offset_, TREE_PROGRESS_TOTAL);
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_collision_cache_);
                bool added_requested = false;
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data_outer)
                {
                    added_requested |= ins_elem.first.second == keys[i].second;
                    if (collision_cache_.contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated collision for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                    }
                    else
                    {
                        collision_cache_[ins_elem.first] = ins_elem.second;
                    }
                }
            }
            if (radius == 0)
            {
                {
                    std::lock_guard<std::mutex> critical_section(*critical_placeable_areas_cache_);
                    for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data_placeable_outer)
                    {
                        if (placeable_areas_cache_.contains(ins_elem.first))
                        {
                            spdlog::warn("Recalculated placeable radius 0 for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                        }
                        else
                        {
                            placeable_areas_cache_[ins_elem.first] = ins_elem.second;
                        }
                    }
                }
            }
        });
}

void TreeModelVolumes::calculateCollisionHolefree(const std::deque<RadiusLayerPair>& keys)
{
    std::vector<LayerIndex> min_layer_per_key(keys.size());
    LayerIndex min_layer = 0;
    LayerIndex max_layer = 0;
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
        max_layer = std::max(max_layer, keys[i].second);
        LayerIndex start_layer;
        {
            std::lock_guard<std::mutex> critical_section(*critical_collision_cache_holefree_);
            start_layer = 1 + getMaxCalculatedLayer(keys[i].first, collision_cache_holefree_);
        }
        min_layer = std::max(min_layer, start_layer);
        min_layer_per_key[i] = start_layer;
    }

    cura::parallel_for<coord_t>(
        min_layer,
        LayerIndex(max_layer + 1),
        [&](const LayerIndex layer_idx)
        {
            std::unordered_map<RadiusLayerPair, Shape> data;
            for (auto [key_idx, key] : keys | ranges::views::enumerate)
            {
                if (layer_idx < min_layer_per_key[key_idx])
                {
                    continue;
                }
                // Logically increase the collision by increase_until_radius
                const coord_t radius = key.first;
                const coord_t increase_radius_ceil = ceilRadius(increase_until_radius_, false) - ceilRadius(radius, true);
                Shape col = getCollision(increase_until_radius_, layer_idx, false).offset(EPSILON - increase_radius_ceil, ClipperLib::jtRound).unionPolygons();
                // ^^^ That last 'unionPolygons' is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
                col = simplifier_.polygon(col);
                data[RadiusLayerPair(radius, layer_idx)] = col;
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_collision_cache_holefree_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (collision_cache_holefree_.contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated collision holefree for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                    }
                    else
                    {
                        collision_cache_holefree_[ins_elem.first] = ins_elem.second;
                    }
                }
            }
        });
}

void TreeModelVolumes::calculateAccumulatedPlaceable0(const LayerIndex max_layer)
{
    LayerIndex start_layer = -1;

    // the placeable on model areas do not exist on layer 0, as there can not be model below it. As such it may be possible that layer 1 is available, but layer 0 does not exist.
    {
        std::lock_guard<std::mutex> critical_section(*critical_accumulated_placeables_cache_radius_0_);
        while (accumulated_placeables_cache_radius_0_.count(start_layer + 1))
        {
            start_layer++;
        }
        start_layer = std::max(LayerIndex{ start_layer + 1 }, LayerIndex{ 1 });
    }
    if (start_layer > max_layer)
    {
        spdlog::debug("Requested calculation for value already calculated ?");
        return;
    }
    Shape accumulated_placeable_0
        = start_layer == 1 ? machine_area_ : getAccumulatedPlaceable0(start_layer - 1).offset(FUDGE_LENGTH + (current_min_xy_dist_ + current_min_xy_dist_delta_));
    // ^^^ The calculation here is done on the areas that are increased by xy_distance, but the result is saved without xy_distance,
    // so here it "restores" the previous state to continue calculating from about where it ended.
    // It would be better to ensure placeable areas of radius 0 do not include the xy distance, and removing the code compensating for it here and in calculatePlaceables.
    std::vector<std::pair<LayerIndex, Shape>> data(max_layer + 1, std::pair<LayerIndex, Shape>(-1, Shape()));

    for (LayerIndex layer = start_layer; layer <= max_layer; layer++)
    {
        accumulated_placeable_0 = accumulated_placeable_0.unionPolygons(getPlaceableAreas(0, layer).offset(FUDGE_LENGTH)).difference(anti_overhang_[layer]);
        std::lock_guard<std::mutex> critical_section(*critical_accumulated_placeables_cache_radius_0_);
        accumulated_placeable_0 = simplifier_.polygon(accumulated_placeable_0);
        data[layer] = std::pair(layer, accumulated_placeable_0);
    }
    cura::parallel_for<size_t>(
        std::max(LayerIndex{ start_layer - 1 }, LayerIndex{ 1 }),
        data.size(),
        [&](const coord_t layer_idx)
        {
            data[layer_idx].second = data[layer_idx].second.offset(-(current_min_xy_dist_ + current_min_xy_dist_delta_));
        });
    {
        std::lock_guard<std::mutex> critical_section(*critical_accumulated_placeables_cache_radius_0_);
        for (const std::pair<LayerIndex, Shape>& ins_elem : data)
        {
            if (accumulated_placeables_cache_radius_0_.contains(ins_elem.first))
            {
                spdlog::warn("Recalculated accumulated_placeables_cache_radius_0_ on layer {}", ins_elem.first);
            }
            else
            {
                accumulated_placeables_cache_radius_0_[ins_elem.first] = ins_elem.second;
            }
        }
    }
}


void TreeModelVolumes::calculateCollisionAvoidance(const std::deque<RadiusLayerPair>& keys)
{
    cura::parallel_for<size_t>(
        0,
        keys.size(),
        [&, keys](const size_t key_idx)
        {
            const coord_t radius = keys[key_idx].first;
            const LayerIndex max_required_layer = keys[key_idx].second;
            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist_ * 1.9);
            LayerIndex start_layer = 0;
            {
                std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_collision_);
                start_layer = 1 + std::max(getMaxCalculatedLayer(radius, avoidance_cache_collision_), max_layer_idx_without_blocker_);
            }

            if (start_layer > max_required_layer)
            {
                return;
            }

            std::vector<std::pair<RadiusLayerPair, Shape>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));
            RadiusLayerPair key(radius, 0);

            Shape latest_avoidance = getAvoidance(radius, start_layer - 1, AvoidanceType::COLLISION, true, true);
            for (const LayerIndex layer : ranges::views::iota(static_cast<size_t>(start_layer), max_required_layer + 1UL))
            {
                key.second = layer;
                Shape col = getCollision(radius, layer, true);
                latest_avoidance = safeOffset(latest_avoidance, -max_move_, ClipperLib::jtRound, -max_step_move, col);

                Shape placeable0RadiusCompensated = getAccumulatedPlaceable0(layer).offset(-std::max(radius, increase_until_radius_), ClipperLib::jtRound);
                latest_avoidance = latest_avoidance.difference(placeable0RadiusCompensated).unionPolygons(getCollision(radius, layer, true));

                Shape next_latest_avoidance = simplifier_.polygon(latest_avoidance);
                latest_avoidance = next_latest_avoidance.unionPolygons(latest_avoidance);
                // ^^^ Ensure the simplification only causes the avoidance to become larger.
                // If the deviation of the simplification causes the avoidance to become smaller than it should be it can cause issues, if it is larger the worst case is that the
                // xy distance is effectively increased by deviation. If there would be an option to ensure the resulting polygon only gets larger by simplifying, it should improve
                // performance further.
                data[layer] = std::pair<RadiusLayerPair, Shape>(key, latest_avoidance);
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_collision_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (avoidance_cache_collision_.contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated collision avoidance for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                    }
                    else
                    {
                        avoidance_cache_collision_[ins_elem.first] = ins_elem.second;
                    }
                }
            }
        });
}


// Ensures offsets are only done in sizes with a max step size per offset while adding the collision offset after each step, this ensures that areas cannot glitch through walls
// defined by the collision when offsetting to fast.
Shape TreeModelVolumes::safeOffset(const Shape& me, coord_t distance, ClipperLib::JoinType jt, coord_t max_safe_step_distance, const Shape& collision) const
{
    const size_t steps = std::abs(distance / std::max(min_offset_per_step_, std::abs(max_safe_step_distance)));
    assert(distance * max_safe_step_distance >= 0);
    Shape ret = me;

    for (size_t i = 0; i < steps; ++i)
    {
        ret = ret.offset(max_safe_step_distance, jt).unionPolygons(collision);
    }
    ret = ret.offset(distance % max_safe_step_distance, jt);

    return ret.unionPolygons(collision);
}

void TreeModelVolumes::calculateAvoidance(const std::deque<RadiusLayerPair>& keys) // todo limit to certain avoidance types for better normal support performance
{
    // For every RadiusLayer pair there are 3 avoidances that have to be calculate, calculated in the same paralell_for loop for better parallelization.
    const std::vector<AvoidanceType> all_types = { AvoidanceType::SLOW, AvoidanceType::FAST_SAFE, AvoidanceType::FAST };
    // TODO: This should be a parallel for nowait (non-blocking), but as the parallel-for situation (as in, proper compiler support) continues to change, we're using the 'normal'
    // one right now.
    cura::parallel_for<size_t>(
        0,
        keys.size() * 3,
        [&, keys, all_types](const size_t iter_idx)
        {
            const size_t key_idx = iter_idx / 3;

            const size_t type_idx = iter_idx % all_types.size();
            const AvoidanceType type = all_types[type_idx];
            const bool slow = type == AvoidanceType::SLOW;
            const bool holefree = type == AvoidanceType::FAST_SAFE;

            const coord_t radius = keys[key_idx].first;
            const LayerIndex max_required_layer = keys[key_idx].second;

            // do not calculate not needed safe avoidances
            if (holefree && radius >= increase_until_radius_ + current_min_xy_dist_delta_)
            {
                return;
            }

            const coord_t offset_speed = slow ? max_move_slow_ : max_move_;
            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist_ * 1.9);
            RadiusLayerPair key(radius, 0);
            Shape latest_avoidance;
            LayerIndex start_layer;
            {
                std::lock_guard<std::mutex> critical_section(*(slow ? critical_avoidance_cache_slow_ : holefree ? critical_avoidance_cache_holefree_ : critical_avoidance_cache_));
                start_layer = 1 + getMaxCalculatedLayer(radius, slow ? avoidance_cache_slow_ : holefree ? avoidance_cache_hole_ : avoidance_cache_);
            }
            if (start_layer > max_required_layer)
            {
                spdlog::debug("Requested calculation for value already calculated ?");
                return;
            }
            start_layer = std::max(start_layer, LayerIndex(1)); // Ensure StartLayer is at least 1 as if no avoidance was calculated getMaxCalculatedLayer returns -1
            std::vector<std::pair<RadiusLayerPair, Shape>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));

            latest_avoidance
                = getAvoidance(radius, start_layer - 1, type, false, true); // minDist as the delta was already added, also avoidance for layer 0 will return the collision.

            // ### main loop doing the calculation
            for (const LayerIndex layer : ranges::views::iota(static_cast<size_t>(start_layer), max_required_layer + 1UL))
            {
                key.second = layer;
                Shape col;
                if ((slow && radius < increase_until_radius_ + current_min_xy_dist_delta_) || holefree)
                {
                    col = getCollisionHolefree(radius, layer, true);
                }
                else
                {
                    col = getCollision(radius, layer, true);
                }

                latest_avoidance = safeOffset(latest_avoidance, -offset_speed, ClipperLib::jtRound, -max_step_move, col);
                Shape next_latest_avoidance = simplifier_.polygon(latest_avoidance);
                latest_avoidance = next_latest_avoidance.unionPolygons(latest_avoidance);
                // ^^^ Ensure the simplification only causes the avoidance to become larger.
                // If the deviation of the simplification causes the avoidance to become smaller than it should be it can cause issues, if it is larger the worst case is that the
                // xy distance is effectively increased by deviation. If there would be an option to ensure the resulting polygon only gets larger by simplifying, it should improve
                // performance further.
                data[layer] = std::pair<RadiusLayerPair, Shape>(key, latest_avoidance);
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_progress_);

                if (precalculated_ && precalculation_progress_ < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO)
                {
                    precalculation_progress_ += support_rests_on_model_ ? 0.4 : 1 * TREE_PROGRESS_PRECALC_AVO / (keys.size() * 3);
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress_ * progress_multiplier_ + progress_offset_, TREE_PROGRESS_TOTAL);
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*(slow ? critical_avoidance_cache_slow_ : holefree ? critical_avoidance_cache_holefree_ : critical_avoidance_cache_));
                auto* cache = &(slow ? avoidance_cache_slow_ : holefree ? avoidance_cache_hole_ : avoidance_cache_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (ins_elem.first.second == -1)
                    {
                        // Layer was not calculated
                        continue;
                    }
                    if (cache->contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated avoidance for {} on layer {} Slow:{} holefree:{}", ins_elem.first.first, ins_elem.first.second, slow, holefree);
                    }
                    else
                    {
                        (*cache)[ins_elem.first] = ins_elem.second;
                    }
                }
            }
        });
}

void TreeModelVolumes::calculatePlaceables(const std::deque<RadiusLayerPair>& keys)
{
    // TODO: This should be a parallel for nowait (non-blocking), but as the parallel-for situation (as in, proper compiler support) continues to change, we're using the 'normal'
    // one right now.

    cura::parallel_for<size_t>(
        0,
        keys.size(),
        [&, keys](const size_t key_idx)
        {
            const coord_t radius = keys[key_idx].first;
            const LayerIndex max_required_layer = keys[key_idx].second;
            std::vector<std::pair<RadiusLayerPair, Shape>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));
            RadiusLayerPair key(radius, 0);

            LayerIndex start_layer;
            {
                std::lock_guard<std::mutex> critical_section(*critical_placeable_areas_cache_);
                start_layer = 1 + getMaxCalculatedLayer(radius, placeable_areas_cache_);
            }
            if (start_layer > max_required_layer)
            {
                spdlog::debug("Requested calculation for value already calculated ?");
                return;
            }

            if (start_layer == 0)
            {
                data[0] = std::pair<RadiusLayerPair, Shape>(key, machine_border_.difference(getCollision(radius, 0, true)));
                start_layer = 1;
            }

            for (const LayerIndex layer : ranges::views::iota(static_cast<size_t>(start_layer), max_required_layer + 1))
            {
                key.second = layer;
                Shape placeable = getPlaceableAreas(0, layer);
                placeable = simplifier_.polygon(placeable); // it is faster to do this here in each thread than once in calculateCollision.
                placeable = placeable.offset(-(radius + (current_min_xy_dist_ + current_min_xy_dist_delta_))).unionPolygons();
                data[layer] = std::pair<RadiusLayerPair, Shape>(key, placeable);
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_progress_);

                if (precalculated_ && precalculation_progress_ < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO)
                {
                    precalculation_progress_ += 0.2 * TREE_PROGRESS_PRECALC_AVO / (keys.size());
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress_ * progress_multiplier_ + progress_offset_, TREE_PROGRESS_TOTAL);
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_placeable_areas_cache_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (ins_elem.first.second == -1)
                    {
                        // Layer was not calculated
                        continue;
                    }
                    if (placeable_areas_cache_.contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated placeables for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                    }
                    else
                    {
                        placeable_areas_cache_[ins_elem.first] = ins_elem.second;
                    }
                }
            }
        });
}


void TreeModelVolumes::calculateAvoidanceToModel(const std::deque<RadiusLayerPair>& keys)
{
    // For every RadiusLayer pair there are 3 avoidances that have to be calculated, calculated in the same parallel_for loop for better parallelization.
    const std::vector<AvoidanceType> all_types = { AvoidanceType::SLOW, AvoidanceType::FAST_SAFE, AvoidanceType::FAST };
    // TODO: This should be a parallel for nowait (non-blocking), but as the parallel-for situation (as in, proper compiler support) continues to change, we're using the 'normal'
    // one right now.
    cura::parallel_for<size_t>(
        0,
        keys.size() * 3,
        [&, keys, all_types](const size_t iter_idx)
        {
            const size_t key_idx = iter_idx / 3;
            const size_t type_idx = iter_idx % all_types.size();
            const AvoidanceType type = all_types[type_idx];
            const bool slow = type == AvoidanceType::SLOW;
            const bool holefree = type == AvoidanceType::FAST_SAFE;
            const coord_t radius = keys[key_idx].first;
            const LayerIndex max_required_layer = keys[key_idx].second;

            // do not calculate not needed safe avoidances
            if (holefree && radius >= increase_until_radius_ + current_min_xy_dist_delta_)
            {
                return;
            }
            const coord_t offset_speed = slow ? max_move_slow_ : max_move_;
            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist_ * 1.9);
            Shape latest_avoidance;
            std::vector<std::pair<RadiusLayerPair, Shape>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Shape>(RadiusLayerPair(radius, -1), Shape()));
            RadiusLayerPair key(radius, 0);

            LayerIndex start_layer;

            {
                std::lock_guard<std::mutex> critical_section(
                    *(slow       ? critical_avoidance_cache_to_model_slow_
                      : holefree ? critical_avoidance_cache_holefree_to_model_
                                 : critical_avoidance_cache_to_model_));
                start_layer = 1 + getMaxCalculatedLayer(radius, slow ? avoidance_cache_to_model_slow_ : holefree ? avoidance_cache_hole_to_model_ : avoidance_cache_to_model_);
            }
            start_layer = std::max(start_layer, LayerIndex(1));
            if (start_layer > max_required_layer)
            {
                spdlog::debug("Requested calculation for value already calculated or max_required_layer is 0?");
                return;
            }
            getPlaceableAreas(radius, max_required_layer); // ensuring Placeable Areas are calculated
            latest_avoidance
                = getAvoidance(radius, start_layer - 1, type, true, true); // minDist as the delta was already added, also avoidance for layer 0 will return the collision.

            // ### main loop doing the calculation
            for (const LayerIndex layer : ranges::views::iota(static_cast<size_t>(start_layer), max_required_layer + 1))
            {
                key.second = layer;
                Shape col = getCollision(radius, layer, true);

                if ((slow && radius < increase_until_radius_ + current_min_xy_dist_delta_) || holefree)
                {
                    col = getCollisionHolefree(radius, layer, true);
                }
                else
                {
                    col = getCollision(radius, layer, true);
                }

                latest_avoidance = safeOffset(latest_avoidance, -offset_speed, ClipperLib::jtRound, -max_step_move, col).difference(getPlaceableAreas(radius, layer));
                Shape next_latest_avoidance = simplifier_.polygon(latest_avoidance);
                latest_avoidance = next_latest_avoidance.unionPolygons(latest_avoidance);
                // ^^^ Ensure the simplification only causes the avoidance to become larger.
                // If the deviation of the simplification causes the avoidance to become smaller than it should be it can cause issues, if it is larger the worst case is that the
                // xy distance is effectively increased by deviation. If there would be an option to ensure the resulting polygon only gets larger by simplifying, it should improve
                // performance further.
                data[layer] = std::pair<RadiusLayerPair, Shape>(key, latest_avoidance);
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_progress_);

                if (precalculated_ && precalculation_progress_ < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO)
                {
                    precalculation_progress_ += 0.4 * TREE_PROGRESS_PRECALC_AVO / (keys.size() * 3);
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress_ * progress_multiplier_ + progress_offset_, TREE_PROGRESS_TOTAL);
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(
                    *(slow       ? critical_avoidance_cache_to_model_slow_
                      : holefree ? critical_avoidance_cache_holefree_to_model_
                                 : critical_avoidance_cache_to_model_));
                auto* cache = &(slow ? avoidance_cache_to_model_slow_ : holefree ? avoidance_cache_hole_to_model_ : avoidance_cache_to_model_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (ins_elem.first.second == -1)
                    {
                        // Layer was not calculated
                        continue;
                    }
                    if (cache->contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated avoidance to model for {} on layer {} Slow:{} holefree:{}", ins_elem.first.first, ins_elem.first.second, slow, holefree);
                    }
                    else
                    {
                        (*cache)[ins_elem.first] = ins_elem.second;
                    }
                }
            }
        });
}

void TreeModelVolumes::calculateFake0Avoidances(const LayerIndex max_layer)
{
    LayerIndex start_layer;
    {
        std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_);
        start_layer = 1 + getMaxCalculatedLayer(0, avoidance_cache_);
    }
    const coord_t radius_offset = -radius_0_;
    cura::parallel_for<size_t>(
        start_layer,
        max_layer + 1,
        [&](const LayerIndex layer_idx)
        {
            RadiusLayerPair key = RadiusLayerPair(0, layer_idx);
            if (! precalculated_ || support_rest_preference_ == RestPreference::BUILDPLATE)
            {
                Shape smaller_avoidance_slow = getAvoidance(1, layer_idx, AvoidanceType::SLOW, false, true).offset(radius_offset, ClipperLib::jtRound);
                Shape smaller_avoidance_fast = getAvoidance(1, layer_idx, AvoidanceType::FAST, false, true).offset(radius_offset, ClipperLib::jtRound);
                Shape smaller_avoidance_fast_safe = getAvoidance(1, layer_idx, AvoidanceType::FAST_SAFE, false, true).offset(radius_offset, ClipperLib::jtRound);
                {
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_slow_);
                    if (avoidance_cache_slow_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 avoidance slow for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_slow_[key] = smaller_avoidance_slow;
                    }
                }
                {
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_);
                    if (avoidance_cache_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 avoidance for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_[key] = smaller_avoidance_fast;
                    }
                }
                {
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_holefree_);
                    if (avoidance_cache_hole_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 avoidance holefree for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_hole_[key] = smaller_avoidance_fast_safe;
                    }
                }
            }

            if (! precalculated_ || support_rests_on_model_)
            {
                Shape smaller_avoidance_to_model_slow = getAvoidance(1, layer_idx, AvoidanceType::SLOW, true, true).offset(radius_offset, ClipperLib::jtRound);
                Shape smaller_avoidance_to_model_fast = getAvoidance(1, layer_idx, AvoidanceType::FAST, true, true).offset(radius_offset, ClipperLib::jtRound);
                Shape smaller_avoidance_to_model_fast_safe = getAvoidance(1, layer_idx, AvoidanceType::FAST_SAFE, true, true).offset(radius_offset, ClipperLib::jtRound);

                {
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_to_model_slow_);
                    if (avoidance_cache_to_model_slow_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 avoidance to model slow for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_to_model_slow_[key] = smaller_avoidance_to_model_slow;
                    }
                }
                {
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_to_model_);
                    if (avoidance_cache_to_model_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 avoidance to model for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_to_model_[key] = smaller_avoidance_to_model_fast;
                    }
                }
                {
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_holefree_to_model_);
                    if (avoidance_cache_hole_to_model_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 avoidance to model holefree for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_hole_to_model_[key] = smaller_avoidance_to_model_fast_safe;
                    }
                }
                if (layer_idx > max_layer_idx_without_blocker_)
                {
                    Shape smaller_avoidance_collision = getAvoidance(1, layer_idx, AvoidanceType::COLLISION, true, true).offset(radius_offset, ClipperLib::jtRound);
                    std::lock_guard<std::mutex> critical_section(*critical_avoidance_cache_collision_);
                    if (avoidance_cache_collision_.contains(key))
                    {
                        spdlog::warn("Recalculated fake 0 collision avoidance for {} on layer {}", key.first, key.second);
                    }
                    else
                    {
                        avoidance_cache_collision_[key] = smaller_avoidance_collision;
                    }
                }
            }
        });
}


void TreeModelVolumes::calculateWallRestrictions(const std::deque<RadiusLayerPair>& keys)
{
    // Wall restrictions are mainly important when they represent actual walls that are printed, and not "just" the configured z_distance, because technically valid placement is no
    // excuse for moving through a wall. As they exist to prevent accidentially moving though a wall at high speed between layers like thie (x = wall,i = influence area,o= empty
    // space,d = blocked area because of z distance) Assume maximum movement distance is two characters and maximum safe movement distance of one character

    /* Potential issue addressed by the wall restrictions: Influence area may lag through a wall
     *  layer z+1:iiiiiiiiiiioooo
     *  layer z+0:xxxxxiiiiiiiooo
     *  layer z-1:ooooixxxxxxxxxx
     */

    // The radius for the upper collission has to be 0 as otherwise one may not enter areas that may be forbidden on layer_idx but not one below (c = not an influence area even
    // though it should ):
    /*
     *  layer z+1:xxxxxiiiiiioo
     *  layer z+0:dddddiiiiiiio
     *  layer z-1:dddocdddddddd
     */
    // Also there can not just the collision of the lower layer be used because if it were:
    /*
     *  layer z+1:dddddiiiiiiiiiio
     *  layer z+0:xxxxxddddddddddc
     *  layer z-1:dddddxxxxxxxxxxc
     */
    // Or of the upper layer be used because if it were:
    /*
     *  layer z+1:dddddiiiiiiiiiio
     *  layer z+0:xxxxcddddddddddc
     *  layer z-1:ddddcxxxxxxxxxxc
     */

    // And just offseting with maximum movement distance (and not in multiple steps) could cause:
    /*
     *  layer z:   oxiiiiiiiiioo
     *  layer z-1: ixiiiiiiiiiii
     */

    // FIXME: This should be a parallel-for nowait (non-blocking), but as the parallel-for situation (as in, proper compiler support) continues to change, we're using the 'normal'
    // one right now.
    cura::parallel_for<size_t>(
        0,
        keys.size(),
        [&, keys](const size_t key_idx)
        {
            const coord_t radius = keys[key_idx].first;
            RadiusLayerPair key(radius, 0);
            coord_t min_layer_bottom;
            std::unordered_map<RadiusLayerPair, Shape> data;
            std::unordered_map<RadiusLayerPair, Shape> data_min;

            {
                std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_);
                min_layer_bottom = getMaxCalculatedLayer(radius, wall_restrictions_cache_);
            }

            if (min_layer_bottom < 1)
            {
                min_layer_bottom = 1;
            }
            for (const auto layer_idx : ranges::views::iota(min_layer_bottom, keys[key_idx].second + 1UL))
            {
                key.second = layer_idx;
                const LayerIndex layer_idx_below = layer_idx - 1;
                Shape wall_restriction = simplifier_.polygon(
                    getCollision(0, layer_idx, false).intersection(getCollision(radius, layer_idx_below, true))); // radius contains current_min_xy_dist_delta already if required
                data.emplace(key, wall_restriction);
                if (current_min_xy_dist_delta_ > 0)
                {
                    Shape wall_restriction_min = simplifier_.polygon(getCollision(0, layer_idx, true).intersection(getCollision(radius, layer_idx_below, true)));
                    data_min.emplace(key, wall_restriction_min);
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (wall_restrictions_cache_.contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated wall restriction for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                    }
                    else
                    {
                        wall_restrictions_cache_[ins_elem.first] = ins_elem.second;
                    }
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_min_);
                for (const std::pair<RadiusLayerPair, Shape>& ins_elem : data)
                {
                    if (wall_restrictions_cache_min_.contains(ins_elem.first))
                    {
                        spdlog::warn("Recalculated wall restriction min for {} on layer {}", ins_elem.first.first, ins_elem.first.second);
                    }
                    else
                    {
                        wall_restrictions_cache_min_[ins_elem.first] = ins_elem.second;
                    }
                }
            }
        });
}

coord_t TreeModelVolumes::ceilRadius(coord_t radius) const
{
    if (radius == 0)
    {
        return 0;
    }
    if (radius <= radius_0_)
    {
        return radius_0_;
    }

    coord_t exponential_result = SUPPORT_TREE_EXPONENTIAL_THRESHOLD * SUPPORT_TREE_EXPONENTIAL_FACTOR;
    const coord_t stepsize = (exponential_result - radius_0_) / (SUPPORT_TREE_PRE_EXPONENTIAL_STEPS + 1);
    coord_t result = radius_0_;
    for (size_t i = 0; i < SUPPORT_TREE_PRE_EXPONENTIAL_STEPS; ++i)
    {
        result += stepsize;
        if (result >= radius && ! ignorable_radii_.count(result))
        {
            return result;
        }
    }

    while (exponential_result < radius || ignorable_radii_.count(exponential_result))
    {
        exponential_result = std::max(coord_t(exponential_result * SUPPORT_TREE_EXPONENTIAL_FACTOR), exponential_result + SUPPORT_TREE_COLLISION_RESOLUTION);
    }
    return exponential_result;
}

template<typename KEY>
const std::optional<std::reference_wrapper<const Shape>> TreeModelVolumes::getArea(const std::unordered_map<KEY, Shape>& cache, const KEY key) const
{
    const auto it = cache.find(key);
    if (it != cache.end())
    {
        return std::optional<std::reference_wrapper<const Shape>>{ it->second };
    }
    else
    {
        return std::optional<std::reference_wrapper<const Shape>>();
    }
}

Shape TreeModelVolumes::calculateMachineBorderCollision(const Shape&& machine_border)
{
    Shape machine_volume_border = machine_border.offset(MM2INT(1000.0)); // Put a border of 1 meter around the print volume so that we don't collide.
    machine_volume_border = machine_volume_border.difference(machine_border); // Subtract the actual volume from the collision area.
    return machine_volume_border;
}

} // namespace cura
