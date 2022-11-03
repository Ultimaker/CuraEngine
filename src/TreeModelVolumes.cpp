// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TreeModelVolumes.h"
#include "TreeSupport.h"
#include "TreeSupportEnums.h"
#include "progress/Progress.h"
#include "sliceDataStorage.h"
#include "utils/ThreadPool.h"
#include "utils/algorithm.h"
#include <spdlog/spdlog.h>
namespace cura
{

TreeModelVolumes::TreeModelVolumes(const SliceDataStorage& storage, const coord_t max_move, const coord_t max_move_slow, size_t current_mesh_idx, double progress_multiplier, double progress_offset, const std::vector<Polygons>& additional_excluded_areas) : max_move_{ std::max(max_move - 2, coord_t(0)) }, max_move_slow_{ std::max(max_move_slow - 2, coord_t(0)) }, progress_multiplier{ progress_multiplier }, progress_offset{ progress_offset }, machine_border_{ calculateMachineBorderCollision(storage.getMachineBorder()) } // -2 to avoid rounding errors
{
    anti_overhang_ = std::vector<Polygons>(storage.support.supportLayers.size(), Polygons());
    std::unordered_map<size_t, size_t> mesh_to_layeroutline_idx;


    coord_t min_maximum_resolution_ = std::numeric_limits<coord_t>::max();
    coord_t min_maximum_deviation_ = std::numeric_limits<coord_t>::max();
    coord_t min_maximum_area_deviation_ = std::numeric_limits<coord_t>::max();

    support_rests_on_model = false;
    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];
        bool added = false;
        for (size_t idx = 0; idx < layer_outlines_.size(); idx++)
        {
            if (checkSettingsEquality(layer_outlines_[idx].first, mesh.settings))
            {
                added = true;
                mesh_to_layeroutline_idx[mesh_idx] = idx;
            }
        }
        if (!added)
        {
            mesh_to_layeroutline_idx[mesh_idx] = layer_outlines_.size();
            layer_outlines_.emplace_back(mesh.settings, std::vector<Polygons>(storage.support.supportLayers.size(), Polygons()));
        }
    }

    for (auto data_pair : layer_outlines_)
    {
        support_rests_on_model |= data_pair.first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
        min_maximum_deviation_ = std::min(min_maximum_deviation_, data_pair.first.get<coord_t>("meshfix_maximum_deviation"));
        min_maximum_resolution_ = std::min(min_maximum_resolution_, data_pair.first.get<coord_t>("meshfix_maximum_resolution"));
        min_maximum_area_deviation_ = std::min(min_maximum_area_deviation_, data_pair.first.get<coord_t>("meshfix_maximum_extrusion_area_deviation"));
    }

    min_maximum_deviation_ = std::min(coord_t(SUPPORT_TREE_MAX_DEVIATION), min_maximum_deviation_);
    current_outline_idx = mesh_to_layeroutline_idx[current_mesh_idx];
    TreeSupportSettings config(layer_outlines_[current_outline_idx].first);

    if (config.support_overrides == SupportDistPriority::Z_OVERRIDES_XY)
    {
        current_min_xy_dist = config.xy_min_distance;

        if (TreeSupportSettings::has_to_rely_on_min_xy_dist_only)
        {
            current_min_xy_dist = std::max(current_min_xy_dist, coord_t(100));
        }

        current_min_xy_dist_delta = std::max(config.xy_distance - current_min_xy_dist, coord_t(0));
    }
    else
    {
        current_min_xy_dist = config.xy_distance;
        current_min_xy_dist_delta = 0;
    }
    increase_until_radius = config.increase_radius_until_radius;

    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];

        cura::parallel_for<coord_t>(0, LayerIndex(layer_outlines_[mesh_to_layeroutline_idx[mesh_idx]].second.size()), // todo LayerIndex
            [&](const LayerIndex layer_idx)
            {
            if (mesh.layer_nr_max_filled_layer < layer_idx)
            {
                return; // cant break as parallel_for wont allow it, this is equivalent to a continue
            }
            Polygons outline = extractOutlineFromMesh(mesh, layer_idx);
            layer_outlines_[mesh_to_layeroutline_idx[mesh_idx]].second[layer_idx].add(outline);
        });
    }
    cura::parallel_for<coord_t>(0, LayerIndex(anti_overhang_.size()), // todo LayerIndex
        [&](const LayerIndex layer_idx)
        {
        if (layer_idx < coord_t(additional_excluded_areas.size()))
        {
            anti_overhang_[layer_idx].add(additional_excluded_areas[layer_idx]);
        }

        if (SUPPORT_TREE_AVOID_SUPPORT_BLOCKER)
        {
            anti_overhang_[layer_idx].add(storage.support.supportLayers[layer_idx].anti_overhang);
        }

        if (storage.primeTower.enabled)
        {
            anti_overhang_[layer_idx].add(layer_idx == 0 ? storage.primeTower.outer_poly_first_layer : storage.primeTower.outer_poly);
        }
        anti_overhang_[layer_idx] = anti_overhang_[layer_idx].unionPolygons();
    });

    for (size_t idx = 0; idx < layer_outlines_.size(); idx++)
    {
        cura::parallel_for<coord_t>(0, anti_overhang_.size(), [&](const LayerIndex layer_idx) { layer_outlines_[idx].second[layer_idx] = layer_outlines_[idx].second[layer_idx].unionPolygons(); }); // todo LayerIndex
    }
    radius_0 = config.getRadius(0);
    support_rest_preference = config.support_rest_preference;
    simplifier = Simplify(min_maximum_resolution_, min_maximum_deviation_, min_maximum_area_deviation_);
}


void TreeModelVolumes::precalculate(coord_t max_layer)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    precalculated = true;

    // Get the config corresponding to one mesh that is in the current group. Which one has to be irrelevant. Not the prettiest way to do this, but it ensures some calculations that may be a bit more complex like inital layer diameter are only done in once.
    TreeSupportSettings config(layer_outlines_[current_outline_idx].first);

    // calculate which radius each layer in the tip may have.
    std::unordered_set<coord_t> possible_tip_radiis;
    for (size_t dtt = 0; dtt <= config.tip_layers; dtt++)
    {
        possible_tip_radiis.emplace(ceilRadius(config.getRadius(dtt)));
        possible_tip_radiis.emplace(ceilRadius(config.getRadius(dtt) + current_min_xy_dist_delta));
    }
    // It theoretically may happen in the tip, that the radius can change so much in-between 2 layers, that a ceil step is skipped (as in there is a radius r so that ceilRadius(radius(dtt))<ceilRadius(r)<ceilRadius(radius(dtt+1))). As such a radius will not reasonable happen in the tree and it will most likely not be requested, there is no need to calculate them. So just skip these.
    for (coord_t radius_eval = ceilRadius(1); radius_eval <= config.branch_radius; radius_eval = ceilRadius(radius_eval + 1))
    {
        if (!possible_tip_radiis.count(radius_eval))
        {
            ignorable_radii_.emplace(radius_eval);
        }
    }

    // it may seem that the required avoidance can be of a smaller radius when going to model (no initial layer diameter for to model branches)
    // but as for every branch going towards the bp, the to model avoidance is required to check for possible merges with to model branches, this assumption is in-fact wrong.
    std::unordered_map<coord_t, LayerIndex> radius_until_layer;
    // while it is possible to calculate, up to which layer the avoidance should be calculated, this simulation is easier to understand, and does not need to be adjusted if something of the radius calculation is changed.
    // Overhead with an assumed worst case of 6600 layers was about 2ms
    for (LayerIndex simulated_dtt = 0; simulated_dtt <= max_layer; simulated_dtt++)
    {
        const LayerIndex current_layer = max_layer - simulated_dtt;
        const coord_t max_regular_radius = ceilRadius(config.getRadius(simulated_dtt, 0) + current_min_xy_dist_delta);
        const coord_t max_min_radius = ceilRadius(config.getRadius(simulated_dtt, 0)); // the maximal radius that the radius with the min_xy_dist can achieve
        const coord_t max_initial_layer_diameter_radius = ceilRadius(config.recommendedMinRadius(current_layer) + current_min_xy_dist_delta);
        if (!radius_until_layer.count(max_regular_radius))
        {
            radius_until_layer[max_regular_radius] = current_layer;
        }
        if (!radius_until_layer.count(max_min_radius))
        {
            radius_until_layer[max_min_radius] = current_layer;
        }
        if (!radius_until_layer.count(max_initial_layer_diameter_radius))
        {
            radius_until_layer[max_initial_layer_diameter_radius] = current_layer;
        }
    }

    // Copy to deque to use in parallel for later.
    std::deque<RadiusLayerPair> relevant_avoidance_radiis;
    std::deque<RadiusLayerPair> relevant_avoidance_radiis_to_model;
    relevant_avoidance_radiis.insert(relevant_avoidance_radiis.end(), radius_until_layer.begin(), radius_until_layer.end());
    relevant_avoidance_radiis_to_model.insert(relevant_avoidance_radiis_to_model.end(), radius_until_layer.begin(), radius_until_layer.end());

    // Append additional radiis needed for collision.

    radius_until_layer[ceilRadius(increase_until_radius, false)] = max_layer; // To calculate collision holefree for every radius, the collision of radius increase_until_radius will be required.
    // Collision for radius 0 needs to be calculated everywhere, as it will be used to ensure valid xy_distance in drawAreas.
    radius_until_layer[0] = max_layer;
    if (current_min_xy_dist_delta != 0)
    {
        radius_until_layer[current_min_xy_dist_delta] = max_layer;
    }

    std::deque<RadiusLayerPair> relevant_collision_radiis;
    relevant_collision_radiis.insert(relevant_collision_radiis.end(), radius_until_layer.begin(), radius_until_layer.end()); // Now that required_avoidance_limit contains the maximum of ild and regular required radius just copy.

    // ### Calculate the relevant collisions
    calculateCollision(relevant_collision_radiis);

    // calculate a separate Collisions with all holes removed. These are relevant for some avoidances that try to avoid holes (called safe)
    std::deque<RadiusLayerPair> relevant_hole_collision_radiis;
    for (RadiusLayerPair key : relevant_avoidance_radiis)
    {
        spdlog::debug("Calculating avoidance of radius {} up to layer {}",key.first,key.second);
        if (key.first < increase_until_radius + current_min_xy_dist_delta)
        {
            relevant_hole_collision_radiis.emplace_back(key);
        }
    }

    // ### Calculate collisions without holes, build from regular collision
    calculateCollisionHolefree(relevant_hole_collision_radiis);

    auto t_coll = std::chrono::high_resolution_clock::now();

    // ### Calculate the relevant avoidances in parallel as far as possible
    {
        std::future<void> placeable_waiter;
        std::future<void> avoidance_waiter;

        if (support_rests_on_model)
        {
            calculatePlaceables(relevant_avoidance_radiis_to_model);
        }
        if (support_rest_preference == RestPreference::BUILDPLATE)
        {
            calculateAvoidance(relevant_avoidance_radiis);
        }

        calculateWallRestrictions(relevant_avoidance_radiis);
        if (support_rests_on_model)
        {
            // If nowait ensure here the following is calculated: calculatePlaceables todo
            calculateAvoidanceToModel(relevant_avoidance_radiis_to_model);
            // If nowait ensure here the following is calculated: calculateAvoidanceToModel todo
        }
        if (support_rest_preference == RestPreference::BUILDPLATE)
        {
            // If nowait ensure here the following is calculated: calculateAvoidance todo
        }
        // If nowait ensure here the following is calculated: calculateWallRestrictions todo
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto dur_col = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_coll - t_start).count();
    auto dur_avo = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_coll).count();

    spdlog::info("Precalculating collision took {} ms. Precalculating avoidance took {} ms.", dur_col, dur_avo);
}

const Polygons& TreeModelVolumes::getCollision(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    coord_t orig_radius = radius;
    std::optional<std::reference_wrapper<const Polygons>> result;
    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }

    // special case as if a radius 0 is requested it could be to ensure correct xy distance. As such it is beneficial if the collision is as close to the configured values as possible.
    if (orig_radius != 0)
    {
        radius = ceilRadius(radius);
    }
    RadiusLayerPair key{ radius, layer_idx };

    {
        std::lock_guard<std::mutex> critical_section_support_max_layer_nr(*critical_avoidance_cache_);
        result = getArea(collision_cache_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated)
    {
        spdlog::warn("Had to calculate collision at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
    }
    calculateCollision(key);
    return getCollision(orig_radius, layer_idx, min_xy_dist);
}

const Polygons& TreeModelVolumes::getCollisionHolefree(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    coord_t orig_radius = radius;
    std::optional<std::reference_wrapper<const Polygons>> result;
    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }
    if (radius >= increase_until_radius + current_min_xy_dist_delta)
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
    if (precalculated)
    {
        spdlog::warn("Had to calculate collision holefree at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
    }
    calculateCollisionHolefree(key);
    return getCollisionHolefree(orig_radius, layer_idx, min_xy_dist);
}


const Polygons& TreeModelVolumes::getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist)
{
    if (layer_idx == 0) // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
    {
        return getCollision(radius, layer_idx, min_xy_dist);
    }

    coord_t orig_radius = radius;

    std::optional<std::reference_wrapper<const Polygons>> result;

    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }
    radius = ceilRadius(radius);

    if (radius >= increase_until_radius + current_min_xy_dist_delta && type == AvoidanceType::FAST_SAFE) // no holes anymore by definition at this request
    {
        type = AvoidanceType::FAST;
    }

    const RadiusLayerPair key{ radius, layer_idx };

    std::unordered_map<RadiusLayerPair, Polygons>* cache_ptr = nullptr;
    std::mutex* mutex_ptr;
    if (!to_model && type == AvoidanceType::FAST)
    {
        cache_ptr = &avoidance_cache_;
        mutex_ptr = critical_avoidance_cache_.get();
    }
    else if (!to_model && type == AvoidanceType::SLOW)
    {
        cache_ptr = &avoidance_cache_slow_;
        mutex_ptr = critical_avoidance_cache_slow_.get();
    }
    else if (!to_model && type == AvoidanceType::FAST_SAFE)
    {
        cache_ptr = &avoidance_cache_hole_;
        mutex_ptr = critical_avoidance_cache_holefree_.get();
    }
    else if (to_model && type == AvoidanceType::FAST)
    {
        cache_ptr = &avoidance_cache_to_model_;
        mutex_ptr = critical_avoidance_cache_to_model_.get();
    }
    else if (to_model && type == AvoidanceType::SLOW)
    {
        cache_ptr = &avoidance_cache_to_model_slow_;
        mutex_ptr = critical_avoidance_cache_to_model_slow_.get();
    }
    else if (to_model && type == AvoidanceType::FAST_SAFE)
    {
        cache_ptr = &avoidance_cache_hole_to_model_;
        mutex_ptr = critical_avoidance_cache_holefree_to_model_.get();
    }
    else
    {
        spdlog::error("Invalid Avoidance Request");
    }


    if (to_model)
    {
        {
            std::lock_guard<std::mutex> critical_section(*mutex_ptr);
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            spdlog::warn("Had to calculate Avoidance to model at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
        }
        calculateAvoidanceToModel(key);
    }
    else
    {
        {
            std::lock_guard<std::mutex> critical_section(*mutex_ptr);
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            spdlog::warn("Had to calculate Avoidance at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
        }
        calculateAvoidance(key);
    }
    return getAvoidance(orig_radius, layer_idx, type, to_model, min_xy_dist); // retrive failed and correct result was calculated. Now it has to be retrived.
}

const Polygons& TreeModelVolumes::getPlaceableAreas(coord_t radius, LayerIndex layer_idx)
{
    std::optional<std::reference_wrapper<const Polygons>> result;
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
    if (precalculated)
    {
        spdlog::warn("Had to calculate Placeable Areas at radius {} and layer {}, but precalculate was called. Performance may suffer!", radius, layer_idx);
    }
    if (radius != 0)
    {
        calculatePlaceables(key);
    }
    else
    {
        getCollision(0, layer_idx, true);
    }
    return getPlaceableAreas(orig_radius, layer_idx);
}


const Polygons& TreeModelVolumes::getWallRestriction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    if (layer_idx == 0) // Should never be requested as there will be no going below layer 0 ..., but just to be sure some semi-sane catch. Alternative would be empty Polygon.
    {
        return getCollision(radius, layer_idx, min_xy_dist);
    }

    coord_t orig_radius = radius;
    min_xy_dist = min_xy_dist && current_min_xy_dist_delta > 0;

    std::optional<std::reference_wrapper<const Polygons>> result;

    radius = ceilRadius(radius);
    const RadiusLayerPair key{ radius, layer_idx };

    std::unordered_map<RadiusLayerPair, Polygons>* cache_ptr;
    if (min_xy_dist)
    {
        cache_ptr = &wall_restrictions_cache_min_;
    }
    else
    {
        cache_ptr = &wall_restrictions_cache_;
    }


    if (min_xy_dist)
    {
        {
            std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_min_);
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            spdlog::warn("Had to calculate Wall restricions at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
        }
    }
    else
    {
        {
            std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_);
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            spdlog::warn("Had to calculate Wall restricions at radius {} and layer {}, but precalculate was called. Performance may suffer!", key.first, key.second);
        }
    }
    calculateWallRestrictions(key);
    return getWallRestriction(orig_radius, layer_idx, min_xy_dist); // Retrieve failed and correct result was calculated. Now it has to be retrieved.
}

coord_t TreeModelVolumes::ceilRadius(coord_t radius, bool min_xy_dist) const
{
    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }
    return ceilRadius(radius);
}
coord_t TreeModelVolumes::getRadiusNextCeil(coord_t radius, bool min_xy_dist) const
{
    coord_t ceiled_radius = ceilRadius(radius, min_xy_dist);

    if (!min_xy_dist)
        ceiled_radius -= current_min_xy_dist_delta;
    return ceiled_radius;
}

bool TreeModelVolumes::checkSettingsEquality(const Settings& me, const Settings& other) const
{
    return TreeSupportSettings(me) == TreeSupportSettings(other);
}


Polygons TreeModelVolumes::extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx) const
{
    constexpr bool external_polys_only = false;
    Polygons total;

    // similar to SliceDataStorage.getLayerOutlines but only for one mesh instead of for everyone

    if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
    {
        return Polygons();
    }
    const SliceLayer& layer = mesh.layers[layer_idx];

    layer.getOutlines(total, external_polys_only);
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
    {
        total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
    }
    coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    coord_t maximum_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    coord_t maximum_area_deviation = mesh.settings.get<coord_t>("meshfix_maximum_extrusion_area_deviation");
    return Simplify(maximum_resolution, maximum_deviation, maximum_area_deviation).polygon(total);
}

LayerIndex TreeModelVolumes::getMaxCalculatedLayer(coord_t radius, const std::unordered_map<RadiusLayerPair, Polygons>& map) const
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


void TreeModelVolumes::calculateCollision(std::deque<RadiusLayerPair> keys)
{
    cura::parallel_for<size_t>(0, keys.size(),
        [&](const size_t i)
        {
        coord_t radius = keys[i].first;
        RadiusLayerPair key(radius, 0);
        std::unordered_map<RadiusLayerPair, Polygons> data_outer;
        std::unordered_map<RadiusLayerPair, Polygons> data_placeable_outer;
        for (size_t outline_idx = 0; outline_idx < layer_outlines_.size(); outline_idx++)
        {
            std::unordered_map<RadiusLayerPair, Polygons> data;
            std::unordered_map<RadiusLayerPair, Polygons> data_placeable;

            const coord_t layer_height = layer_outlines_[outline_idx].first.get<coord_t>("layer_height");
            const bool support_rests_on_this_model = layer_outlines_[outline_idx].first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
            const coord_t z_distance_bottom = layer_outlines_[outline_idx].first.get<coord_t>("support_bottom_distance");
            const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom, layer_height);
            const coord_t z_distance_top_layers = round_up_divide(layer_outlines_[outline_idx].first.get<coord_t>("support_top_distance"), layer_height);
            const LayerIndex max_anti_overhang_layer = anti_overhang_.size() - 1;
            const LayerIndex max_required_layer = keys[i].second + std::max(coord_t(1), z_distance_top_layers);
            const coord_t xy_distance = outline_idx == current_outline_idx ? current_min_xy_dist : layer_outlines_[outline_idx].first.get<coord_t>("support_xy_distance");
            // technically this causes collision for the normal xy_distance to be larger by current_min_xy_dist_delta for all not currently processing meshes as this delta will be added at request time.
            // avoiding this would require saving each collision for each outline_idx separately.
            // and later for each avoidance... But avoidance calculation has to be for the whole scene and can NOT be done for each outline_idx separately and combined later.
            // so avoiding this inaccuracy seems infeasible as it would require 2x the avoidance calculations => 0.5x the performance.
            coord_t min_layer_bottom;
            {
                std::lock_guard<std::mutex> critical_section(*critical_collision_cache_);
                min_layer_bottom = getMaxCalculatedLayer(radius, collision_cache_) - z_distance_bottom_layers;
            }

            if (min_layer_bottom < 0)
            {
                min_layer_bottom = 0;
            }
            for (LayerIndex layer_idx = min_layer_bottom; layer_idx <= max_required_layer; layer_idx++)
            {
                key.second = layer_idx;
                Polygons collision_areas = machine_border_;
                if (size_t(layer_idx) < layer_outlines_[outline_idx].second.size())
                {
                    collision_areas.add(layer_outlines_[outline_idx].second[layer_idx]);
                }
                collision_areas = collision_areas.offset(radius + xy_distance); // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                data[key].add(collision_areas); // if a key does not exist when it is accessed it is added!
            }


            // Add layers below, to ensure correct support_bottom_distance. Also save placeable areas of radius 0, if required for this mesh.
            for (LayerIndex layer_idx = max_required_layer; layer_idx >= min_layer_bottom; layer_idx--)
            {
                key.second = layer_idx;
                for (size_t layer_offset = 1; layer_offset <= z_distance_bottom_layers && layer_idx - coord_t(layer_offset) > min_layer_bottom; layer_offset++)
                {
                    data[key].add(data[RadiusLayerPair(radius, layer_idx - layer_offset)]);
                }
                if (support_rests_on_this_model && radius == 0 && layer_idx < coord_t(1 + keys[i].second))
                {
                    data[key] = data[key].unionPolygons();
                    Polygons above = data[RadiusLayerPair(radius, layer_idx + 1)];
                    if (max_anti_overhang_layer >= layer_idx + 1)
                    {
                        above = above.unionPolygons(anti_overhang_[layer_idx]);
                    }
                    else
                    {
                        above = above.unionPolygons(); // just to be sure the area is correctly unioned as otherwise difference may behave unexpectedly.
                    }
                    Polygons placeable = data[key].unionPolygons().difference(above);
                    data_placeable[RadiusLayerPair(radius, layer_idx + 1)] = data_placeable[RadiusLayerPair(radius, layer_idx + 1)].unionPolygons(placeable);
                }
            }

            // Add collision layers above to ensure correct support_top_distance.
            for (LayerIndex layer_idx = min_layer_bottom; layer_idx <= max_required_layer; layer_idx++)
            {
                key.second = layer_idx;
                for (coord_t layer_offset = 1; layer_offset <= z_distance_top_layers && layer_offset + layer_idx <= max_required_layer; layer_offset++)
                {
                    data[key].add(data[RadiusLayerPair(radius, layer_idx + layer_offset)]);
                }
                if (max_anti_overhang_layer >= layer_idx)
                {
                    data[key] = data[key].unionPolygons(anti_overhang_[layer_idx].offset(radius));
                }
                else
                {
                    data[key] = data[key].unionPolygons();
                }
            }

            for (LayerIndex layer_idx = max_required_layer; layer_idx > keys[i].second; layer_idx--)
            {
                data.erase(RadiusLayerPair(radius, layer_idx)); // all these dont have the correct z_distance_top_layers as they can still have areas above them
            }

            for (auto pair : data)
            {
                pair.second = simplifier.polygon(pair.second);
                data_outer[pair.first] = data_outer[pair.first].unionPolygons(pair.second);
            }
            if (radius == 0)
            {
                for (auto pair : data_placeable)
                {
                    pair.second = simplifier.polygon(pair.second);
                    data_placeable_outer[pair.first] = data_placeable_outer[pair.first].unionPolygons(pair.second);
                }
            }
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_progress);

            if (precalculated && precalculation_progress < TREE_PROGRESS_PRECALC_COLL)
            {
                precalculation_progress += TREE_PROGRESS_PRECALC_COLL / keys.size();
                Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
            }
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_collision_cache_);
            collision_cache_.insert(data_outer.begin(), data_outer.end());
        }
        if (radius == 0)
        {
            {
                std::lock_guard<std::mutex> critical_section(*critical_placeable_areas_cache_);
                placeable_areas_cache_.insert(data_placeable_outer.begin(), data_placeable_outer.end());
            }
        }
    });
}
void TreeModelVolumes::calculateCollisionHolefree(std::deque<RadiusLayerPair> keys)
{
    LayerIndex max_layer = 0;
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
        max_layer = std::max(max_layer, keys[i].second);
    }

    cura::parallel_for<coord_t>(0, max_layer + 1, // todo LayerIndex
        [&](const LayerIndex layer_idx)
        {
        std::unordered_map<RadiusLayerPair, Polygons> data;
        for (RadiusLayerPair key : keys)
        {
            // Logically increase the collision by increase_until_radius
            coord_t radius = key.first;
            coord_t increase_radius_ceil = ceilRadius(increase_until_radius, false) - ceilRadius(radius, true);
            Polygons col = getCollision(increase_until_radius, layer_idx, false).offset(5 - increase_radius_ceil, ClipperLib::jtRound).unionPolygons(); // this union is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
            col = simplifier.polygon(col);
            data[RadiusLayerPair(radius, layer_idx)] = col;
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_collision_cache_holefree_);
            collision_cache_holefree_.insert(data.begin(), data.end());
        }
    });
}


// ensures offsets are only done in sizes with a max step size per offset while adding the collision offset after each step, this ensures that areas cannot glitch through walls defined by the collision when offsetting to fast
Polygons TreeModelVolumes::safeOffset(const Polygons& me, coord_t distance, ClipperLib::JoinType jt, coord_t max_safe_step_distance, const Polygons& collision) const
{
    const size_t steps = std::abs(distance / max_safe_step_distance);
    assert(distance * max_safe_step_distance >= 0);
    Polygons ret = me;

    for (size_t i = 0; i < steps; i++)
    {
        ret = ret.offset(max_safe_step_distance, jt).unionPolygons(collision);
    }
    ret = ret.offset(distance % max_safe_step_distance, jt);

    return ret.unionPolygons(collision);
}

void TreeModelVolumes::calculateAvoidance(std::deque<RadiusLayerPair> keys)
{
    // For every RadiusLayer pair there are 3 avoidances that have to be calculate, calculated in the same paralell_for loop for better parallelization.
    const std::vector<AvoidanceType> all_types = { AvoidanceType::SLOW, AvoidanceType::FAST_SAFE, AvoidanceType::FAST };
    cura::parallel_for<size_t>(0, keys.size() * 3, // todo:In a perfect world this would be a parallel for nowait, but as currently (5.0,5.1) cura changes its parallelization so often reimplementing a non blocking parallel for each version is too much work, so until things have settled this will stay as a normal parallel for
        [&, keys, all_types](const size_t iter_idx)
        {
        size_t key_idx = iter_idx / 3;
        {
            size_t type_idx = iter_idx % all_types.size();
            AvoidanceType type = all_types[type_idx];
            const bool slow = type == AvoidanceType::SLOW;
            const bool holefree = type == AvoidanceType::FAST_SAFE;

            coord_t radius = keys[key_idx].first;
            LayerIndex max_required_layer = keys[key_idx].second;

            // do not calculate not needed safe avoidances
            if (holefree && radius >= increase_until_radius + current_min_xy_dist_delta)
            {
                return;
            }

            const coord_t offset_speed = slow ? max_move_slow_ : max_move_;
            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist * 1.9);
            RadiusLayerPair key(radius, 0);
            Polygons latest_avoidance;
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
            std::vector<std::pair<RadiusLayerPair, Polygons>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Polygons>(RadiusLayerPair(radius, -1), Polygons()));

            latest_avoidance = getAvoidance(radius, start_layer - 1, type, false, true); // minDist as the delta was already added, also avoidance for layer 0 will return the collision.

            // ### main loop doing the calculation
            for (LayerIndex layer = start_layer; layer <= max_required_layer; layer++)
            {
                key.second = layer;
                Polygons col;
                if ((slow && radius < increase_until_radius + current_min_xy_dist_delta) || holefree)
                {
                    col = getCollisionHolefree(radius, layer, true);
                }
                else
                {
                    col = getCollision(radius, layer, true);
                }

                latest_avoidance = safeOffset(latest_avoidance, -offset_speed, ClipperLib::jtRound, -max_step_move, col);
                latest_avoidance = simplifier.polygon(latest_avoidance);
                data[layer] = std::pair<RadiusLayerPair, Polygons>(key, latest_avoidance);
            }

            {
                std::lock_guard<std::mutex> critical_section(*critical_progress);

                if (precalculated && precalculation_progress < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO)
                {
                    precalculation_progress += support_rests_on_model ? 0.4 : 1 * TREE_PROGRESS_PRECALC_AVO / (keys.size() * 3);
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
                }
            }

            {
                std::lock_guard<std::mutex> critical_section(*(slow ? critical_avoidance_cache_slow_ : holefree ? critical_avoidance_cache_holefree_ : critical_avoidance_cache_));
                (slow ? avoidance_cache_slow_ : holefree ? avoidance_cache_hole_ : avoidance_cache_).insert(data.begin(), data.end());
            }
        }
    });
}

void TreeModelVolumes::calculatePlaceables(std::deque<RadiusLayerPair> keys)
{
    cura::parallel_for<size_t>(0, keys.size(), // todo:In a perfect world this would be a parallel for nowait, but as currently (5.0,5.1) cura changes its parallelization so often reimplementing a non blocking parallel for each version is too much work, so until things have settled this will stay as a normal parallel for
        [&, keys](const size_t key_idx)
        {
        const coord_t radius = keys[key_idx].first;
        const LayerIndex max_required_layer = keys[key_idx].second;
        std::vector<std::pair<RadiusLayerPair, Polygons>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Polygons>(RadiusLayerPair(radius, -1), Polygons()));
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
            data[0] = std::pair<RadiusLayerPair, Polygons>(key, machine_border_.difference(getCollision(radius, 0, true)));
            start_layer = 1;
        }

        for (LayerIndex layer = start_layer; layer <= max_required_layer; layer++)
        {
            key.second = layer;
            Polygons placeable = getPlaceableAreas(0, layer);
            placeable = simplifier.polygon(placeable); // it is faster to do this here in each thread than once in calculateCollision.
            placeable = placeable.offset(-(radius + (current_min_xy_dist + current_min_xy_dist_delta))).unionPolygons(); // todo comment
            data[layer] = std::pair<RadiusLayerPair, Polygons>(key, placeable);
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_progress);

            if (precalculated && precalculation_progress < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO)
            {
                precalculation_progress += 0.2 * TREE_PROGRESS_PRECALC_AVO / (keys.size());
                Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
            }
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_placeable_areas_cache_);
            placeable_areas_cache_.insert(data.begin(), data.end());
        }
    });
}


void TreeModelVolumes::calculateAvoidanceToModel(std::deque<RadiusLayerPair> keys)
{
    // For every RadiusLayer pair there are 3 avoidances that have to be calculated, calculated in the same parallel_for loop for better parallelization.
    const std::vector<AvoidanceType> all_types = { AvoidanceType::SLOW, AvoidanceType::FAST_SAFE, AvoidanceType::FAST };
    cura::parallel_for<size_t>(0, keys.size() * 3, // todo:In a perfect world this would be a parallel for nowait, but as currently (5.0,5.1) cura changes its parallelization so often reimplementing a non blocking parallel for each version is too much work, so until things have settled this will stay as a normal parallel for
        [&, keys, all_types](const size_t iter_idx)
        {
        size_t key_idx = iter_idx / 3;
        size_t type_idx = iter_idx % all_types.size();
        AvoidanceType type = all_types[type_idx];
        bool slow = type == AvoidanceType::SLOW;
        bool holefree = type == AvoidanceType::FAST_SAFE;
        coord_t radius = keys[key_idx].first;
        LayerIndex max_required_layer = keys[key_idx].second;

        // do not calculate not needed safe avoidances
        if (holefree && radius >= increase_until_radius + current_min_xy_dist_delta)
        {
            return;
        }
        getPlaceableAreas(radius, max_required_layer); // ensuring Placeableareas are calculated
        const coord_t offset_speed = slow ? max_move_slow_ : max_move_;
        const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist * 1.9);
        Polygons latest_avoidance;
        std::vector<std::pair<RadiusLayerPair, Polygons>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Polygons>(RadiusLayerPair(radius, -1), Polygons()));
        RadiusLayerPair key(radius, 0);

        LayerIndex start_layer;

        {
            std::lock_guard<std::mutex> critical_section(*(slow ? critical_avoidance_cache_to_model_slow_ : holefree ? critical_avoidance_cache_holefree_to_model_ : critical_avoidance_cache_to_model_));
            start_layer = 1 + getMaxCalculatedLayer(radius, slow ? avoidance_cache_to_model_slow_ : holefree ? avoidance_cache_hole_to_model_ : avoidance_cache_to_model_);
        }
        if (start_layer > max_required_layer)
        {
            spdlog::debug("Requested calculation for value already calculated ?");
            return;
        }
        start_layer = std::max(start_layer, LayerIndex(1));
        latest_avoidance = getAvoidance(radius, start_layer - 1, type, true, true); // minDist as the delta was already added, also avoidance for layer 0 will return the collision.

        // ### main loop doing the calculation
        for (LayerIndex layer = start_layer; layer <= max_required_layer; layer++)
        {
            key.second = layer;
            Polygons col = getCollision(radius, layer, true);

            if ((slow && radius < increase_until_radius + current_min_xy_dist_delta) || holefree)
            {
                col = getCollisionHolefree(radius, layer, true);
            }
            else
            {
                col = getCollision(radius, layer, true);
            }

            latest_avoidance = safeOffset(latest_avoidance, -offset_speed, ClipperLib::jtRound, -max_step_move, col).difference(getPlaceableAreas(radius, layer));
            latest_avoidance = simplifier.polygon(latest_avoidance);
            data[layer] = std::pair<RadiusLayerPair, Polygons>(key, latest_avoidance);
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_progress);

            if (precalculated && precalculation_progress < TREE_PROGRESS_PRECALC_COLL + TREE_PROGRESS_PRECALC_AVO)
            {
                precalculation_progress += 0.4 * TREE_PROGRESS_PRECALC_AVO / (keys.size() * 3);
                Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, TREE_PROGRESS_TOTAL);
            }
        }

        {
            std::lock_guard<std::mutex> critical_section(*(slow ? critical_avoidance_cache_to_model_slow_ : holefree ? critical_avoidance_cache_holefree_to_model_ : critical_avoidance_cache_to_model_));
            (slow ? avoidance_cache_to_model_slow_ : holefree ? avoidance_cache_hole_to_model_ : avoidance_cache_to_model_).insert(data.begin(), data.end());
        }
    });
}


void TreeModelVolumes::calculateWallRestrictions(std::deque<RadiusLayerPair> keys)
{
    // Wall restrictions are mainly important when they represent actual walls that are printed, and not "just" the configured z_distance, because technically valid placement is no excuse for moving through a wall.
    // As they exist to prevent accidentially moving though a wall at high speed between layers like thie (x = wall,i = influence area,o= empty space,d = blocked area because of z distance) Assume maximum movement distance is two characters and maximum safe movement distance of one character


    /* Potential issue addressed by the wall restrictions: Influence area may lag through a wall
     *  layer z+1:iiiiiiiiiiioooo
     *  layer z+0:xxxxxiiiiiiiooo
     *  layer z-1:ooooixxxxxxxxxx
     */

    // The radius for the upper collission has to be 0 as otherwise one may not enter areas that may be forbidden on layer_idx but not one below (c = not an influence area even though it should ):
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

    cura::parallel_for<size_t>(0, keys.size(), // todo:In a perfect world this would be a parallel for nowait, but as currently (5.0,5.1) cura changes its parallelization so often reimplementing a non blocking parallel for each version is too much work, so until things have settled this will stay as a normal parallel for
        [&, keys](const size_t key_idx)
        {
        coord_t radius = keys[key_idx].first;
        RadiusLayerPair key(radius, 0);
        coord_t min_layer_bottom;
        std::unordered_map<RadiusLayerPair, Polygons> data;
        std::unordered_map<RadiusLayerPair, Polygons> data_min;

        {
            std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_);
            min_layer_bottom = getMaxCalculatedLayer(radius, wall_restrictions_cache_);
        }

        if (min_layer_bottom < 1)
        {
            min_layer_bottom = 1;
        }
        for (LayerIndex layer_idx = min_layer_bottom; layer_idx <= keys[key_idx].second; layer_idx++)
        {
            key.second = layer_idx;
            LayerIndex layer_idx_below = layer_idx - 1;
            Polygons wall_restriction = simplifier.polygon(getCollision(0, layer_idx, false).intersection(getCollision(radius, layer_idx_below, true))); // radius contains current_min_xy_dist_delta already if required
            data.emplace(key, wall_restriction);
            if (current_min_xy_dist_delta > 0)
            {
                Polygons wall_restriction_min = simplifier.polygon(getCollision(0, layer_idx, true).intersection(getCollision(radius, layer_idx_below, true)));
                data_min.emplace(key, wall_restriction_min);
            }
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_);
            wall_restrictions_cache_.insert(data.begin(), data.end());
        }

        {
            std::lock_guard<std::mutex> critical_section(*critical_wall_restrictions_cache_min_);
            wall_restrictions_cache_min_.insert(data_min.begin(), data_min.end());
        }
    });
}

coord_t TreeModelVolumes::ceilRadius(coord_t radius) const
{
    if (radius == 0)
    {
        return 0;
    }

    if (radius <= radius_0)
    {
        return radius_0;
    }

    if (SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION)
    {
        // generate SUPPORT_TREE_PRE_EXPONENTIAL_STEPS of radiis before starting to exponentially increase them.

        coord_t exponential_result = SUPPORT_TREE_EXPONENTIAL_THRESHOLD * SUPPORT_TREE_EXPONENTIAL_FACTOR;
        const coord_t stepsize = (exponential_result - radius_0) / (SUPPORT_TREE_PRE_EXPONENTIAL_STEPS + 1);
        coord_t result = radius_0;
        for (size_t step = 0; step < SUPPORT_TREE_PRE_EXPONENTIAL_STEPS; step++)
        {
            result += stepsize;
            if (result >= radius && !ignorable_radii_.count(result))
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
    else
    { // generates equidistant steps of size SUPPORT_TREE_COLLISION_RESOLUTION starting from radius_0. If SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION then this code is dead, and can safely be removed.
        coord_t ceil_step_n = (radius - radius_0) / SUPPORT_TREE_COLLISION_RESOLUTION;
        coord_t resulting_ceil = radius_0 + (ceil_step_n + ((radius - radius_0) % SUPPORT_TREE_COLLISION_RESOLUTION != 0)) * SUPPORT_TREE_COLLISION_RESOLUTION;

        if (radius <= radius_0 && radius != 0)
        {
            return radius_0;
        }
        else if (ignorable_radii_.count(resulting_ceil))
        {
            return ceilRadius(resulting_ceil + 1);
        }
        else
        {
            return resulting_ceil;
        }
    }
}

template <typename KEY>
const std::optional<std::reference_wrapper<const Polygons>> TreeModelVolumes::getArea(const std::unordered_map<KEY, Polygons>& cache, const KEY key) const
{
    const auto it = cache.find(key);
    if (it != cache.end())
    {
        return std::optional<std::reference_wrapper<const Polygons>>{ it->second };
    }
    else
    {
        return std::optional<std::reference_wrapper<const Polygons>>();
    }
}


Polygons TreeModelVolumes::calculateMachineBorderCollision(const Polygons&& machine_border)
{
    Polygons machine_volume_border = machine_border.offset(MM2INT(1000.0)); // Put a border of 1 meter around the print volume so that we don't collide.
    machine_volume_border = machine_volume_border.difference(machine_border); // Subtract the actual volume from the collision area.
    return machine_volume_border;
}

} // namespace cura