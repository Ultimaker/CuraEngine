//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "sliceDataStorage.h"
#include "TreeModelVolumes.h"

namespace cura
{

TreeModelVolumes::TreeModelVolumes(const SliceDataStorage& storage, coord_t xy_distance, coord_t max_move,
                           coord_t radius_sample_resolution) :
    machine_border_{calculateMachineBorderCollision(storage.getMachineBorder())},
    xy_distance_{xy_distance},
    max_move_{max_move},
    radius_sample_resolution_{radius_sample_resolution}
{
    for (std::size_t layer_idx  = 0; layer_idx < storage.support.supportLayers.size(); ++layer_idx)
    {
        constexpr bool include_support = false;
        constexpr bool include_prime_tower = true;
        layer_outlines_.push_back(storage.getLayerOutlines(layer_idx, include_support, include_prime_tower));
    }
}

const Polygons& TreeModelVolumes::getCollision(coord_t radius, LayerIndex layer_idx) const
{
    radius = ceilRadius(radius);
    RadiusLayerPair key{radius, layer_idx};
    const auto it = collision_cache_.find(key);
    if (it != collision_cache_.end())
    {
        return it->second;
    }
    else
    {
        return calculateCollision(key);
    }
}

const Polygons& TreeModelVolumes::getAvoidance(coord_t radius, LayerIndex layer_idx) const
{
    radius = ceilRadius(radius);
    RadiusLayerPair key{radius, layer_idx};
    const auto it = avoidance_cache_.find(key);
    if (it != avoidance_cache_.end())
    {
        return it->second;
    }
    else
    {
        return calculateAvoidance(key);
    }
}

const Polygons& TreeModelVolumes::getInternalModel(coord_t radius, LayerIndex layer_idx) const
{
    radius = ceilRadius(radius);
    RadiusLayerPair key{radius, layer_idx};
    const auto it = internal_model_cache_.find(key);
    if (it != internal_model_cache_.end())
    {
        return it->second;
    }
    else
    {
        return calculateInternalModel(key);
    }
}

coord_t TreeModelVolumes::ceilRadius(coord_t radius) const
{
    const auto remainder = radius % radius_sample_resolution_;
    const auto delta = remainder != 0 ? radius_sample_resolution_- remainder : 0;
    return radius + delta;
}

const Polygons& TreeModelVolumes::calculateCollision(const RadiusLayerPair& key) const
{
    const auto& radius = key.first;
    const auto& layer_idx = key.second;

    auto collision_areas = machine_border_;
    if (layer_idx < static_cast<int>(layer_outlines_.size()))
    {
        collision_areas = collision_areas.unionPolygons(layer_outlines_[layer_idx]);
    }
    collision_areas = collision_areas.offset(xy_distance_ + radius, ClipperLib::JoinType::jtRound);
    const auto ret = collision_cache_.insert({key, std::move(collision_areas)});
    assert(ret.second);
    return ret.first->second;
}

const Polygons& TreeModelVolumes::calculateAvoidance(const RadiusLayerPair& key) const
{
    const auto& radius = key.first;
    const auto& layer_idx = key.second;

    if (layer_idx == 0)
    {
        avoidance_cache_[key] = getCollision(radius, 0);
        return avoidance_cache_[key];
    }

    // Avoidance for a given layer depends on all layers beneath it so could have very deep recursion depths if
    // called at high layer heights. We can limit the reqursion depth to N by checking if the if the layer N
    // below the current one exists and if not, forcing the calculation of that layer. This may cause another recursion
    // if the layer at 2N below the current one but we won't exceed our limit unless there are N*N uncalculated layers
    // below our current one.
    constexpr auto max_recursion_depth = 100;
    // Check if we would exceed the recursion limit by trying to process this layer
    if (layer_idx >= max_recursion_depth
        && avoidance_cache_.find({radius, layer_idx - max_recursion_depth}) == avoidance_cache_.end())
    {
        // Force the calculation of the layer `max_recursion_depth` below our current one, ignoring the result.
        getAvoidance(radius, layer_idx - max_recursion_depth);
    }
    auto avoidance_areas = getAvoidance(radius, layer_idx - 1).offset(-max_move_).smooth(5);
    avoidance_areas = avoidance_areas.unionPolygons(getCollision(radius, layer_idx));
    const auto ret = avoidance_cache_.insert({key, std::move(avoidance_areas)});
    assert(ret.second);
    return ret.first->second;
}

const Polygons& TreeModelVolumes::calculateInternalModel(const RadiusLayerPair& key) const
{
    const auto& radius = key.first;
    const auto& layer_idx = key.second;

    const auto& internal_areas = getAvoidance(radius, layer_idx).difference(getCollision(radius, layer_idx));
    const auto ret = internal_model_cache_.insert({key, internal_areas});
    assert(ret.second);
    return ret.first->second;
}

Polygons TreeModelVolumes::calculateMachineBorderCollision(Polygon machine_border)
{
    Polygons machine_volume_border;
    machine_volume_border.add(machine_border.offset(MM2INT(1000))); //Put a border of 1m around the print volume so that we don't collide.
    machine_border.reverse(); //Makes the polygon negative so that we subtract the actual volume from the collision area.
    machine_volume_border.add(machine_border);
    return machine_volume_border;
}

}