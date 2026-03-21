// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/SliceDataStorageEnvironmentAdapter.h"

#include <cura-formulae-engine/parser/parser.h>

#include <range/v3/algorithm/contains.hpp>

#include "slice_data/MeshGroupSliceData.h"

namespace cfe = CuraFormulaeEngine;

namespace cura
{

SliceDataStorageEnvironmentAdapter::SliceDataStorageEnvironmentAdapter(const MeshGroupSliceData& storage, const CuraFormulaeEngine::env::Environment* next_environment)
    : storage_(storage)
    , next_environment_(next_environment)
    , additional_variables_(
          { "initial_layer_bb_min_x", "initial_layer_bb_max_x", "initial_layer_bb_min_y", "initial_layer_bb_max_y", "initial_layer_bb_width", "initial_layer_bb_height" })
{
}

std::optional<cfe::eval::Value> SliceDataStorageEnvironmentAdapter::get(const std::string& variable_id) const
{
    if (! ranges::contains(additional_variables_, variable_id))
    {
        return next_environment_ ? next_environment_->get(variable_id) : std::nullopt;
    }

    if (variable_id.starts_with("initial_layer_bb_"))
    {
        if (! initial_layer_bb_.has_value())
        {
            constexpr LayerIndex layer_nr = 0;
            constexpr bool include_support = true;
            constexpr bool include_prime_tower = true;
            constexpr bool external_polys_only = true;
            constexpr int extruder_nr = -1;
            constexpr bool include_models = true;
            const_cast<SliceDataStorageEnvironmentAdapter*>(this)->initial_layer_bb_
                = AABB(storage_.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_polys_only, extruder_nr, include_models));
        }

        coord_t initial_layer_bb_value = 0;
        if (variable_id.ends_with("min_x"))
        {
            initial_layer_bb_value = initial_layer_bb_->min_.X;
        }
        else if (variable_id.ends_with("max_x"))
        {
            initial_layer_bb_value = initial_layer_bb_->max_.X;
        }
        else if (variable_id.ends_with("min_y"))
        {
            initial_layer_bb_value = initial_layer_bb_->min_.Y;
        }
        else if (variable_id.ends_with("max_y"))
        {
            initial_layer_bb_value = initial_layer_bb_->max_.Y;
        }
        else if (variable_id.ends_with("width"))
        {
            initial_layer_bb_value = initial_layer_bb_->width();
        }
        else if (variable_id.ends_with("height"))
        {
            initial_layer_bb_value = initial_layer_bb_->height();
        }

        return cfe::eval::Value(INT2MM(initial_layer_bb_value));
    }

    return std::nullopt;
}

bool SliceDataStorageEnvironmentAdapter::has(const std::string& key) const
{
    return ranges::contains(additional_variables_, key) || (next_environment_ && next_environment_->has(key));
}

std::unordered_map<std::string, cfe::eval::Value> SliceDataStorageEnvironmentAdapter::getAll() const
{
    std::unordered_map<std::string, cfe::eval::Value> result;

    if (next_environment_)
    {
        result = next_environment_->getAll();
    }

    for (const std::string& key : additional_variables_)
    {
        std::optional<cfe::eval::Value> value = get(key);
        if (value.has_value())
        {
            result[key] = value.value();
        }
    }

    return result;
}

} // namespace cura
