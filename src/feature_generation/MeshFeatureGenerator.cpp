// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshFeatureGenerator.h"

#include <sliceDataStorage.h>

namespace cura
{

MeshFeatureGenerator::MeshFeatureGenerator(const std::shared_ptr<SliceMeshStorage>& mesh)
    : mesh_(mesh)
{
}

bool MeshFeatureGenerator::isActive() const
{
    if (mesh_->layers.empty())
    {
        return false;
    }

    const Settings& mesh_settings = mesh_->settings;
    if (mesh_settings.get<bool>("anti_overhang_mesh") || mesh_settings.get<bool>("support_mesh"))
    {
        return false;
    }

    return true;
}

void MeshFeatureGenerator::generateFeatures(const SliceDataStorage& storage, const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const
{
    if (layer_index > mesh_->layer_nr_max_filled_layer)
    {
        return;
    }

    const SliceLayer& layer = mesh_->layers[layer_index];
    for (const SliceLayerPart& part : layer)
    {
        if (part.outline.empty())
        {
            continue;
        }

        generateFeatures(storage, layer_index, extruder_plans, part);
    }
}

std::shared_ptr<SliceMeshStorage> MeshFeatureGenerator::getMesh() const
{
    return mesh_;
}

} // namespace cura