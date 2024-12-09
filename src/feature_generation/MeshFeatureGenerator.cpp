// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshFeatureGenerator.h"

#include <sliceDataStorage.h>

#include "print_operation/LayerPlan.h"

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

void MeshFeatureGenerator::generateFeatures(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const std::vector<ExtruderPlanPtr>& extruder_plans) const
{
    if (layer_plan->getLayerIndex() > mesh_->layer_nr_max_filled_layer)
    {
        return;
    }

    const SliceLayer& layer = mesh_->layers[layer_plan->getLayerIndex()];
    for (const SliceLayerPart& part : layer.parts)
    {
        if (part.outline.empty())
        {
            continue;
        }

        generateFeatures(storage, layer_plan, extruder_plans, part);
    }
}

std::shared_ptr<SliceMeshStorage> MeshFeatureGenerator::getMesh() const
{
    return mesh_;
}

} // namespace cura