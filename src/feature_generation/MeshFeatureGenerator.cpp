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
    return ! mesh_->layers.empty();
}

void MeshFeatureGenerator::generateFeatures(const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const
{
}

} // namespace cura