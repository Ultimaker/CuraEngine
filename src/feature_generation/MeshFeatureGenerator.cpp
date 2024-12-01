// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/MeshFeatureGenerator.h"

namespace cura
{

inline MeshFeatureGenerator::MeshFeatureGenerator(const std::shared_ptr<SliceMeshStorage>& mesh)
    : mesh_(mesh)
{
}

inline bool MeshFeatureGenerator::isActive() const
{
    // Mesh generators are always active, you don't want to print e.g. only supports, right ?
    return true;
}

inline void MeshFeatureGenerator::generateFeatures(const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const
{
}

} // namespace cura