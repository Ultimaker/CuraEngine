// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "feature_generation/FeatureGenerator.h"

namespace cura
{

class SliceMeshStorage;

class MeshFeatureGenerator : public FeatureGenerator
{
public:
    explicit MeshFeatureGenerator(const std::shared_ptr<SliceMeshStorage>& mesh);

    bool isActive() const override;

    void generateFeatures(const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const override;

private:
    std::shared_ptr<SliceMeshStorage> mesh_;
};

} // namespace cura
