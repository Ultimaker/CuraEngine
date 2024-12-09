// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "feature_generation/MeshFeatureGenerator.h"

namespace cura
{

class SliceLayerPart;

class MeshInsetsGenerator : public MeshFeatureGenerator
{
public:
    explicit MeshInsetsGenerator(const std::shared_ptr<SliceMeshStorage>& mesh);

    bool isActive() const override;

protected:
    void generateFeatures(const SliceDataStorage& storage, const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans, const SliceLayerPart& part)
        const override;
};

} // namespace cura