// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "feature_generation/FeatureGenerator.h"

namespace cura
{

class SliceMeshStorage;
class SliceLayerPart;

class MeshFeatureGenerator : public FeatureGenerator
{
public:
    explicit MeshFeatureGenerator(const std::shared_ptr<SliceMeshStorage>& mesh);

    bool isActive() const override;

    void generateFeatures(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const std::vector<ExtruderPlanPtr>& extruder_plans) const final;

protected:
    std::shared_ptr<SliceMeshStorage> getMesh() const;

    virtual void
        generateFeatures(const SliceDataStorage& storage, const LayerPlanPtr& layer_plan, const std::vector<ExtruderPlanPtr>& extruder_plans, const SliceLayerPart& part) const = 0;

private:
    std::shared_ptr<SliceMeshStorage> mesh_;
};

} // namespace cura
