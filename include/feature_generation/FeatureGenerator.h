// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <vector>

#include "print_operation/ExtruderPlanPtr.h"

namespace cura
{

struct LayerIndex;
class SliceDataStorage;

class FeatureGenerator
{
public:
    virtual ~FeatureGenerator() = default; // Force class being polymorphic

    virtual bool isActive() const = 0;

    virtual void generateFeatures(const SliceDataStorage& storage, const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const = 0;
};

} // namespace cura
