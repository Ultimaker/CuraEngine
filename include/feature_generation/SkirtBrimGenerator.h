// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <settings/EnumSettings.h>

#include "feature_generation/FeatureGenerator.h"

namespace cura
{

enum class EPlatformAdhesion;

class SkirtBrimGenerator : public FeatureGenerator
{
public:
    explicit SkirtBrimGenerator();

    bool isActive() const override;

    void generateFeatures(const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const override;

private:
    EPlatformAdhesion adhesion_type_{ EPlatformAdhesion::NONE };
    bool support_brim_enable_{ false };
};

} // namespace cura