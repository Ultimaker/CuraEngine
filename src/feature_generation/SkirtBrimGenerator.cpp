// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/SkirtBrimGenerator.h"

#include <Application.h>
#include <settings/EnumSettings.h>
#include <settings/Settings.h>

namespace cura
{

SkirtBrimGenerator::SkirtBrimGenerator()
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;
    adhesion_type_ = settings.get<EPlatformAdhesion>("adhesion_type");
    support_brim_enable_ = settings.get<bool>("support_brim_enable");
}

inline bool SkirtBrimGenerator::isActive() const
{
    return adhesion_type_ == EPlatformAdhesion::SKIRT || adhesion_type_ == EPlatformAdhesion::BRIM || support_brim_enable_;
}

inline void SkirtBrimGenerator::generateFeatures(const LayerIndex& layer_index, const std::vector<ExtruderPlanPtr>& extruder_plans) const
{
}

} // namespace cura