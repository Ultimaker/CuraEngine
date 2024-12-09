// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>

namespace cura
{

class LayerPlan;

using LayerPlanPtr = std::shared_ptr<LayerPlan>;
using ConstLayerPlanPtr = std::shared_ptr<const LayerPlan>;

} // namespace cura
