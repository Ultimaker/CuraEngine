// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>

namespace cura
{

class ExtruderPlan;

using ExtruderPlanPtr = std::shared_ptr<ExtruderPlan>;
using ConstExtruderPlanPtr = std::shared_ptr<const ExtruderPlan>;

} // namespace cura
