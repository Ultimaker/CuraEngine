// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/ExtruderPlan.h"

#include <range/v3/algorithm/find_if.hpp>

#include "print_operation/FeatureExtrusion.h"

namespace cura
{

ExtruderPlan::ExtruderPlan(const size_t extruder_nr) noexcept
    : extruder_nr_(extruder_nr)
{
}

size_t ExtruderPlan::getExtruderNr() const noexcept
{
    return extruder_nr_;
}

void ExtruderPlan::appendFeatureExtrusion(const std::shared_ptr<FeatureExtrusion>& feature_extrusion, const bool check_non_empty)
{
    if (! check_non_empty || ! feature_extrusion->empty())
    {
        appendOperation(feature_extrusion);
    }
}

ExtruderPlanPtr ExtruderPlan::find(const std::vector<ExtruderPlanPtr>& extruder_plans, const size_t extruder_nr)
{
    auto iterator = ranges::find_if(
        extruder_plans,
        [&extruder_nr](const ExtruderPlanPtr& extruder_plan)
        {
            return extruder_plan->getExtruderNr() == extruder_nr;
        });

    if (iterator != extruder_plans.end())
    {
        return *iterator;
    }

    return nullptr;
}

} // namespace cura
