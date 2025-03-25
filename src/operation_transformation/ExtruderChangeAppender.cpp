// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/ExtruderChangeAppender.h"

#include <range/v3/view/sliding.hpp>

#include "print_operation/ExtruderChange.h"
#include "print_operation/ExtruderPlan.h"
#include "print_operation/ExtruderPlanPtr.h"
#include "print_operation/LayerPlan.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

ExtruderChangeAppender::ExtruderChangeAppender()
{
}

void ExtruderChangeAppender::process(PrintPlan* print_plan)
{
    std::vector<ExtruderPlanPtr> extruder_plans = print_plan->findOperationsByType<ExtruderPlan>(PrintOperationSequence::SearchOrder::Forward, 1);

    for (const auto consecutive_extruder_plans : extruder_plans | ranges::views::sliding(2))
    {
        const ExtruderPlanPtr& before = consecutive_extruder_plans[0];
        const ExtruderPlanPtr& after = consecutive_extruder_plans[1];

        if (before->getExtruderNr() != after->getExtruderNr())
        {
            LayerPlanPtr layer_plan = std::dynamic_pointer_cast<LayerPlan>(before->getParent());
            if (layer_plan)
            {
                auto extruder_change = std::make_shared<ExtruderChange>(before->getExtruderNr(), after->getExtruderNr());
                layer_plan->insertExtruderChangeAfter(before, extruder_change);
            }
            else
            {
                spdlog::error("Extruder plan doesn't have a layer plan as parent");
            }
        }
    }
}

} // namespace cura