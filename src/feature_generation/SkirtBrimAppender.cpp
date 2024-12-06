// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "feature_generation/SkirtBrimAppender.h"

#include <Application.h>
#include <Slice.h>
#include <settings/EnumSettings.h>
#include <settings/Settings.h>

#include <range/v3/algorithm/contains.hpp>

#include "print_operation/ExtruderPlan.h"
#include "print_operation/PrintOperationPtr.h"

namespace cura
{

SkirtBrimAppender::SkirtBrimAppender()
{
}

void SkirtBrimAppender::process(PrintPlan* print_plan)
{
    const Settings& settings = Application::getInstance().current_slice_->scene.settings;
    const auto adhesion_type = settings.get<EPlatformAdhesion>("adhesion_type");
    const bool support_brim_enable = settings.get<bool>("support_brim_enable");

    if (adhesion_type != EPlatformAdhesion::SKIRT && adhesion_type != EPlatformAdhesion::BRIM && ! support_brim_enable)
    {
        return;
    }

    // Collect actually used extruders. At this point we assume that all added extruder plans are non-empty.
    std::set<size_t> used_extruders;
    print_plan->findOperation(
        [&used_extruders](const PrintOperationPtr& operation)
        {
            if (const auto extruder_plan = std::dynamic_pointer_cast<ExtruderPlan>(operation))
            {
                used_extruders.insert(extruder_plan->getExtruderNr());
            }
            return false;
        },
        PrintOperationSequence::SearchOrder::Forward,
        1);
}

} // namespace cura