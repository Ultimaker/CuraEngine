// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_EXTRUDERPLANSCHEDULER_H
#define PATHPROCESSING_EXTRUDERPLANSCHEDULER_H

#include <ranges>

#include "path_planning/ExtruderPlan.h"
#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class ExtruderPlanScheduler final : public PrintOperationProcessor<ExtruderPlan>
{
public:
    void process(ExtruderPlan* extruder_plan) override;

private:
#warning initialize with extruder start position
    Point3LL current_position_;
};

} // namespace cura

#endif // PATHPROCESSING_EXTRUDERPLANSCHEDULER_H
