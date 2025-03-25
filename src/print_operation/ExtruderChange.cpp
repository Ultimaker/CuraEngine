// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/ExtruderChange.h"

#include "plan_export/PlanExporter.h"


namespace cura
{

ExtruderChange::ExtruderChange(const ExtruderNumber previous_extruder, const ExtruderNumber next_extruder)
    : PrintOperation()
    , previous_extruder_(previous_extruder)
    , next_extruder_(next_extruder)
{
}

void ExtruderChange::write(PlanExporter& exporter) const
{
    exporter.writeExtruderChange(next_extruder_);
}

} // namespace cura
