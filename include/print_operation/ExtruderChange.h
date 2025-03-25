// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "ExtruderNumber.h"
#include "print_operation/PrintOperation.h"

namespace cura
{

class ExtruderChange : public PrintOperation
{
public:
    explicit ExtruderChange(const ExtruderNumber previous_extruder, const ExtruderNumber next_extruder);

    void write(PlanExporter& exporter) const override;

private:
    const ExtruderNumber previous_extruder_;
    const ExtruderNumber next_extruder_;
};

} // namespace cura
