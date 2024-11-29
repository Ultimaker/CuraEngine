// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_EXTRUSIONMOVE_H
#define PATHPLANNING_EXTRUSIONMOVE_H

#include "print_operation/ExtruderMove.h"
#include "settings/types/Ratio.h"

namespace cura
{

class ExtrusionMove : public ExtruderMove
{
public:
    explicit ExtrusionMove(const Point3LL& position, const Ratio& line_width_ratio = 1.0_r);

    void write(PlanExporter& exporter) const override;

private:
    Ratio line_width_ratio_{ 1.0 };
};

} // namespace cura

#endif // PATHPLANNING_EXTRUSIONMOVE_H
