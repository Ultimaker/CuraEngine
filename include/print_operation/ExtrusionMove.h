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
    explicit ExtrusionMove(const Point3LL& position, const coord_t line_width_start, const std::optional<coord_t>& line_width_end = std::nullopt);

    void write(PlanExporter& exporter) const override;

private:
    coord_t line_width_start_{ 0 };
    coord_t line_width_end_{ 0 };
};

} // namespace cura

#endif // PATHPLANNING_EXTRUSIONMOVE_H
