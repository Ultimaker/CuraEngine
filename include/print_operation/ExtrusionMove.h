// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <settings/types/Ratio.h>
#include <settings/types/Velocity.h>

#include "print_operation/ExtruderMove.h"

namespace cura
{

class ExtrusionMove : public ExtruderMove
{
public:
    explicit ExtrusionMove(const Point3LL& position, const coord_t line_width_start, const Velocity& speed, const std::optional<coord_t>& line_width_end = std::nullopt);

    void write(PlanExporter& exporter) const override;

private:
    coord_t line_width_start_{ 0 };
    coord_t line_width_end_{ 0 };
    Velocity speed_;
    Ratio flow_ratio_{ 1.0_r };
};

} // namespace cura
