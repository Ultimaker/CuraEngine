// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ExtruderMoveSet.h"

#include "path_planning/ExtruderMove.h"

namespace cura
{

bool ExtruderMoveSet::empty() const noexcept
{
    return extruder_moves_.empty();
}

void ExtruderMoveSet::write(PathExporter& exporter, const LayerPlan& layer_plan) const
{
    for (const std::shared_ptr<ExtruderMove>& extruder_move : extruder_moves_)
    {
        extruder_move->write(exporter, layer_plan, *this);
    }
}

coord_t ExtruderMoveSet::getZOffset() const
{
    return z_offset_;
}

const Ratio& ExtruderMoveSet::getSpeedFactor() const
{
    return speed_factor_;
}

const Ratio& ExtruderMoveSet::getSpeedBackPressureFactor() const
{
    return speed_back_pressure_factor_;
}

const Ratio& ExtruderMoveSet::getFlow() const
{
    return flow_;
}

const Ratio& ExtruderMoveSet::getWidthFactor() const
{
    return width_factor_;
}

void ExtruderMoveSet::appendExtruderMove(const std::shared_ptr<ExtruderMove>& extruder_move)
{
    extruder_moves_.push_back(extruder_move);
}

} // namespace cura
