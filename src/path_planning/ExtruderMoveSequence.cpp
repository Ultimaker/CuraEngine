// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ExtruderMoveSequence.h"

#include "path_planning/ExtruderMove.h"

namespace cura
{

coord_t ExtruderMoveSequence::getZOffset() const
{
    return z_offset_;
}

const Ratio& ExtruderMoveSequence::getSpeedFactor() const
{
    return speed_factor_;
}

const Ratio& ExtruderMoveSequence::getSpeedBackPressureFactor() const
{
    return speed_back_pressure_factor_;
}

void ExtruderMoveSequence::appendExtruderMove(const std::shared_ptr<ExtruderMove>& extruder_move)
{
    appendOperation(extruder_move);
}

} // namespace cura
