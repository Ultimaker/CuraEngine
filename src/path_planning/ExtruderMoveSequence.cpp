// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ExtruderMoveSequence.h"

#include "path_planning/ExtrusionMove.h"

namespace cura
{

ExtruderMoveSequence::ExtruderMoveSequence(const Point3LL& start_position)
    : start_position_(start_position)
{
}

const Point3LL& ExtruderMoveSequence::getStartPosition() const
{
    return start_position_;
}

std::optional<Point3LL> ExtruderMoveSequence::findStartPosition() const
{
    return start_position_;
}

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

void ExtruderMoveSequence::appendExtruderMove(const Point3LL& position, const Ratio& line_width_ratio)
{
    appendOperation(std::make_shared<ExtrusionMove>(position, line_width_ratio));
}

} // namespace cura
