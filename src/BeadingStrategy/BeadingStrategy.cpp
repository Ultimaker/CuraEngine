//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cassert>

#include "BeadingStrategy.h"

namespace cura
{

BeadingStrategy::BeadingStrategy(coord_t optimal_width, coord_t default_transition_length, float transitioning_angle)
    : optimal_width(optimal_width)
    , default_transition_length(default_transition_length)
    , transitioning_angle(transitioning_angle)
{
    name = "Unknown";
}

coord_t BeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    if (lower_bead_count == 0)
    {
        return 10;
    }
    return default_transition_length;
}

float BeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    coord_t lower_optimum = getOptimalThickness(lower_bead_count);
    coord_t transition_point = getTransitionThickness(lower_bead_count);
    coord_t upper_optimum = getOptimalThickness(lower_bead_count + 1);
    return 1.0 - float(transition_point - lower_optimum) / float(upper_optimum - lower_optimum);
}

std::vector<coord_t> BeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
{
    return std::vector<coord_t>();
}

std::string BeadingStrategy::toString() const
{
    return name;
}

coord_t BeadingStrategy::getDefaultTransitionLength() const
{
    return default_transition_length;
}

coord_t BeadingStrategy::getOptimalWidth() const
{
    return optimal_width;
}

AngleRadians BeadingStrategy::getTransitioningAngle() const
{
    return transitioning_angle;
}


} // namespace cura
