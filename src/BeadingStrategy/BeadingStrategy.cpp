//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cassert>

#include "BeadingStrategy/BeadingStrategy.h"

namespace cura
{

BeadingStrategy::BeadingStrategy
(
    coord_t optimal_width,
    Ratio wall_split_middle_threshold,
    Ratio wall_add_middle_threshold,
    coord_t default_transition_length,
    float transitioning_angle
) :
    optimal_width(optimal_width),
    wall_split_middle_threshold(wall_split_middle_threshold),
    wall_add_middle_threshold(wall_add_middle_threshold),
    default_transition_length(default_transition_length),
    transitioning_angle(transitioning_angle)
{
    name = "Unknown";
}

BeadingStrategy::BeadingStrategy(const BeadingStrategy& other) :
    optimal_width(other.optimal_width),
    wall_split_middle_threshold(other.wall_split_middle_threshold),
    wall_add_middle_threshold(other.wall_add_middle_threshold),
    default_transition_length(other.default_transition_length),
    transitioning_angle(other.transitioning_angle),
    name(other.name)
{}

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

Ratio BeadingStrategy::getSplitMiddleThreshold() const
{
    return wall_split_middle_threshold;
}

Ratio BeadingStrategy::getAddMiddleThreshold() const
{
    return wall_add_middle_threshold;
}

AngleRadians BeadingStrategy::getTransitioningAngle() const
{
    return transitioning_angle;
}

coord_t BeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return optimal_width * bead_count;
}

coord_t BeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    const coord_t lower_ideal_width = getOptimalThickness(lower_bead_count);
    const coord_t higher_ideal_width = getOptimalThickness(lower_bead_count + 1);
    const Ratio threshold = lower_bead_count % 2 == 1 ? wall_split_middle_threshold : wall_add_middle_threshold;
    return lower_ideal_width + threshold * (higher_ideal_width - lower_ideal_width);
}

} // namespace cura
