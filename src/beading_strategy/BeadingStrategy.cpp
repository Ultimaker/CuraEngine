// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "beading_strategy/BeadingStrategy.h"

#include <cassert>

namespace curaengine::beading_strategy
{

BeadingStrategy::BeadingStrategy(
    ::curaengine::utils::coord_t optimal_width,
    ::curaengine::types::Ratio wall_split_middle_threshold,
    ::curaengine::types::Ratio wall_add_middle_threshold,
    ::curaengine::utils::coord_t default_transition_length,
    double transitioning_angle)
    : name_("Unknown")
    , optimal_width_(optimal_width)
    , wall_split_middle_threshold_(wall_split_middle_threshold)
    , wall_add_middle_threshold_(wall_add_middle_threshold)
    , default_transition_length_(default_transition_length)
    , transitioning_angle_(transitioning_angle)
{
}

BeadingStrategy::BeadingStrategy(const BeadingStrategy& other)
    : name_(other.name_)
    , optimal_width_(other.optimal_width_)
    , wall_split_middle_threshold_(other.wall_split_middle_threshold_)
    , wall_add_middle_threshold_(other.wall_add_middle_threshold_)
    , default_transition_length_(other.default_transition_length_)
    , transitioning_angle_(other.transitioning_angle_)
{
}

::curaengine::utils::coord_t BeadingStrategy::getTransitioningLength(::curaengine::utils::coord_t lower_bead_count) const
{
    if (lower_bead_count == 0)
    {
        return 10;
    }
    return default_transition_length_;
}

double BeadingStrategy::getTransitionAnchorPos(::curaengine::utils::coord_t lower_bead_count) const
{
    ::curaengine::utils::coord_t lower_optimum = getOptimalThickness(lower_bead_count);
    ::curaengine::utils::coord_t transition_point = getTransitionThickness(lower_bead_count);
    ::curaengine::utils::coord_t upper_optimum = getOptimalThickness(lower_bead_count + 1);
    return 1.0 - static_cast<double>(transition_point - lower_optimum) / static_cast<double>(upper_optimum - lower_optimum);
}

std::vector<::curaengine::utils::coord_t> BeadingStrategy::getNonlinearThicknesses([[maybe_unused]] ::curaengine::utils::coord_t lower_bead_count) const
{
    return std::vector<::curaengine::utils::coord_t>();
}

std::string BeadingStrategy::toString() const
{
    return name_;
}

::curaengine::utils::coord_t BeadingStrategy::getDefaultTransitionLength() const
{
    return default_transition_length_;
}

::curaengine::utils::coord_t BeadingStrategy::getOptimalWidth() const
{
    return optimal_width_;
}

::curaengine::types::Ratio BeadingStrategy::getSplitMiddleThreshold() const
{
    return wall_split_middle_threshold_;
}

::curaengine::types::Ratio BeadingStrategy::getAddMiddleThreshold() const
{
    return wall_add_middle_threshold_;
}

::curaengine::types::AngleRadians BeadingStrategy::getTransitioningAngle() const
{
    return transitioning_angle_;
}

::curaengine::utils::coord_t BeadingStrategy::getOptimalThickness(::curaengine::utils::coord_t bead_count) const
{
    return optimal_width_ * bead_count;
}

::curaengine::utils::coord_t BeadingStrategy::getTransitionThickness(::curaengine::utils::coord_t lower_bead_count) const
{
    const ::curaengine::utils::coord_t lower_ideal_width = getOptimalThickness(lower_bead_count);
    const ::curaengine::utils::coord_t higher_ideal_width = getOptimalThickness(lower_bead_count + 1);
    const ::curaengine::types::Ratio threshold = lower_bead_count % 2 == 1 ? wall_split_middle_threshold_ : wall_add_middle_threshold_;
    return lower_ideal_width + threshold * (higher_ideal_width - lower_ideal_width);
}

} // namespace curaengine::beading_strategy
