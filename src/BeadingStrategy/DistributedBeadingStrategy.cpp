// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.
#include "BeadingStrategy/DistributedBeadingStrategy.h"

#include <numeric>

namespace cura
{

DistributedBeadingStrategy::DistributedBeadingStrategy(
    const coord_t optimal_width,
    const coord_t default_transition_length,
    const AngleRadians transitioning_angle,
    const Ratio wall_split_middle_threshold,
    const Ratio wall_add_middle_threshold,
    const int distribution_radius)
    : BeadingStrategy(optimal_width, wall_split_middle_threshold, wall_add_middle_threshold, default_transition_length, transitioning_angle)
{
    if (distribution_radius >= 2)
    {
        one_over_distribution_radius_squared_ = 1.0f / (distribution_radius - 1) * 1.0f / (distribution_radius - 1);
    }
    else
    {
        one_over_distribution_radius_squared_ = 1.0f / 1 * 1.0f / 1;
    }
    name_ = "DistributedBeadingStrategy";
}

DistributedBeadingStrategy::Beading DistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 2)
    {
        const coord_t to_be_divided = thickness - bead_count * optimal_width_;
        const double middle = static_cast<double>(bead_count - 1) / 2;

        const auto getWeight = [middle, this](coord_t bead_idx)
        {
            const double dev_from_middle = bead_idx - middle;
            return std::max(0.0, 1.0 - one_over_distribution_radius_squared_ * dev_from_middle * dev_from_middle);
        };

        std::vector<double> weights;
        weights.resize(bead_count);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            weights[bead_idx] = getWeight(bead_idx);
        }

        const double total_weight = std::accumulate(weights.cbegin(), weights.cend(), 0.0);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            const double weight_fraction = weights[bead_idx] / total_weight;
            const coord_t splitup_left_over_weight = to_be_divided * weight_fraction;
            const coord_t width = optimal_width_ + splitup_left_over_weight;
            if (bead_idx == 0)
            {
                ret.toolpath_locations.emplace_back(width / 2);
            }
            else
            {
                ret.toolpath_locations.emplace_back(ret.toolpath_locations.back() + (ret.bead_widths.back() + width) / 2);
            }
            ret.bead_widths.emplace_back(width);
        }
        ret.left_over = 0;
    }
    else if (bead_count == 2)
    {
        const coord_t outer_width = thickness / 2;
        ret.bead_widths.emplace_back(outer_width);
        ret.bead_widths.emplace_back(outer_width);
        ret.toolpath_locations.emplace_back(outer_width / 2);
        ret.toolpath_locations.emplace_back(thickness - outer_width / 2);
        ret.left_over = 0;
    }
    else if (bead_count == 1)
    {
        const coord_t outer_width = thickness;
        ret.bead_widths.emplace_back(outer_width);
        ret.toolpath_locations.emplace_back(outer_width / 2);
        ret.left_over = 0;
    }
    else
    {
        ret.left_over = thickness;
    }

    return ret;
}

coord_t DistributedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    const coord_t naive_count = thickness / optimal_width_; // How many lines we can fit in for sure.
    const coord_t remainder = thickness - naive_count * optimal_width_; // Space left after fitting that many lines.
    const coord_t minimum_line_width = optimal_width_ * (naive_count % 2 == 1 ? wall_split_middle_threshold_ : wall_add_middle_threshold_);
    return naive_count + (remainder >= minimum_line_width); // If there's enough space, fit an extra one.
}

} // namespace cura
