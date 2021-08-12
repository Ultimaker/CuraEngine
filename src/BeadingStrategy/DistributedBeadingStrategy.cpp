// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.
#include <numeric>
#include "DistributedBeadingStrategy.h"

namespace cura
{

DistributedBeadingStrategy::DistributedBeadingStrategy(const coord_t optimal_width,
                                const coord_t default_transition_length,
                                const AngleRadians transitioning_angle,
                                const Ratio wall_split_middle_threshold,
                                const Ratio wall_add_middle_threshold,
                                const int distribution_radius)
    : BeadingStrategy(optimal_width, default_transition_length, transitioning_angle)
    , wall_split_middle_threshold(wall_split_middle_threshold)
    , wall_add_middle_threshold(wall_add_middle_threshold)
{
    if(distribution_radius >= 1)
    {
        one_over_distribution_radius_squared = 1.0f / distribution_radius * 1.0f / distribution_radius;
    }
    else
    {
        one_over_distribution_radius_squared = 1.0f / 1 * 1.0f / 1;
    }
    name = "DistributedBeadingStrategy";
}

DistributedBeadingStrategy::Beading DistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 2)
    {
        const coord_t to_be_divided = thickness - bead_count * optimal_width;
        const float middle = static_cast<float>(bead_count - 1) / 2;

        const auto getWeight = [middle, this](coord_t bead_idx)
        {
            const float dev_from_middle = bead_idx - middle;
            return std::max(0.0f, 1.0f - one_over_distribution_radius_squared * dev_from_middle * dev_from_middle);
        };

        std::vector<float> weights;
        weights.resize(bead_count);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            weights[bead_idx] = getWeight(bead_idx);
        }

        const float total_weight = std::accumulate(weights.cbegin(), weights.cend(), 0.f);
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            const float weight_fraction = weights[bead_idx] / total_weight;
            const coord_t splitup_left_over_weight = to_be_divided * weight_fraction;
            const coord_t width = optimal_width + splitup_left_over_weight;
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

coord_t DistributedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return bead_count * optimal_width;
}

coord_t DistributedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    return lower_bead_count * optimal_width + optimal_width * (lower_bead_count % 2 == 1 ? wall_split_middle_threshold : wall_add_middle_threshold);
}

coord_t DistributedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    return (thickness + optimal_width / 2) / optimal_width;
}

} // namespace cura
