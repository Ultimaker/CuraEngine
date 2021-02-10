//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <numeric>

#include "InwardDistributedBeadingStrategy.h"

namespace cura
{

    InwardDistributedBeadingStrategy::Beading InwardDistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    {
        Beading ret;

        ret.total_thickness = thickness;
        if (bead_count > 0)
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
        else
        {
            ret.left_over = thickness;
        }

        return ret;
    }

} // namespace cura
