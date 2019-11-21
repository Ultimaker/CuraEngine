//Copyright (c) 2019 Ultimaker B.V.


#include "InwardDistributedBeadingStrategy.h"

namespace arachne
{

InwardDistributedBeadingStrategy::Beading InwardDistributedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;

    ret.total_thickness = thickness;
    if (bead_count > 0)
    {
        coord_t to_be_divided = thickness - bead_count * optimal_width;
        float total_weight = 0;
        float middle = static_cast<float>(bead_count - 1) / 2;
        
        auto getWeight = [middle, this](coord_t bead_idx)
        {
            float dev_from_middle = bead_idx - middle;
//             if (dev_from_middle > 3) return 0.0f;
            return std::max(0.0f, 1.0f - one_over_distribution_radius_squared * dev_from_middle * dev_from_middle);
//             return 1.0f / (1.0 + .5*dev_from_middle * dev_from_middle * dev_from_middle * dev_from_middle);
//             return 1.0f / (.6 + sqrt(std::abs(dev_from_middle * dev_from_middle * dev_from_middle)));
//             return 1.0f / (.5 + sqrt(std::abs(dev_from_middle * dev_from_middle * dev_from_middle)));
        };
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            total_weight += getWeight(bead_idx);
        }
        for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
        {
            coord_t width = optimal_width + to_be_divided * getWeight(bead_idx) / total_weight;
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



} // namespace arachne
