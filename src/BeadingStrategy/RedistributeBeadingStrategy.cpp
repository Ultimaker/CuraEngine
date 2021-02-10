//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RedistributeBeadingStrategy.h"

#include <algorithm>
#include <numeric>

namespace cura
{
    BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    {
        Beading ret = parent->compute(thickness, bead_count);

        // Early out when the only walls are outer, the parent can have been trusted to handle it. TODO: False! Now we need to lock postition
        if (bead_count < 3)
        {
            if (bead_count == 2 && optimal_width_outer < (thickness / 2))
            {
                const coord_t back_pos = bead_count - 1;

                // Compensate for the outer wall lock factor:
                ret.bead_widths[0] = static_cast<coord_t>(outer_wall_lock_factor * optimal_width_outer + outer_wall_lock_inverse * ret.bead_widths[0]);
                ret.bead_widths[back_pos] = static_cast<coord_t>(outer_wall_lock_factor * optimal_width_outer + outer_wall_lock_inverse * ret.bead_widths[back_pos]);
                ret.toolpath_locations[0] = ret.bead_widths[0] / 2;
                ret.toolpath_locations[back_pos] = thickness - ret.bead_widths[back_pos] / 2;

                // Check if a third wall needs to be created, if so, do it, otherwise put the rest-width in leftovers:
                coord_t left_over = std::max(static_cast<coord_t>(0), thickness - (ret.bead_widths.front() + ret.bead_widths.back()));
                if (left_over >= this->getTransitionThickness(0))
                {
                    ret.bead_widths.insert(std::next(ret.bead_widths.begin()), left_over);
                    ret.toolpath_locations.insert(std::next(ret.toolpath_locations.begin()), thickness / 2);
                    ret.left_over = 0;
                }
                else
                {
                    ret.left_over = left_over;
                }
            }
            return ret;
        }

        // Determine outer bead ratio
        const coord_t outer_width = ret.bead_widths.front() + ret.bead_widths.back();
        const coord_t outer_width_optimal = optimal_width_outer * 2;
        const coord_t outer_width_deviation = static_cast<coord_t>(static_cast<double>(outer_width_optimal - outer_width) * outer_wall_lock_factor + 0.5);
        const coord_t outer_width_required = outer_width + outer_width_deviation;
        const Ratio outer_bead_ratio = static_cast<double>(outer_width_required) / static_cast<double>(outer_width);

        // Update outer beads
        ret.bead_widths.front() = static_cast<coord_t>(static_cast<double>(ret.bead_widths.front()) * outer_bead_ratio + 0.5);
        ret.bead_widths.back() = static_cast<coord_t>(static_cast<double>(ret.bead_widths.back()) * outer_bead_ratio + 0.5);

        // Determine inner bead ratio
        auto inner_bead_width_begin = std::next(ret.bead_widths.begin());
        auto inner_bead_width_end = std::prev(ret.bead_widths.end());
        const coord_t total_inner_bead_width =
            std::accumulate(inner_bead_width_begin, inner_bead_width_end, static_cast<coord_t>(0));
        const coord_t total_required_inner_bead_width =
            thickness - ret.bead_widths.front() - ret.bead_widths.back();
        const Ratio inner_bead_ratio =
            static_cast<double>(total_required_inner_bead_width) / static_cast<double>(total_inner_bead_width);

        for (auto bead_width_it = inner_bead_width_begin; bead_width_it < inner_bead_width_end; bead_width_it++)
        {
            *bead_width_it = static_cast<coord_t>(static_cast<double>(*bead_width_it) * inner_bead_ratio + 0.5);
        }

        // Filter out bead_widths that violate the transition width and recalculate if needed
        ret.bead_widths.erase(std::remove_if(ret.bead_widths.begin(), ret.bead_widths.end(),
                                             [&](const coord_t width)
                                             {
                                                 return width <= parent->getTransitionThickness(0);
                                             }), ret.bead_widths.end());
        if (ret.bead_widths.size() < ret.toolpath_locations.size())
        {
            const coord_t left_over = thickness - std::accumulate(ret.bead_widths.cbegin(), ret.bead_widths.cend(), static_cast<coord_t>(0));
            logWarning("Removed a bead -> leftover %d, no_beads -> %d, you should update!\n", left_over, ret.bead_widths.size());
        }
        bead_count = ret.bead_widths.size();
        ret.toolpath_locations.resize(bead_count);

        // Update the first half of the toolpath-locations with the updated bead-widths (starting from 0, up to half):
        coord_t last_coord = 0;
        coord_t last_width = 0;
        for (coord_t i_location = 0; i_location < bead_count / 2; ++i_location)
        {
            ret.toolpath_locations[i_location] = last_coord + (last_width + ret.bead_widths[i_location]) / 2;
            last_coord = ret.toolpath_locations[i_location];
            last_width = ret.bead_widths[i_location];
        }

        // Handle the position of any middle wall (note that the width will already have been set correctly):
        if (bead_count % 2 == 1)
        {
            ret.toolpath_locations[bead_count / 2] = thickness / 2;
        }

        // Update the last half of the toolpath-locations with the updated bead-widths (starting from thickness, down to half):
        last_coord = thickness;
        last_width = 0;
        for (coord_t i_location = bead_count - 1; i_location >= bead_count - (bead_count / 2); --i_location)
        {
            ret.toolpath_locations[i_location] = last_coord - (last_width + ret.bead_widths[i_location]) / 2;
            last_coord = ret.toolpath_locations[i_location];
            last_width = ret.bead_widths[i_location];
        }

        ret.left_over = thickness - std::accumulate(ret.bead_widths.cbegin(), ret.bead_widths.cend(), static_cast<coord_t>(0));
        return ret;
    }

    } // namespace cura
