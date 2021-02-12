//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RedistributeBeadingStrategy.h"

#include <algorithm>
#include <numeric>
#include <cassert>

namespace cura
{
BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (bead_count > 2)
    {
        const coord_t inner_transition_width = optimal_width_inner * minimum_variable_line_width;
        const coord_t outer_bead_width =
            getOptimalOuterBeadWidth(thickness, optimal_width_outer, inner_transition_width);

        coord_t virtual_thickness = thickness;
        coord_t virtual_bead_count = bead_count;

        if (outer_wall_lock)
        {
            // Outer wall is locked in size en position for wall regions of 3 and higher which have at least a
            // thickness equal to two times the optimal outer width and the minimal inner wall width.
            virtual_thickness -= outer_bead_width * 2;
            virtual_bead_count -= 2;
        }

        // Calculate the beads and widths of the inner walls only
        Beading ret = parent->compute(virtual_thickness, virtual_bead_count);

        if (outer_wall_lock)
        {
            // Insert the outer beads
            ret.bead_widths.insert(ret.bead_widths.begin(), outer_bead_width);
            ret.bead_widths.emplace_back(outer_bead_width);
        }
        else
        {
            const double current_total_outer_bead_width = static_cast<double>(ret.bead_widths.front() + ret.bead_widths.back());
            const double optimal_total_outer_bead_width = static_cast<double>(outer_bead_width * 2);
            const Ratio outer_bead_ratio = optimal_total_outer_bead_width / current_total_outer_bead_width;
            ret.bead_widths.front() = ret.bead_widths.front() * outer_bead_ratio;
            ret.bead_widths.back() = ret.bead_widths.back() * outer_bead_ratio;

            const double current_total_inner_bead_width = static_cast<double>(thickness - current_total_outer_bead_width);
            const double optimal_total_inner_bead_width = static_cast<double>(optimal_width_inner * (bead_count - 2));
            const Ratio inner_bead_ratio = optimal_total_inner_bead_width / current_total_inner_bead_width;
            auto inner_begin = std::next(ret.bead_widths.begin());
            auto inner_end = std::prev(ret.bead_widths.end());
            std::transform(inner_begin, inner_end, inner_begin,
                [&inner_bead_ratio](const coord_t width)
                           {
                               return width * inner_bead_ratio;
                           });
        }

        // Filter out beads that violate the minimum inner wall widths and recompute if necessary
        const bool removed_inner_beads = validateInnerBeadWidths(ret, inner_transition_width);
        if (removed_inner_beads)
        {
            ret = compute(thickness, bead_count - 1);
        }

        // Ensure that the positions of the beads are distributed over the thickness
        resetToolPathLocations(ret, thickness);

        return ret;
    }
    Beading ret = parent->compute(thickness, bead_count);

    // Filter out beads that violate the minimum inner wall widths and recompute if necessary
    const coord_t outer_transition_width = optimal_width_inner * minimum_variable_line_width;
    const bool removed_inner_beads = validateInnerBeadWidths(ret, outer_transition_width);
    if (removed_inner_beads)
    {
        ret = compute(thickness, bead_count - 1);
    }

    // Ensure that the positions of the beads are distributed over the thickness
    resetToolPathLocations(ret, thickness);

    return ret;
}

coord_t RedistributeBeadingStrategy::getOptimalOuterBeadWidth(const coord_t thickness, const coord_t optimal_width_outer, const coord_t inner_transition_width)
{
    const coord_t total_outer_optimal_width = optimal_width_outer * 2;
    coord_t outer_bead_width = thickness / 2;
    if (total_outer_optimal_width < thickness)
    {
        if (total_outer_optimal_width + inner_transition_width > thickness)
        {
            outer_bead_width -= inner_transition_width / 2;
        }
        else
        {
            outer_bead_width = optimal_width_outer;
        }
    }
    return outer_bead_width;
}

void RedistributeBeadingStrategy::resetToolPathLocations(BeadingStrategy::Beading& beading, const coord_t thickness)
{
    const size_t bead_count = beading.bead_widths.size();
    beading.toolpath_locations.resize(bead_count);

    if (bead_count < 1)
    {
        beading.toolpath_locations.resize(0);
        beading.total_thickness = thickness;
        beading.left_over = thickness;
        return;
    }

    // Update the first half of the toolpath-locations with the updated bead-widths (starting from 0, up to half):
    coord_t last_coord = 0;
    coord_t last_width = 0;
    for (size_t i_location = 0; i_location < bead_count / 2; ++i_location)
    {
        beading.toolpath_locations[i_location] = last_coord + (last_width + beading.bead_widths[i_location]) / 2;
        last_coord = beading.toolpath_locations[i_location];
        last_width = beading.bead_widths[i_location];
    }

    // Handle the position of any middle wall (note that the width will already have been set correctly):
    if (bead_count % 2 == 1)
    {
        beading.toolpath_locations[bead_count / 2] = thickness / 2;
    }

    // Update the last half of the toolpath-locations with the updated bead-widths (starting from thickness, down to half):
    last_coord = thickness;
    last_width = 0;
    for (size_t i_location = bead_count - 1; i_location >= bead_count - (bead_count / 2); --i_location)
    {
        beading.toolpath_locations[i_location] = last_coord - (last_width + beading.bead_widths[i_location]) / 2;
        last_coord = beading.toolpath_locations[i_location];
        last_width = beading.bead_widths[i_location];
    }

    // Ensure correct total and left over thickness
    beading.total_thickness = thickness;
    beading.left_over = thickness - std::accumulate(beading.bead_widths.cbegin(), beading.bead_widths.cend(), static_cast<coord_t>(0));
}

bool RedistributeBeadingStrategy::validateInnerBeadWidths(BeadingStrategy::Beading& beading, const coord_t minimum_width_inner)
{
    // Filter out bead_widths that violate the transition width and recalculate if needed
    const size_t unfilltered_beads = beading.bead_widths.size();
    auto inner_begin = std::next(beading.bead_widths.begin());
    auto inner_end = std::prev(beading.bead_widths.end());
    beading.bead_widths.erase(
        std::remove_if(inner_begin, inner_end,
                       [&minimum_width_inner](const coord_t width)
                       {
                           return width < minimum_width_inner;
                       }),
        inner_end);
    return unfilltered_beads != beading.bead_widths.size();
}

} // namespace cura
