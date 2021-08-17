//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "RedistributeBeadingStrategy.h"

#include <algorithm>
#include <numeric>

namespace cura
{

RedistributeBeadingStrategy::RedistributeBeadingStrategy(   const coord_t optimal_width_outer,
                                                            const coord_t optimal_width_inner,
                                                            const double minimum_variable_line_width,
                                                            BeadingStrategy* parent) :
        BeadingStrategy(parent->getOptimalWidth(), parent->getDefaultTransitionLength(), parent->getTransitioningAngle()),
        parent(parent),
        optimal_width_outer(optimal_width_outer),
        optimal_width_inner(optimal_width_inner),
        minimum_variable_line_width(minimum_variable_line_width)
{
    name = "RedistributeBeadingStrategy";
}

coord_t RedistributeBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    const coord_t inner_bead_count = bead_count > 2 ? bead_count - 2 : 0;
    const coord_t outer_bead_count = bead_count - inner_bead_count;

    return parent->getOptimalThickness(inner_bead_count) + optimal_width_outer * outer_bead_count;
}

coord_t RedistributeBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    return parent->getTransitionThickness(lower_bead_count);
}

coord_t RedistributeBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    return parent->getOptimalBeadCount(thickness);
}

coord_t RedistributeBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    return parent->getTransitioningLength(lower_bead_count);
}

float RedistributeBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    return parent->getTransitionAnchorPos(lower_bead_count);
}

std::string RedistributeBeadingStrategy::toString() const
{
    return toString() + parent->toString();
}

BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    Beading ret;
    if (bead_count > 2)
    {
        const coord_t inner_transition_width = optimal_width_inner * minimum_variable_line_width;
        const coord_t outer_bead_width =
            getOptimalOuterBeadWidth(thickness, optimal_width_outer, inner_transition_width);

        // Outer wall is locked in size and position for wall regions of 3 and higher which have at least a
        // thickness equal to two times the optimal outer width and the minimal inner wall width.
        const coord_t virtual_thickness = thickness - outer_bead_width * 2;
        const coord_t virtual_bead_count = bead_count - 2;

        // Calculate the beads and widths of the inner walls only
        ret = parent->compute(virtual_thickness, virtual_bead_count);

        // Insert the outer beads
        ret.bead_widths.insert(ret.bead_widths.begin(), outer_bead_width);
        ret.bead_widths.emplace_back(outer_bead_width);
    }
    else
    {
        ret = parent->compute(thickness, bead_count);
    }

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

coord_t RedistributeBeadingStrategy::getOptimalOuterBeadWidth(const coord_t thickness, const coord_t optimal_width_outer, const coord_t minimum_width_inner)
{
    const coord_t total_outer_optimal_width = optimal_width_outer * 2;
    coord_t outer_bead_width = thickness / 2;
    if (total_outer_optimal_width < thickness)
    {
        if (total_outer_optimal_width + minimum_width_inner > thickness)
        {
            outer_bead_width -= minimum_width_inner / 2;
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
    const size_t unfiltered_beads = beading.bead_widths.size();
    if(unfiltered_beads <= 2) //Outer walls are exempt. If there are 2 walls the range below will be empty. If there is 1 or 0 walls it would be invalid.
    {
        return false;
    }
    auto inner_begin = std::next(beading.bead_widths.begin());
    auto inner_end = std::prev(beading.bead_widths.end());
    beading.bead_widths.erase(
        std::remove_if(inner_begin, inner_end,
        [&minimum_width_inner](const coord_t width)
        {
            return width < minimum_width_inner;
        }),
        inner_end);
    return unfiltered_beads != beading.bead_widths.size();
    }

} // namespace cura
