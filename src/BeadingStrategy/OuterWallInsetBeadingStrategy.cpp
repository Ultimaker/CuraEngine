//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "OuterWallInsetBeadingStrategy.h"

#include <algorithm>

namespace cura
{
    BeadingStrategy::Beading OuterWallInsetBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    {
        Beading ret = parent->compute(thickness, bead_count);

        // Actual count and thickness as represented by extant walls. Don't count any potential zero-width 'signalling' walls.
        bead_count = std::count_if(ret.bead_widths.begin(), ret.bead_widths.end(), [](const coord_t width) { return width > 0; });

        // No need to apply any inset if there is just a single wall.
        if (bead_count < 2)
        {
            return ret;
        }
        
        // Actually move the outer wall inside. Ensure that the outer wall never goes beyond the middle line.
        ret.toolpath_locations[0] = std::min(ret.toolpath_locations[0] + outer_wall_offset, thickness / 2);
        return ret;
    }

} // namespace cura
