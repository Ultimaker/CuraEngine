// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/PartsView.h"

#include <algorithm>
#include <vector>

#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"

namespace cura
{

size_t PartsView::getPartContaining(size_t poly_idx, size_t* boundary_poly_idx) const
{
    const PartsView& partsView = *this;
    for (size_t part_idx_now = 0; part_idx_now < partsView.size(); part_idx_now++)
    {
        const std::vector<size_t>& partView = partsView[part_idx_now];
        if (partView.size() == 0)
        {
            continue;
        }
        std::vector<size_t>::const_iterator result = std::find(partView.begin(), partView.end(), poly_idx);
        if (result != partView.end())
        {
            if (boundary_poly_idx)
            {
                *boundary_poly_idx = partView[0];
            }
            return part_idx_now;
        }
    }
    return NO_INDEX;
}

SingleShape PartsView::assemblePart(size_t part_idx) const
{
    const PartsView& partsView = *this;
    SingleShape ret;
    if (part_idx != NO_INDEX)
    {
        for (size_t poly_idx_ff : partsView[part_idx])
        {
            ret.push_back(polygons_[poly_idx_ff]);
        }
    }
    return ret;
}

SingleShape PartsView::assemblePartContaining(size_t poly_idx, size_t* boundary_poly_idx) const
{
    SingleShape ret;
    size_t part_idx = getPartContaining(poly_idx, boundary_poly_idx);
    if (part_idx != NO_INDEX)
    {
        return assemblePart(part_idx);
    }
    return ret;
}

} // namespace cura
