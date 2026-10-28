// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SliceLayerPart.h"

namespace curaengine
{
::curaengine::geometry::Shape& SliceLayerPart::getOwnInfillArea()
{
    return const_cast<::curaengine::geometry::Shape&>(const_cast<const SliceLayerPart*>(this)->getOwnInfillArea());
}

const ::curaengine::geometry::Shape& SliceLayerPart::getOwnInfillArea() const
{
    if (infill_area_own)
    {
        return *infill_area_own;
    }
    else
    {
        return infill_area;
    }
}

bool SliceLayerPart::hasWallAtInsetIndex(size_t inset_idx) const
{
    for (const ::curaengine::utils::VariableWidthLines& lines : wall_toolpaths)
    {
        for (const ::curaengine::utils::ExtrusionLine& line : lines)
        {
            if (line.inset_idx_ == inset_idx)
            {
                return true;
            }
        }
    }
    return false;
}

} // namespace curaengine
