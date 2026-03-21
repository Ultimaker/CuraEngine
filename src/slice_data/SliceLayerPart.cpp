// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "slice_data/SliceLayerPart.h"


namespace cura
{

Shape& SliceLayerPart::getOwnInfillArea()
{
    return const_cast<Shape&>(const_cast<const SliceLayerPart*>(this)->getOwnInfillArea());
}

const Shape& SliceLayerPart::getOwnInfillArea() const
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
    for (const VariableWidthLines& lines : wall_toolpaths)
    {
        for (const ExtrusionLine& line : lines)
        {
            if (line.inset_idx_ == inset_idx)
            {
                return true;
            }
        }
    }
    return false;
}

} // namespace cura
