// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "slice_data/SliceLayer.h"


namespace cura
{

Shape SliceLayer::getOutlines(bool external_polys_only) const
{
    Shape ret;
    getOutlines(ret, external_polys_only);
    return ret;
}

void SliceLayer::getOutlines(Shape& result, bool external_polys_only) const
{
    for (const SliceLayerPart& part : parts)
    {
        if (external_polys_only)
        {
            result.push_back(part.outline.outerPolygon());
        }
        else
        {
            result.push_back(part.print_outline);
        }
    }
}

} // namespace cura
