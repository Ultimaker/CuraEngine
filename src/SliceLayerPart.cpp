// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SliceLayerPart.h"

#include <cmath>

namespace curaengine
{
using namespace types::geometry;
using namespace types;
using namespace utils;
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

Shape SliceLayerPart::getSupportedRegion(const coord_t layer_height, const AngleDegrees& overhang_angle) const
{
    if (overhang_angle < 90.0)
    {
        const coord_t overhang_width = layer_height * std::tan(AngleRadians(overhang_angle));
        return supported_extrusion_areas_below.offset(overhang_width + 2 * epsilon);
    }

    return Shape();
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

} // namespace curaengine
