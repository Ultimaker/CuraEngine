// Copyright (c) 2026 CuraEngine contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_CROSSHATCHINFILL_H
#define INFILL_CROSSHATCHINFILL_H

#include "infill/AbstractLinesInfill.h"

namespace cura
{

/*!
 * Cross Hatch alternates stacks of parallel lines with perpendicular stacks.
 * Corrugated transition layers bond the two orientations together.
 *
 * Based on Bambu Lab's Cross Hatch algorithm (also used by OrcaSlicer), whose
 * transition geometry was inspired by David Eccles' improved 3D honeycomb.
 * The low-density repeat-height adjustment follows OrcaSlicer.
 * Transition progress stops at the shared diagonal geometry and starts at a
 * straight line, avoiding density-dependent jumps between adjacent layers.
 */
class CrossHatchInfill : public AbstractLinesInfill
{
public:
    CrossHatchInfill()
        : AbstractLinesInfill(1)
    {
    }

    void generateInfill(
        OpenLinesSet& result_polylines,
        Shape& result_polygons,
        bool zig_zaggify,
        coord_t line_distance,
        const Shape& in_outline,
        coord_t z,
        coord_t line_width,
        const AngleDegrees& rotation) const;

protected:
    OpenLinesSet generateParallelLines(coord_t line_distance, const AABB& bounding_box, coord_t z, coord_t line_width) const override;
};

} // namespace cura

#endif
