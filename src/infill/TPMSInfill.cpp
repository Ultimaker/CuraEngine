//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

// Implement a few TPMS (Triply Periodic Minimal Surfaces) infill patterns

#include "TPMSInfill.h"
#include "../utils/AABB.h"
#include "../utils/polygon.h"

namespace cura {

TPMSInfill::TPMSInfill(const bool zig_zaggify, const coord_t line_distance, const coord_t z, const EFillResolution resolution, const Point& infill_origin, const AngleDegrees fill_angle)
    : zig_zaggify(zig_zaggify)
    , line_distance(line_distance)
    , z(z)
    , resolution(resolution)
    , infill_origin(infill_origin)
    , fill_angle_rads(fill_angle / (180 / M_PI))
{
}

TPMSInfill::~TPMSInfill() {
}

Point TPMSInfill::rotate_around_origin(const Point& point, const double rads)
{
    return (rads != 0) ? infill_origin + rotate(point - infill_origin, rads) : point;
}

void TPMSInfill::generate(Polygons& result_lines, const Polygons& outline)
{
    Polygons rotated_outline = outline;
    if (fill_angle_rads != 0)
    {
        for (PolygonRef poly : rotated_outline)
        {
            for (Point& point : poly)
            {
                point = rotate_around_origin(point, -fill_angle_rads);
            }
        }
    }
    const AABB aabb(rotated_outline);

    // scale pitch so that total amount of filament used matches the amount used by the "line" infill pattern
    int pitch = line_distance * pitchScaling();
    int num_steps = 4;
    int step = pitch / num_steps;
    const int max_steps = (resolution == EFillResolution::LOW_RESOLUTION) ? 4 : (resolution == EFillResolution::MEDIUM_RESOLUTION) ? 8 : 16;
    while (step > 500 && num_steps < max_steps)
    {
        num_steps *= 2;
        step = pitch / num_steps;
    }
    pitch = step * num_steps; // recalculate to avoid precision errors

    x_min = infill_origin.X - std::ceil((float)(infill_origin.X - aabb.min.X) / pitch) * pitch;
    y_min = infill_origin.Y - std::ceil((float)(infill_origin.Y - aabb.min.Y) / pitch + 0.25) * pitch;
    x_max = infill_origin.X + std::ceil((float)(aabb.max.X - infill_origin.X) / pitch) * pitch;
    y_max = infill_origin.Y + std::ceil((float)(aabb.max.Y - infill_origin.Y) / pitch + 0.25) * pitch;

    generateCoordinates(result_lines, outline, pitch, step);

    if (zig_zaggify)
    {
        generateConnections(result_lines, outline);
    }
}

} // namespace cura