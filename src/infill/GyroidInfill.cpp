// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/GyroidInfill.h"

#include <numbers>

#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "utils/AABB.h"

namespace cura
{

OpenLinesSet GyroidInfill::generateParallelLines(const coord_t line_distance, const Shape& in_outline, const coord_t z, const coord_t line_width) const
{
    // generate infill based on the gyroid equation: sin_x * cos_y + sin_y * cos_z + sin_z * cos_x = 0
    // kudos to the author of the Slic3r implementation equation code, the equation code here is based on that

    const AABB aabb(in_outline);

    int pitch = line_distance * 2.41; // this produces similar density to the "line" infill pattern
    int num_steps = 4;
    int step = pitch / num_steps;
    while (step > 500 && num_steps < 16)
    {
        num_steps *= 2;
        step = pitch / num_steps;
    }
    pitch = step * num_steps; // recalculate to avoid precision errors
    const double z_rads = 2 * std::numbers::pi * z / pitch;
    const double cos_z = std::cos(z_rads);
    const double sin_z = std::sin(z_rads);
    std::vector<coord_t> odd_line_coords;
    std::vector<coord_t> even_line_coords;
    OpenLinesSet result;
    if (std::abs(sin_z) <= std::abs(cos_z))
    {
        // "vertical" lines
        const double phase_offset = ((cos_z < 0) ? std::numbers::pi : 0) + std::numbers::pi;
        for (coord_t y = 0; y < pitch; y += step)
        {
            const double y_rads = 2 * std::numbers::pi * y / pitch;
            const double a = cos_z;
            const double b = std::sin(y_rads + phase_offset);
            const double odd_c = sin_z * std::cos(y_rads + phase_offset);
            const double even_c = sin_z * std::cos(y_rads + phase_offset + std::numbers::pi);
            const double h = std::sqrt(a * a + b * b);
            const double odd_x_rads = ((h != 0) ? std::asin(odd_c / h) + std::asin(b / h) : 0) - std::numbers::pi / 2;
            const double even_x_rads = ((h != 0) ? std::asin(even_c / h) + std::asin(b / h) : 0) - std::numbers::pi / 2;
            odd_line_coords.push_back(odd_x_rads / std::numbers::pi * pitch);
            even_line_coords.push_back(even_x_rads / std::numbers::pi * pitch);
        }
        const unsigned num_coords = odd_line_coords.size();
        unsigned num_columns = 0;
        for (coord_t x = (std::floor(aabb.min_.X / pitch) - 2.25) * pitch; x <= aabb.max_.X + pitch / 2; x += pitch / 2)
        {
            OpenPolyline line;
            for (coord_t y = (std::floor(aabb.min_.Y / pitch) - 1) * pitch; y <= aabb.max_.Y + pitch; y += pitch)
            {
                for (unsigned i = 0; i < num_coords; ++i)
                {
                    line.push_back(Point2LL(x + ((num_columns & 1) ? odd_line_coords[i] : even_line_coords[i]) / 2 + pitch, y + (coord_t)(i * step)));
                }
            }
            result.push_back(line);
            ++num_columns;
        }
    }
    else
    {
        // "horizontal" lines
        const double phase_offset = (sin_z < 0) ? std::numbers::pi : 0;
        for (coord_t x = 0; x < pitch; x += step)
        {
            const double x_rads = 2 * std::numbers::pi * x / pitch;
            const double a = sin_z;
            const double b = std::cos(x_rads + phase_offset);
            const double odd_c = cos_z * std::sin(x_rads + phase_offset + std::numbers::pi);
            const double even_c = cos_z * std::sin(x_rads + phase_offset);
            const double h = std::sqrt(a * a + b * b);
            const double odd_y_rads = ((h != 0) ? std::asin(odd_c / h) + std::asin(b / h) : 0) + std::numbers::pi / 2;
            const double even_y_rads = ((h != 0) ? std::asin(even_c / h) + std::asin(b / h) : 0) + std::numbers::pi / 2;
            odd_line_coords.push_back(odd_y_rads / std::numbers::pi * pitch);
            even_line_coords.push_back(even_y_rads / std::numbers::pi * pitch);
        }
        const unsigned num_coords = odd_line_coords.size();
        unsigned num_rows = 0;
        for (coord_t y = (std::floor(aabb.min_.Y / pitch) - 1) * pitch; y <= aabb.max_.Y + pitch / 2; y += pitch / 2)
        {
            OpenPolyline line;
            for (coord_t x = (std::floor(aabb.min_.X / pitch) - 1) * pitch; x <= aabb.max_.X + pitch; x += pitch)
            {
                for (unsigned i = 0; i < num_coords; ++i)
                {
                    line.push_back(Point2LL(x + (coord_t)(i * step), y + ((num_rows & 1) ? odd_line_coords[i] : even_line_coords[i]) / 2));
                }
            }
            result.push_back(line);
            ++num_rows;
        }
    }

    return result;
}

} // namespace cura
