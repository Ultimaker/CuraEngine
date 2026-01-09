// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/RegularNGonalInfill.h"

#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "geometry/conversions/Point2D_Point2LL.h"
#include "utils/AABB.h"

namespace cura
{

RegularNGonalInfill::RegularNGonalInfill(RegularNGonType type)
    : type_(type)
{
}

OpenLinesSet RegularNGonalInfill::generateParallelLines(const coord_t line_distance, const Shape& in_outline, const coord_t z, const coord_t line_width) const
{
    std::array<SegmentsPattern, 2> patterns;
    coord_t delta_y = 0;
    coord_t pattern_width = 0;
    coord_t pattern_height = 0;

    const double circumradius = line_distance / (std::cos(std::numbers::pi / 6));

    switch (type_)
    {
    case RegularNGonType::Octagon:
    {
        const double segment_length = (std::cos(std::numbers::pi / 8) * circumradius) / 2.0;
        const double segment_right_side = segment_length / std::sqrt(2.0);

        patterns = generatePatterns(segment_length, segment_right_side, segment_right_side, std::llrint(segment_length / 2));
        delta_y = -(segment_length * 1.5 + segment_right_side);
        pattern_width = 2 * segment_length + 2 * segment_right_side;
        pattern_height = pattern_width;
        break;
    }

    case RegularNGonType::Hexagon:
    {
        // Theoretical dimensions of the right segment
        const double segment_right_full_x = circumradius * std::cos(std::numbers::pi / 6);
        const double segment_right_full_y = circumradius * std::sin(std::numbers::pi / 6);

        // Actual dimensions of the right segment, applied with the line_width reduction so that adjacent hexagons don't have overlapping lines
        const double segment_right_x = segment_right_full_x - line_width;
        const double segment_right_y = segment_right_x * std::tan(std::numbers::pi / 6);

        const double segment_up_length = circumradius + (segment_right_full_y - segment_right_y);

        patterns = generatePatterns(segment_up_length, segment_right_x, segment_right_y, line_width / 2);
        delta_y = -(segment_up_length * 1.5 + segment_right_y);
        pattern_width = line_distance * 2;
        pattern_height = segment_up_length * 2 + segment_right_y * 2;
        break;
    }
    }

    return generateRegularNGonalLines(in_outline, patterns, delta_y, pattern_width, pattern_height);
}

OpenLinesSet RegularNGonalInfill::generateRegularNGonalLines(
    const Shape& in_outline,
    const std::array<SegmentsPattern, 2>& patterns,
    const coord_t delta_y,
    const coord_t pattern_width,
    const coord_t pattern_height)
{
    if (pattern_width <= 0 || pattern_height <= 0)
    {
        return {};
    }

    const AABB aabb(in_outline);

    const int start_col = aabb.min_.X / pattern_width - 1;
    const int end_col = (aabb.max_.X / pattern_width) + 1;
    const int start_row = aabb.min_.Y / pattern_height - 1;
    const int end_row = (aabb.max_.Y / pattern_height) + 1;

    OpenLinesSet raw_lines;
    for (int col = start_col; col <= end_col; ++col)
    {
        for (const SegmentsPattern& pattern : patterns)
        {
            OpenPolyline raw_line;

            Point2LL current_position(pattern.delta_x + col * pattern_width, delta_y + start_row * pattern_height);
            raw_line.push_back(current_position);

            for (int row = start_row; row <= end_row; ++row)
            {
                for (const Point2LL& segment : pattern.segments)
                {
                    current_position += segment;
                    raw_line.push_back(current_position);
                }
            }

            raw_lines.push_back(raw_line);
        }
    }

    return raw_lines;
}

std::array<RegularNGonalInfill::SegmentsPattern, 2>
    RegularNGonalInfill::generatePatterns(const double segment_up_length, const double segment_right_x, const double segment_right_y, const coord_t absolute_delta_x)
{
    const Point2LL segment_up = toPoint2LL(Point2D(0, segment_up_length));
    const Point2LL segment_right = toPoint2LL(Point2D(segment_right_x, segment_right_y));
    const Point2LL segment_left(-segment_right.X, segment_right.Y);

    return { SegmentsPattern{ .segments = { segment_up, segment_left, segment_up, segment_right }, .delta_x = -absolute_delta_x },
             SegmentsPattern{ .segments = { segment_up, segment_right, segment_up, segment_left }, .delta_x = absolute_delta_x } };
}

} // namespace cura
