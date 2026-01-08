// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/RegularNGonalInfill.h"

#include "geometry/OpenPolyline.h"
#include "geometry/Shape.h"
#include "geometry/conversions/Point2D_Point2LL.h"
#include "utils/AABB.h"
#include "utils/linearAlg2D.h"

namespace cura
{

OpenLinesSet RegularNGonalInfill::generateParallelLines(const coord_t line_distance, const Shape& in_outline, const coord_t z, const coord_t line_width) const
{
    const AABB aabb(in_outline);

    const double circumradius = line_distance / (std::cos(std::numbers::pi / 6));

    const double segment_right_full_x = line_distance;
    const double segment_right_x = segment_right_full_x - line_width;
    const double segment_right_full_y = circumradius * std::sin(std::numbers::pi / 6);
    const double segment_right_y = segment_right_x * std::tan(std::numbers::pi / 6);
    const Point2LL segment_right = toPoint2LL(Point2D(segment_right_x, segment_right_y));

    const Point2LL segment_up = toPoint2LL(Point2D(0, circumradius - 2 * (segment_right_full_y - segment_right_y)));

    const Point2LL segment_left(-segment_right.X, segment_right.Y);

    const std::array segments = { std::array{ segment_up, segment_left, segment_up, segment_right }, std::array{ segment_up, segment_right, segment_up, segment_left } };

    const coord_t pattern_width = line_distance * 2;
    const coord_t start_position_x = (aabb.min_.X / pattern_width) * pattern_width;
    const coord_t pattern_height = segment_up.Y * 2 + segment_left.Y * 2;
    const coord_t start_position_y = (aabb.min_.Y / pattern_height) * pattern_height;
    const Point2LL start_position(start_position_x, start_position_y);

    Point2LL current_position = start_position;
    size_t line_index = 0;
    OpenLinesSet raw_lines;
    while (current_position < aabb.max_.X)
    {
        const std::array<Point2LL, 4>& current_segments = segments[line_index % 2];
        current_position = Point2LL((line_index % 2 == 0 ? -line_width / 2 : line_width / 2) + start_position_x + (line_index / 2) * line_distance * 2, start_position_y);

        OpenPolyline raw_line;
        raw_line.push_back(current_position);
        size_t segment_index = 0;
        while (current_position.Y < aabb.max_.Y)
        {
            current_position += current_segments[segment_index % 4];
            raw_line.push_back(current_position);
            segment_index++;
        }

        raw_lines.push_back(raw_line);
        line_index++;
    }

    return raw_lines;
}

} // namespace cura
