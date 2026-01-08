// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_ABSTRACTLINESINFILL_H
#define INFILL_ABSTRACTLINESINFILL_H

#include "geometry/OpenLinesSet.h"
#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{
class Shape;

class AbstractLinesInfill
{
public:
    virtual ~AbstractLinesInfill() = default;

    void generateInfill(
        OpenLinesSet& result_polylines,
        Shape& result_polygons,
        const bool zig_zaggify,
        const coord_t line_distance,
        const Shape& in_outline,
        const coord_t z,
        const coord_t line_width) const;

protected:
    AbstractLinesInfill() = default;

    virtual OpenLinesSet generateParallelLines(const coord_t line_distance, const Shape& in_outline, const coord_t z, const coord_t line_width) const = 0;

private:
    static OpenLinesSet zigZaggify(
        const std::array<std::vector<Point2LL>, 2>& chains,
        std::array<std::vector<unsigned>, 2>& connected_to,
        const std::vector<int>& line_numbers,
        const Shape& in_outline);

    static OpenLinesSet fitLines(const OpenLinesSet& raw_lines, const bool zig_zaggify, const Shape& in_outline);
};

} // namespace cura

#endif