// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_REGULARNGONALINFILL_H
#define INFILL_REGULARNGONALINFILL_H

#include "infill/AbstractLinesInfill.h"

namespace cura
{

/*!
 * Infill generator that can handle hexagonal (honeycomb) and octagonal patterns, i.e. regular N-gonal shapes
 */
class RegularNGonalInfill : public AbstractLinesInfill
{
public:
    enum class RegularNGonType
    {
        Hexagon,
        Octagon
    };

    RegularNGonalInfill(RegularNGonType type);

    ~RegularNGonalInfill() override = default;

protected:
    /*!
     * Generate the parallel vertical lines that will all together form a hexagonal or octagonal pattern
     * @param line_distance The distance between lines to generate.
     * @param bounding_box The bounding box in which to print the pattern.
     * @param z The Z coordinate of this layer. Different Z coordinates cause the pattern to vary, producing a 3D pattern.
     * @param line_width The line width at which the infill will be printed.
     * @return The raw parallel lines to be included.
     */
    OpenLinesSet generateParallelLines(const coord_t line_distance, const AABB& bounding_box, const coord_t z, const coord_t line_width) const override;

private:
    struct SegmentsPattern
    {
        std::array<Point2LL, 4> segments;
        coord_t delta_x;
    };

    /*!
     * Generate the raw parallel lines to form a regular N-gonal shape. Each given pattern is composed of 2 sub-patterns, one at the left of the vertical line,
     * and one at the right. Each sub-pattern contains 4 segments that should be something like this:
     *    /  |  \     segment_right / segment_left
     *   |   |   |    segment_up
     *    \  |  /     segment_left / segment_right
     *     | | |      segment_up
     *
     * @param in_outline The outline in which to print the pattern. The input shape, so to say.
     * @param patterns The base segment patterns to be repeatedly used to form the pattern.
     * @param delta_y The Y delta from the center of the polygon to the actual start of the pattern.
     * @param pattern_width The total width of the pattern that will be repeated along the X axis.
     * @param pattern_height The total height of the pattern that will be repeated along the Y axis.
     * @return The raw parallel lines to be included.
     */
    static OpenLinesSet generateRegularNGonalLines(
        const AABB& bounding_box,
        const std::array<SegmentsPattern, 2>& patterns,
        const coord_t delta_y,
        const coord_t pattern_width,
        const coord_t pattern_height);

    /*!
     * Generate the base segments pattern given the base coordinates. @sa generateRegularNGonalLines()
     * @param segment_up_length The length of the up segment
     * @param segment_right_x The X coordinates of the right segment.
     * @param segment_right_y The Y coordinates of the right segment.
     * @param absolute_delta_x The X delta to be applied on each side of the vertical line.
     * @return The segment pattern to be applied repeatedly to generate the N-gonal shape.
     */
    static std::array<SegmentsPattern, 2>
        generatePatterns(const double segment_up_length, const double segment_right_x, const double segment_right_y, const coord_t absolute_delta_x);

private:
    const RegularNGonType type_;
};

} // namespace cura

#endif