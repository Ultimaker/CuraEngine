// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace curaengine::utils
{
class AABB;
}

namespace curaengine::types
{
class AngleDegrees;
}

namespace curaengine::geometry
{
class OpenLinesSet;
class Shape;
}

namespace curaengine::infill
{






/*!
 * Base class to generate infills that follow parallel lines but have a pattern along those lines, e.g. gyroid or honeycomb
 */
class AbstractLinesInfill
{
public:
    virtual ~AbstractLinesInfill() = default;

    /*!
     * Generate the infill in the given outline
     * @param[out] result_polylines Output variable to store the resulting polyline segments in.
     * @param[out] result_polygons Output variable to store the resulting polygons in. This should theoretically be empty, but it can happen that connecting+stitching generate *
     * closed loops that are turned into polygons.
     * @param zig_zaggify Whether to connect the polylines at their endpoints, forming one single polyline or at least
     * @param line_distance Distance between adjacent curves. This determines the density of the pattern (when printed at a fixed line width).
     * @param in_outline The outline in which to print the pattern. The input shape, so to say.
     * @param z The Z coordinate of this layer. Different Z coordinates cause the pattern to vary, producing a 3D pattern.
     * @param line_width The line width at which the infill will be printed.
     * @param rotation The rotation angle of the infill to be applied.
     */
    void generateInfill(
        ::curaengine::geometry::OpenLinesSet& result_polylines,
        ::curaengine::geometry::Shape& result_polygons,
        const bool zig_zaggify,
        const ::curaengine::utils::coord_t line_distance,
        const ::curaengine::geometry::Shape& in_outline,
        const ::curaengine::utils::coord_t z,
        const ::curaengine::utils::coord_t line_width,
        const ::curaengine::types::AngleDegrees& rotation) const;

protected:
    AbstractLinesInfill() = default;

    /*!
     * Method to be implemented by child classes to generate the raw parallel lines to be included in the infill. The generated lines should be completely filling the bounding box
     * of the given outline, and are allowed to go outside the model.
     * @param line_distance The distance between lines to generate.
     * @param bounding_box The bounding box in which to print the pattern.
     * @param z The Z coordinate of this layer. Different Z coordinates cause the pattern to vary, producing a 3D pattern.
     * @param line_width The line width at which the infill will be printed.
     * @return A set containing the raw parallel lines to be included. Each line of the set should be a complete line on a column, even if goes out of the model
     *         one or multiple times.
     */
    virtual ::curaengine::geometry::OpenLinesSet generateParallelLines(const ::curaengine::utils::coord_t line_distance, const ::curaengine::utils::AABB& bounding_box, const ::curaengine::utils::coord_t z, const ::curaengine::utils::coord_t line_width) const = 0;

private:
    /*! Helper structure that contain data about the split lines to be joined */
    struct SplitLines
    {
        std::array<::curaengine::geometry::Point2LL, 2> chain; // The start and end positions of the intersected polyline
        size_t column_id; // The ID of the original polyline this one is a part from
    };

    /*!
     * Adds polylines to join the intersected segments together
     * @param split_lines The start and end positions of intersected polylines
     * @param in_outline The outline in which to print the pattern. The input shape, so to say.
     * @return The joining segments to be added
     */
    static ::curaengine::geometry::OpenLinesSet zigZaggify(std::vector<SplitLines>& split_lines, const ::curaengine::geometry::Shape& in_outline);

    /*!
     * Cut the given raw lines so that they fit inside the model outer contour
     * @param raw_lines The raw generates lines that may go outside the model
     * @param zig_zaggify Indicates whether the lines should also be connected together at their tips
     * @param in_outline The outline in which to print the pattern. The input shape, so to say.
     * @return The (unconnected) set of proper infill lines to be printed
     */
    static ::curaengine::geometry::OpenLinesSet fitLines(const ::curaengine::geometry::OpenLinesSet& raw_lines, const bool zig_zaggify, const ::curaengine::geometry::Shape& in_outline);
};

} // namespace curaengine::infill
