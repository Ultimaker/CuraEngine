// Copyright (c) 2026 UltiMaker
#pragma once

#include "types/SVG.h"
#include "types/geometry/ExtrusionSegment.h"
#include "types/geometry/Polygon.h"

namespace curaengine::types
{

/*!
 * Get statistics of the resulting toolpaths
 */
class ToolpathVisualizer
{
public:
    ToolpathVisualizer(SVG& svg)
        : svg_(svg)
    {
    }

    void outline(const types::geometry::Shape& input);
    void toolpaths(const std::vector<geometry::ExtrusionSegment>& all_segments, bool rounded_visualization = true);
    void underfill(const types::geometry::Shape& underfills);
    void overfill(const types::geometry::Shape& overfills, const types::geometry::Shape& double_overfills = types::geometry::Shape());
    void width_legend(const types::geometry::Shape& input, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization);
    void widths(
        const std::vector<geometry::ExtrusionSegment>& all_segments,
        coord_t nozzle_size,
        coord_t max_dev,
        coord_t min_w,
        bool rounded_visualization,
        bool exaggerate_widths = false);

private:
    SVG& svg_;
};

} // namespace curaengine::types
