// Copyright (c) 2026 UltiMaker
#pragma once

#include "ExtrusionSegment.h"
#include "SVG.h"
#include "geometry/Polygon.h"

namespace curaengine::utils
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

    void outline(const ::curaengine::geometry::Shape& input);
    void toolpaths(const std::vector<ExtrusionSegment>& all_segments, bool rounded_visualization = true);
    void underfill(const ::curaengine::geometry::Shape& underfills);
    void overfill(const ::curaengine::geometry::Shape& overfills, const ::curaengine::geometry::Shape& double_overfills = ::curaengine::geometry::Shape());
    void width_legend(const ::curaengine::geometry::Shape& input, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization);
    void widths(const std::vector<ExtrusionSegment>& all_segments, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization, bool exaggerate_widths = false);

private:
    SVG& svg_;
};

} // namespace curaengine::utils
