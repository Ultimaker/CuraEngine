// Copyright (c) 2019 Ultimaker B.V.
#ifndef TOOLPATH_VISUALIZER_H
#define TOOLPATH_VISUALIZER_H

#include "ExtrusionSegment.h"
#include "SVG.h"
#include "geometry/Polygon.h"

namespace cura
{
using namespace cura;

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

    void outline(const Shape& input);
    void toolpaths(const std::vector<ExtrusionSegment>& all_segments, bool rounded_visualization = true);
    void underfill(const Shape& underfills);
    void overfill(const Shape& overfills, const Shape& double_overfills = Shape());
    void width_legend(const Shape& input, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization);
    void widths(const std::vector<ExtrusionSegment>& all_segments, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization, bool exaggerate_widths = false);

private:
    SVG& svg_;
};

} // namespace cura
#endif // TOOLPATH_VISUALIZER_H
