//Copyright (c) 2019 Ultimaker B.V.
#ifndef TOOLPATH_VISUALIZER_H
#define TOOLPATH_VISUALIZER_H

#include "polygon.h"
#include "SVG.h"
#include "ExtrusionSegment.h"

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
    : svg(svg)
    {
    }
    void outline(const Polygons& input);
    void toolpaths(const std::vector<ExtrusionSegment>& all_segments, bool rounded_visualization = true);
    void underfill(const Polygons& underfills);
    void overfill(const Polygons& overfills, const Polygons& double_overfills = Polygons());
    void width_legend(const Polygons& input, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization);
    void widths(const std::vector<ExtrusionSegment>& all_segments, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization, bool exaggerate_widths = false);
private:
    SVG& svg;
};




} // namespace cura
#endif // TOOLPATH_VISUALIZER_H
