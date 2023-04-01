//Copyright (c) 2022 Ultimaker B.V.

#include "utils/ToolpathVisualizer.h"

#include <sstream>


namespace cura
{

void ToolpathVisualizer::outline(const Polygons& input)
{
    svg.writeAreas(input, SVG::Color::GRAY, SVG::Color::NONE, 2);
    svg.nextLayer();
}

void ToolpathVisualizer::toolpaths(const std::vector<ExtrusionSegment>& all_segments, bool rounded_visualization)
{
    for (float w = .9; w > .25; w = 1.0 - (1.0 - w) * 1.2)
    {
        Polygons polys;
        for (size_t segment_idx = 0; segment_idx < all_segments.size(); segment_idx++)
        {
            ExtrusionSegment s = all_segments[segment_idx];
            s.from.w *= w / .9;
            s.to.w *= w / .9;
            Polygons covered = s.toPolygons(false);
            polys.add(covered);
        }
        int c = 255 - 200 * (w - .25);
        SVG::ColorObject clr(c, c, c);
        polys = polys.execute(ClipperLib::pftNonZero);
        polys = PolygonUtils::connect(polys);
        for (PolygonRef connected : polys)
            svg.writeAreas(connected, clr, SVG::Color::NONE);
        if (!rounded_visualization) break;
    }
    svg.nextLayer();
}


void ToolpathVisualizer::underfill(const Polygons& underfills)
{
    svg.writeAreas(underfills, SVG::ColorObject(0,128,255), SVG::Color::NONE);
    svg.nextLayer();
}
void ToolpathVisualizer::overfill(const Polygons& overfills, const Polygons& double_overfills)
{
    svg.writeAreas(overfills, SVG::ColorObject(255,128,0), SVG::Color::NONE);
    svg.nextLayer();
    svg.writeAreas(double_overfills, SVG::ColorObject(255,100,0), SVG::Color::NONE);
    if (!double_overfills.empty())
    {
        svg.nextLayer();
    }
}

void ToolpathVisualizer::width_legend(const Polygons& input, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization)
{
    auto to_string = [](float v)
    {
        std::ostringstream ss;
        ss << v;
        return ss.str();
    };
    AABB aabb(input);
    ExtrusionJunction legend_btm(Point(aabb.max.X + nozzle_size + max_dev, aabb.max.Y), nozzle_size - max_dev, 0);
    ExtrusionJunction legend_top(Point(aabb.max.X + nozzle_size + max_dev, aabb.min.Y), nozzle_size + max_dev, 0);
    ExtrusionJunction legend_mid((legend_top.p + legend_btm.p) / 2, (legend_top.w + legend_btm.w) / 2, 0);
    legend_btm.p += (legend_mid.p - legend_btm.p) / 4;
    legend_top.p += (legend_mid.p - legend_top.p) / 4;
    ExtrusionSegment legend_segment(legend_btm, legend_top, true, false);
    svg.writeAreas(legend_segment.toPolygons(false), SVG::ColorObject(200,200,200), SVG::Color::NONE); // real outline
    std::vector<ExtrusionSegment> all_segments_plus;
    all_segments_plus.emplace_back(legend_segment); // colored
    
    Point legend_text_offset(nozzle_size, 0);
    svg.writeText(legend_top.p + legend_text_offset, to_string(INT2MM(legend_top.w)));
    svg.writeText(legend_btm.p + legend_text_offset, to_string(INT2MM(legend_btm.w)));
    svg.writeText(legend_mid.p + legend_text_offset, to_string(INT2MM(legend_mid.w)));
    svg.writeLine(legend_top.p, legend_top.p + legend_text_offset);
    svg.writeLine(legend_btm.p, legend_btm.p + legend_text_offset);
    svg.writeLine(legend_mid.p, legend_mid.p + legend_text_offset);
    
    widths(all_segments_plus, nozzle_size, max_dev, min_w, rounded_visualization);
}

void ToolpathVisualizer::widths(const std::vector<ExtrusionSegment>& all_segments, coord_t nozzle_size, coord_t max_dev, coord_t min_w, bool rounded_visualization, bool exaggerate_widths)
{

//     Point3 middle = rounded_visualization? Point3(255,255,255) : Point3(192,192,192);
    Point3 middle(255,255,255);
    Point3 wide(255,0,0);
    Point3 narrow(0,0,255);
    
//     Polygons connecteds = PolygonUtils::connect(area_covered);
//     for (PolygonRef connected : connecteds)
//         svg.writeAreas(connected, SVG::Color::BLACK, SVG::Color::NONE);
    
    for (float w = .9; w > .25; w = 1.0 - (1.0 - w) * 1.2)
    {
        int brightness = rounded_visualization? 255 - 200 * (w - .25) : 192;
        for (size_t segment_idx = 0; segment_idx < all_segments.size(); segment_idx++)
        {
            ExtrusionSegment ss = all_segments[segment_idx];
//             ss.from.w *= w;
//             ss.to.w *= w;
            for (ExtrusionSegment s : ss.discretize(MM2INT(0.1)))
            {
                coord_t avg_w = (s.from.w + s.to.w) / 2;
                Point3 clr;
                float color_ratio = std::min(1.0, double(std::abs(avg_w - nozzle_size)) / max_dev);
                color_ratio = color_ratio * .5 + .5 * sqrt(color_ratio);
                if (avg_w > nozzle_size)
                {
                    clr = wide * color_ratio + middle * (1.0 - color_ratio );
                }
                else
                {
                    clr = narrow * color_ratio + middle * (1.0 - color_ratio );
                }
                clr = clr * brightness / 255;
                    
//                 coord_t clr_max = std::max(clr.x, std::max(clr.y, clr.z));
//                 clr = clr * 255 / clr_max;

//                 clr.y = clr.y * (255 - 92 * clr.dot(green) / green.vSize() / 255) / 255;
                if (exaggerate_widths)
                {
                    s.from.w = std::max(min_w, min_w + (s.from.w - (nozzle_size - max_dev)) * 5 / 4);
                    s.to.w = std::max(min_w, min_w + (s.to.w - (nozzle_size - max_dev)) * 5 / 4);
                }
//                 else
//                 {
//                     s.from.w *= 0.9;
//                     s.to.w *= 0.9;
//                 }
                s.from.w *= w / .9;
                s.to.w *= w / .9;
                Polygons covered = s.toPolygons();
                svg.writeAreas(covered, SVG::ColorObject(clr.x, clr.y, clr.z), SVG::Color::NONE);
            }
        }
        if (!rounded_visualization) break;
        svg.nextLayer();
    }
}

} // namespace cura
