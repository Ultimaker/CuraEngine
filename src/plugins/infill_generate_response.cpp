// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/infill_generate_response.h"

#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"

namespace curaengine::plugins
{
infill_generate_response::native_value_type infill_generate_response::operator()(const infill_generate_response::value_type& message) const
{
    VariableWidthLines toolpaths;
    Shape result_polygons;
    OpenLinesSet result_lines;

    for (auto& tool_path : message.tool_paths().tool_paths())
    {
        ExtrusionLine lines;
        for (auto& msg_junction : tool_path.junctions())
        {
            auto& p = msg_junction.point();
            auto junction = ExtrusionJunction{ p.x(), p.y(), msg_junction.width() };
            lines.emplace_back(junction);
        }

        toolpaths.push_back(lines);
    }

    std::vector<VariableWidthLines> toolpaths_;
    toolpaths_.push_back(toolpaths);

    for (auto& polygon_msg : message.polygons().polygons())
    {
        Shape polygon{};

        Polygon outline{};
        for (auto& path_msg : polygon_msg.outline().path())
        {
            outline.push_back(Point2LL{ path_msg.x(), path_msg.y() });
        }
        polygon.push_back(outline);


        for (auto& hole_msg : polygon_msg.holes())
        {
            Polygon hole{};
            for (auto& path_msg : hole_msg.path())
            {
                hole.emplace_back(path_msg.x(), path_msg.y());
            }
            polygon.push_back(hole);
        }

        result_polygons.push_back(polygon);
    }

    for (auto& polygon : message.poly_lines().paths())
    {
        OpenPolyline poly_line;
        for (auto& p : polygon.path())
        {
            poly_line.emplace_back(p.x(), p.y());
        }
        result_lines.emplace_back(poly_line);
    }

    return { toolpaths_, result_polygons, result_lines };
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
