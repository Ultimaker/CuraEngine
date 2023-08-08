// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher


#include "plugins/converters.h"

#include "WallToolPaths.h"
#include "pathPlanning/GCodePath.h"
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
#include "utils/polygon.h"

namespace cura::plugins
{

empty::value_type empty::operator()() const
{
    return {};
}

constexpr empty::native_value_type empty::operator()(const value_type&) const
{
    return nullptr;
}

broadcast_settings_request::value_type broadcast_settings_request::operator()(const broadcast_settings_request::native_value_type& slice_message) const
{
    value_type message{};
    auto* global_settings = message.mutable_global_settings()->mutable_settings();
    for (const auto& setting : slice_message.global_settings().settings())
    {
        global_settings->emplace(setting.name(), setting.value());
    }

    auto* extruders_settings = message.mutable_extruder_settings();
    for (const auto& extruder : slice_message.extruders())
    {
        auto* settings = extruders_settings->Add()->mutable_settings();
        for (const auto& setting : extruder.settings().settings())
        {
            settings->emplace(setting.name(), setting.value());
        }
    }

    auto* object_settings = message.mutable_object_settings();
    for (const auto& object : slice_message.object_lists())
    {
        auto* settings = object_settings->Add()->mutable_settings();
        for (const auto& setting : object.settings())
        {
            settings->emplace(setting.name(), setting.value());
        }
    }

    auto* limit_to_extruder = message.mutable_limit_to_extruder();
    for (const auto& setting_extruder : slice_message.limit_to_extruder())
    {
        limit_to_extruder->emplace(setting_extruder.name(), setting_extruder.extruder());
    }
    return message;
}

handshake_request::value_type handshake_request::operator()(const handshake_request::native_value_type& slot_info) const
{
    value_type message{};
    message.set_slot_id(slot_info.slot_id);
    message.set_version_range(slot_info.version_range.data());
    return message;
}

handshake_response::native_value_type handshake_response::operator()(const handshake_response::value_type& message, std::string_view peer) const
{
    return { .slot_version = message.slot_version(),
             .plugin_name = message.plugin_name(),
             .plugin_version = message.plugin_version(),
             .peer = std::string{ peer },
             .broadcast_subscriptions = std::set<int>(message.broadcast_subscriptions().begin(), message.broadcast_subscriptions().end()) };
}

simplify_request::value_type
    simplify_request::operator()(const simplify_request::native_value_type& polygons, const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation)
        const
{
    value_type message{};
    if (polygons.empty())
    {
        return message;
    }

    auto* msg_polygons = message.mutable_polygons();
    auto* msg_polygon = msg_polygons->add_polygons();
    auto* msg_outline = msg_polygon->mutable_outline();

    for (const auto& point : ranges::front(polygons.paths))
    {
        auto* msg_outline_path = msg_outline->add_path();
        msg_outline_path->set_x(point.X);
        msg_outline_path->set_y(point.Y);
    }

    auto* msg_holes = msg_polygon->mutable_holes();
    for (const auto& polygon : polygons.paths | ranges::views::drop(1))
    {
        auto* msg_hole = msg_holes->Add();
        for (const auto& point : polygon)
        {
            auto* msg_path = msg_hole->add_path();
            msg_path->set_x(point.X);
            msg_path->set_y(point.Y);
        }
    }

    message.set_max_resolution(max_resolution);
    message.set_max_deviation(max_resolution);
    message.set_max_area_deviation(max_resolution);
    return message;
}

simplify_response::native_value_type simplify_response::operator()(const simplify_response::value_type& message) const
{
    native_value_type poly{};
    for (const auto& paths : message.polygons().polygons())
    {
        Polygon o{};
        for (const auto& point : paths.outline().path())
        {
            o.add(Point{ point.x(), point.y() });
        }
        poly.add(o);

        for (const auto& hole : paths.holes())
        {
            Polygon h{};
            for (const auto& point : hole.path())
            {
                h.add(Point{ point.x(), point.y() });
            }
            poly.add(h);
        }
    }
    return poly;
}

postprocess_request::value_type postprocess_request::operator()(const postprocess_request::native_value_type& gcode) const
{
    value_type message{};
    message.set_gcode_word(gcode);
    return message;
}

postprocess_response::native_value_type postprocess_response::operator()(const postprocess_response::value_type& message) const
{
    return message.gcode_word();
}

infill_generate_request::value_type
    infill_generate_request::operator()(const infill_generate_request::native_value_type& inner_contour, const std::string& pattern, const Settings& settings) const
{
    value_type message{};
    message.set_pattern(pattern);
    auto* msg_settings = message.mutable_settings()->mutable_settings();
    for (const auto& [key, value] : settings.getFlattendSettings())
    {
        msg_settings->insert({ key, value });
    }

    if (inner_contour.empty())
    {
        return message;
    }

    auto* msg_polygons = message.mutable_infill_areas();
    auto* msg_polygon = msg_polygons->add_polygons();
    auto* msg_outline = msg_polygon->mutable_outline();

    for (const auto& point : ranges::front(inner_contour.paths))
    {
        auto* msg_outline_path = msg_outline->add_path();
        msg_outline_path->set_x(point.X);
        msg_outline_path->set_y(point.Y);
    }

    auto* msg_holes = msg_polygon->mutable_holes();
    for (const auto& polygon : inner_contour.paths | ranges::views::drop(1))
    {
        auto* msg_hole = msg_holes->Add();
        for (const auto& point : polygon)
        {
            auto* msg_path = msg_hole->add_path();
            msg_path->set_x(point.X);
            msg_path->set_y(point.Y);
        }
    }

    return message;
}

infill_generate_response::native_value_type infill_generate_response::operator()(const infill_generate_response::value_type& message) const
{
    VariableWidthLines toolpaths;
    Polygons result_polygons;
    Polygons result_lines;

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
        Polygons polygon{};

        Polygon outline{};
        for (auto& path_msg : polygon_msg.outline().path())
        {
            outline.add(Point{ path_msg.x(), path_msg.y() });
        }
        polygon.add(outline);


        for (auto& hole_msg : polygon_msg.holes())
        {
            Polygon hole{};
            for (auto& path_msg : hole_msg.path())
            {
                hole.add(Point{ path_msg.x(), path_msg.y() });
            }
            polygon.add(hole);
        }

        result_polygons.add(polygon);
    }

    for (auto& polygon : message.poly_lines().paths())
    {
        Polygon poly_line;
        for (auto& p : polygon.path())
        {
            poly_line.emplace_back(Point{ p.x(), p.y() });
        }
        result_lines.emplace_back(poly_line);
    }

    return { toolpaths_, result_polygons, result_lines };
}


gcode_paths_modify_request::value_type
    gcode_paths_modify_request::operator()(const gcode_paths_modify_request::native_value_type& paths, const size_t extruder_nr, const LayerIndex layer_nr) const
{
    value_type message{};
    message.set_extruder_nr(extruder_nr);
    message.set_layer_nr(layer_nr);

    // Construct the repeated GCodepath message
    auto* gcode_paths = message.mutable_gcode_paths();
    for (const auto& path : paths)
    {
        auto* gcode_path = gcode_paths->Add();

        // Construct the OpenPath from the points in a GCodePath
        auto* points = gcode_path->mutable_path()->add_path();
        for (const auto& point : path.points)
        {
            points->set_x(point.X);
            points->set_y(point.Y);
        }

        // Construct the estimations for the GCodePath
        auto* estimations = gcode_path->mutable_estimates();
        estimations->set_extrude_time(path.estimates.extrude_time);
    }


    return message;
}


gcode_paths_modify_response::native_value_type gcode_paths_modify_response::operator()(const gcode_paths_modify_response::value_type& message) const
{
    return gcode_paths_modify_response::native_value_type();
}
} // namespace cura::plugins