// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher


#include "plugins/converters.h"

#include "GCodePathConfig.h"
#include "WallToolPaths.h"
#include "pathPlanning/GCodePath.h"
#include "pathPlanning/SpeedDerivatives.h"
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
#include "utils/polygon.h"

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

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

handshake_request::value_type handshake_request::operator()(const std::string& name, const std::string& version, const handshake_request::native_value_type& slot_info) const
{
    value_type message{};
    message.set_slot_id(slot_info.slot_id);
    message.set_version(slot_info.version.data());
    message.set_plugin_name(name);
    message.set_plugin_version(version);
    return message;
}

handshake_response::native_value_type handshake_response::operator()(const handshake_response::value_type& message, std::string_view peer) const
{
    return { .slot_version_range = message.slot_version_range(),
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

simplify_response::native_value_type
    simplify_response::operator()([[maybe_unused]] const simplify_response::native_value_type& original_value, const simplify_response::value_type& message) const
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

postprocess_response::native_value_type
    postprocess_response::operator()([[maybe_unused]] const postprocess_response::native_value_type& original_value, const postprocess_response::value_type& message) const
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

[[nodiscard]] constexpr v0::SpaceFillType gcode_paths_modify_request::getSpaceFillType(const cura::SpaceFillType space_fill_type) noexcept
{
    switch (space_fill_type)
    {
    case SpaceFillType::None:
        return v0::SpaceFillType::NONE;
    case SpaceFillType::Polygons:
        return v0::SpaceFillType::POLYGONS;
    case SpaceFillType::PolyLines:
        return v0::SpaceFillType::POLY_LINES;
    case SpaceFillType::Lines:
        return v0::SpaceFillType::LINES;
    default:
        return v0::SpaceFillType::NONE;
    }
}

[[nodiscard]] constexpr v0::PrintFeature gcode_paths_modify_request::getPrintFeature(const cura::PrintFeatureType print_feature_type) noexcept
{
    switch (print_feature_type)
    {
    case PrintFeatureType::NoneType:
        return v0::PrintFeature::NONETYPE;
    case PrintFeatureType::OuterWall:
        return v0::PrintFeature::OUTERWALL;
    case PrintFeatureType::InnerWall:
        return v0::PrintFeature::INNERWALL;
    case PrintFeatureType::Skin:
        return v0::PrintFeature::SKIN;
    case PrintFeatureType::Support:
        return v0::PrintFeature::SUPPORT;
    case PrintFeatureType::SkirtBrim:
        return v0::PrintFeature::SKIRTBRIM;
    case PrintFeatureType::Infill:
        return v0::PrintFeature::INFILL;
    case PrintFeatureType::SupportInfill:
        return v0::PrintFeature::SUPPORTINFILL;
    case PrintFeatureType::MoveCombing:
        return v0::PrintFeature::MOVECOMBING;
    case PrintFeatureType::MoveRetraction:
        return v0::PrintFeature::MOVERETRACTION;
    case PrintFeatureType::SupportInterface:
        return v0::PrintFeature::SUPPORTINTERFACE;
    case PrintFeatureType::PrimeTower:
        return v0::PrintFeature::PRIMETOWER;
    case PrintFeatureType::NumPrintFeatureTypes:
        return v0::PrintFeature::NUMPRINTFEATURETYPES;
    default:
        return v0::PrintFeature::NONETYPE;
    }
}

gcode_paths_modify_request::value_type
    gcode_paths_modify_request::operator()(const gcode_paths_modify_request::native_value_type& paths, const size_t extruder_nr, const LayerIndex layer_nr) const
{
    value_type message{};
    message.set_extruder_nr(static_cast<int64_t>(extruder_nr));
    message.set_layer_nr(layer_nr);

    // Construct the repeated GCodepath message
    auto* gcode_paths = message.mutable_gcode_paths();
    for (const auto& path : paths)
    {
        auto* gcode_path = gcode_paths->Add();
        // Construct the OpenPath from the points in a GCodePath
        for (const auto& point : path.points)
        {
            auto* points = gcode_path->mutable_path()->add_path();
            points->set_x(point.X);
            points->set_y(point.Y);
        }
        gcode_path->set_space_fill_type(getSpaceFillType(path.space_fill_type));
        gcode_path->set_flow(path.flow);
        gcode_path->set_width_factor(path.width_factor);
        gcode_path->set_spiralize(path.spiralize);
        gcode_path->set_speed_factor(path.speed_factor);
        gcode_path->set_speed_back_pressure_factor(path.speed_back_pressure_factor);
        gcode_path->set_retract(path.retract);
        gcode_path->set_unretract_before_last_travel_move(path.unretract_before_last_travel_move);
        gcode_path->set_perform_z_hop(path.perform_z_hop);
        gcode_path->set_perform_prime(path.perform_prime);
        gcode_path->set_skip_agressive_merge_hint(path.skip_agressive_merge_hint);
        gcode_path->set_done(path.done);
        gcode_path->set_fan_speed(path.getFanSpeed());
        gcode_path->set_mesh_name(path.mesh ? path.mesh->mesh_name : "");
        gcode_path->set_feature(getPrintFeature(path.config.type));
        gcode_path->mutable_speed_derivatives()->set_velocity(path.config.getSpeed());
        gcode_path->mutable_speed_derivatives()->set_acceleration(path.config.getAcceleration());
        gcode_path->mutable_speed_derivatives()->set_jerk(path.config.getJerk());
        gcode_path->set_line_width(path.config.getLineWidth());
        gcode_path->set_layer_thickness(path.config.getLayerThickness());
        gcode_path->set_flow_ratio(path.config.getFlowRatio());
        gcode_path->set_is_bridge_path(path.config.isBridgePath());
    }

    return message;
}

[[nodiscard]] constexpr PrintFeatureType gcode_paths_modify_response::getPrintFeatureType(const v0::PrintFeature feature) noexcept
{
    switch (feature)
    {
    case v0::PrintFeature::NONETYPE:
        return PrintFeatureType::NoneType;
    case v0::PrintFeature::OUTERWALL:
        return PrintFeatureType::OuterWall;
    case v0::PrintFeature::INNERWALL:
        return PrintFeatureType::InnerWall;
    case v0::PrintFeature::SKIN:
        return PrintFeatureType::Skin;
    case v0::PrintFeature::SUPPORT:
        return PrintFeatureType::Support;
    case v0::PrintFeature::SKIRTBRIM:
        return PrintFeatureType::SkirtBrim;
    case v0::PrintFeature::INFILL:
        return PrintFeatureType::Infill;
    case v0::PrintFeature::SUPPORTINFILL:
        return PrintFeatureType::SupportInfill;
    case v0::PrintFeature::MOVECOMBING:
        return PrintFeatureType::MoveCombing;
    case v0::PrintFeature::MOVERETRACTION:
        return PrintFeatureType::MoveRetraction;
    case v0::PrintFeature::SUPPORTINTERFACE:
        return PrintFeatureType::SupportInterface;
    case v0::PrintFeature::PRIMETOWER:
        return PrintFeatureType::PrimeTower;
    case v0::PrintFeature::NUMPRINTFEATURETYPES:
        return PrintFeatureType::NumPrintFeatureTypes;
    default:
        return PrintFeatureType::NoneType;
    }
}

[[nodiscard]] constexpr SpaceFillType gcode_paths_modify_response::getSpaceFillType(const v0::SpaceFillType space_fill_type) noexcept
{
    switch (space_fill_type)
    {
    case v0::SpaceFillType::NONE:
        return SpaceFillType::None;
    case v0::SpaceFillType::POLYGONS:
        return SpaceFillType::Polygons;
    case v0::SpaceFillType::POLY_LINES:
        return SpaceFillType::PolyLines;
    case v0::SpaceFillType::LINES:
        return SpaceFillType::Lines;
    default:
        return SpaceFillType::None;
    }
}

[[nodiscard]] GCodePathConfig gcode_paths_modify_response::buildConfig(const v0::GCodePath& path)
{
    return { .type = getPrintFeatureType(path.feature()),
             .line_width = path.line_width(),
             .layer_thickness = path.layer_thickness(),
             .flow = path.flow_ratio(),
             .speed_derivatives
             = SpeedDerivatives{ .speed = path.speed_derivatives().velocity(), .acceleration = path.speed_derivatives().acceleration(), .jerk = path.speed_derivatives().jerk() },
             .is_bridge_path = path.is_bridge_path(),
             .fan_speed = path.fan_speed() };
}

gcode_paths_modify_response::native_value_type
    gcode_paths_modify_response::operator()(gcode_paths_modify_response::native_value_type& original_value, const gcode_paths_modify_response::value_type& message) const
{
    std::vector<GCodePath> paths;
    using map_t = std::unordered_map<std::string, std::shared_ptr<const SliceMeshStorage>>;
    auto meshes = original_value
                | ranges::views::filter(
                      [](const auto& path)
                      {
                          return path.mesh != nullptr;
                      })
                | ranges::views::transform(
                      [](const auto& path) -> map_t::value_type
                      {
                          return { path.mesh->mesh_name, path.mesh };
                      })
                | ranges::to<map_t>;

    for (const auto& gcode_path_msg : message.gcode_paths())
    {
        GCodePath path{
            .config = buildConfig(gcode_path_msg),
            .mesh = gcode_path_msg.mesh_name().empty() ? nullptr : meshes.at(gcode_path_msg.mesh_name()),
            .space_fill_type = getSpaceFillType(gcode_path_msg.space_fill_type()),
            .flow = gcode_path_msg.flow(),
            .width_factor = gcode_path_msg.width_factor(),
            .spiralize = gcode_path_msg.spiralize(),
            .speed_factor = gcode_path_msg.speed_factor(),
            .speed_back_pressure_factor = gcode_path_msg.speed_back_pressure_factor(),
            .retract = gcode_path_msg.retract(),
            .unretract_before_last_travel_move = gcode_path_msg.unretract_before_last_travel_move(),
            .perform_z_hop = gcode_path_msg.perform_z_hop(),
            .perform_prime = gcode_path_msg.perform_prime(),
            .skip_agressive_merge_hint = gcode_path_msg.skip_agressive_merge_hint(),
            .done = gcode_path_msg.done(),
            .fan_speed = gcode_path_msg.fan_speed(),
        };

        path.points = gcode_path_msg.path().path()
                    | ranges::views::transform(
                          [](const auto& point_msg)
                          {
                              return Point{ point_msg.x(), point_msg.y() };
                          })
                    | ranges::to_vector;

        paths.emplace_back(path);
    }

    return paths;
}
} // namespace cura::plugins