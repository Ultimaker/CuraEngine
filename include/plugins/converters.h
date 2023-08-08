// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_CONVERTERS_H
#define PLUGINS_CONVERTERS_H

#include "Cura.pb.h"
#include "WallToolPaths.h"
#include "cura/plugins/slots/broadcast/v0/broadcast.grpc.pb.h"
#include "cura/plugins/slots/broadcast/v0/broadcast.pb.h"
#include "cura/plugins/slots/gcode_paths/v0/modify.grpc.pb.h"
#include "cura/plugins/slots/gcode_paths/v0/modify.pb.h"
#include "cura/plugins/slots/handshake/v0/handshake.grpc.pb.h"
#include "cura/plugins/slots/handshake/v0/handshake.pb.h"
#include "cura/plugins/slots/infill/v0/generate.grpc.pb.h"
#include "cura/plugins/slots/infill/v0/generate.pb.h"
#include "cura/plugins/slots/postprocess/v0/modify.grpc.pb.h"
#include "cura/plugins/slots/postprocess/v0/modify.pb.h"
#include "cura/plugins/slots/simplify/v0/modify.grpc.pb.h"
#include "cura/plugins/slots/simplify/v0/modify.pb.h"
#include "plugins/metadata.h"
#include "plugins/types.h"
#include "utils/polygon.h"

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>
#include <spdlog/spdlog.h>

#include <google/protobuf/empty.pb.h>
#include <string>
#include <tuple>

namespace cura::plugins
{


struct empty
{
    using value_type = google::protobuf::Empty; ///< The protobuf message type.
    using native_value_type = std::nullptr_t; ///< The native value type.

    value_type operator()() const
    {
        return {};
    }

    constexpr native_value_type operator()(const value_type&) const
    {
        return nullptr;
    }
};

struct broadcast_settings_request
{
    using value_type = slots::broadcast::v0::BroadcastServiceSettingsRequest; ///< The protobuf message type.
    using native_value_type = cura::proto::Slice; ///< The native value type.

    /**
     * @brief Converts native data for broadcasting to a `proto::BroadcastServiceSettingsRequest` message.
     *
     * @param key The key of the setting to be broadcasted.
     * @param value The value of the setting to be broadcasted.
     * @return The converted `proto::BroadcastServiceSettingsRequest` message.
     */
    value_type operator()(const native_value_type& slice_message) const
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
};


struct handshake_request
{
    using value_type = slots::handshake::v0::CallRequest; ///< The protobuf message type.
    using native_value_type = slot_metadata; ///< The native value type.

    /**
     * @brief Converts native data for handshake to a `proto::HandshakeRequest` message.
     *
     * @param service_name The name of the service.
     * @param version_range The version range of the service.
     * @return The converted `proto::HandshakeRequest` message.
     */

    value_type operator()(const native_value_type& slot_info) const
    {
        value_type message{};
        message.set_slot_id(slot_info.slot_id);
        message.set_version_range(slot_info.version_range.data());
        return message;
    }
};

struct handshake_response
{
    using value_type = slots::handshake::v0::CallResponse; ///< The protobuf message type.
    using native_value_type = plugin_metadata; ///< The native value type.

    /**
     * @brief Converts a `proto::HandshakeResponse` message to native data.
     *
     * @param message The `proto::HandshakeResponse` message.
     * @return The native data.
     */
    native_value_type operator()(const value_type& message, std::string_view peer) const
    {
        return { .slot_version = message.slot_version(),
                 .plugin_name = message.plugin_name(),
                 .plugin_version = message.plugin_version(),
                 .peer = std::string{ peer },
                 .broadcast_subscriptions = std::set<int>(message.broadcast_subscriptions().begin(), message.broadcast_subscriptions().end()) };
    }
};


struct simplify_request
{
    using value_type = slots::simplify::v0::modify::CallRequest; ///< The protobuf message type.
    using native_value_type = Polygons; ///< The native value type.

    /**
     * @brief Converts native data for simplification to a `proto::SimplifyRequest` message.
     *
     * @param polygons The polygons to be simplified.
     * @param max_resolution The maximum resolution for the simplified polygons.
     * @param max_deviation The maximum deviation for the simplified polygons.
     * @param max_area_deviation The maximum area deviation for the simplified polygons.
     * @return The converted `proto::SimplifyRequest` message.
     */
    value_type operator()(const native_value_type& polygons, const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation) const
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
};

/**
 * @brief A converter struct for simplify responses.
 *
 * The `simplify_response` struct provides a conversion function that converts a `proto::SimplifyResponse`
 * message to a native value type.
 */
struct simplify_response
{
    using value_type = slots::simplify::v0::modify::CallResponse; ///< The protobuf message type.
    using native_value_type = Polygons; ///< The native value type.

    /**
     * @brief Converts a `proto::SimplifyResponse` message to a native value type.
     *
     * @param message The `proto::SimplifyResponse` message.
     * @return The converted native value.
     */
    native_value_type operator()(const value_type& message) const
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
};


struct postprocess_request
{
    using value_type = slots::postprocess::v0::modify::CallRequest; ///< The protobuf message type.
    using native_value_type = std::string; ///< The native value type.

    /**
     * @brief Converts a native G-code string to a `proto::PostprocessRequest` message.
     *
     * @param gcode The native G-code string.
     * @return The converted `proto::PostprocessRequest` message.
     */
    value_type operator()(const native_value_type& gcode) const
    {
        value_type message{};
        message.set_gcode_word(gcode);
        return message;
    }
};

struct postprocess_response
{
    using value_type = slots::postprocess::v0::modify::CallResponse;
    using native_value_type = std::string;

    native_value_type operator()(const value_type& message) const
    {
        return message.gcode_word();
    }
};

struct infill_generate_request
{
    using value_type = slots::infill::v0::generate::CallRequest;
    using native_value_type = Polygons;

    value_type operator()(const native_value_type& inner_contour, const std::string& pattern, const Settings& settings) const
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
};

struct infill_generate_response
{
    using value_type = slots::infill::v0::generate::CallResponse;
    using native_value_type = std::tuple<std::vector<VariableWidthLines>, Polygons, Polygons>;

    native_value_type operator()(const value_type& message) const
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
};

struct gcode_paths_modify_request
{
    using value_type = slots::gcode_paths::v0::modify::CallRequest;
    using native_value_type = std::vector<double>;

    value_type operator()(const native_value_type& paths, const std::integral auto extruder_nr, const std::integral auto layer_nr) const
    {
        value_type message{};
        return message;
    }
};

struct gcode_paths_modify_response
{
    using value_type = slots::gcode_paths::v0::modify::CallResponse;
    using native_value_type = std::vector<double>;

    native_value_type operator()(const value_type& message) const
    {
        return {};
    }
};

} // namespace cura::plugins


#endif
