// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_CONVERTERS_H
#define PLUGINS_CONVERTERS_H

#include <memory>
#include <range/v3/view/drop.hpp>

#include "plugins/types.h"

#include "postprocess.grpc.pb.h"
#include "simplify.grpc.pb.h"

namespace cura::plugins
{

/**
 * @brief A converter struct for plugin requests.
 *
 * The `plugin_request` struct provides a conversion function that converts a native slot ID
 * to a `proto::PluginRequest` message.
 */
struct plugin_request
{
    using value_type = proto::PluginRequest;  ///< The protobuf message type.
    using native_value_type = cura::plugins::SlotID; ///< The native value type.

    /**
     * @brief Converts a native slot ID to a `proto::PluginRequest` message.
     *
     * @param slot_id The native slot ID.
     * @return The converted `proto::PluginRequest` message.
     */
    value_type operator()(const native_value_type& slot_id) const
    {
        value_type message{};
        message.set_id(slot_id);
        return message;
    }
};

/**
 * @brief A converter struct for plugin responses.
 *
 * The `plugin_response` struct provides a conversion function that converts a `proto::PluginResponse`
 * message to a native value type.
 */
struct plugin_response
{
    using value_type = proto::PluginResponse; ///< The protobuf message type.
    using native_value_type = std::pair<std::string, std::string>; ///< The native value type.

    /**
     * @brief Converts a `proto::PluginResponse` message to a native value type.
     *
     * @param message The `proto::PluginResponse` message.
     * @return The converted native value.
     */
    native_value_type operator()(const value_type& message) const
    {
        return std::make_pair(message.version(), message.plugin_hash());
    }
};

/**
 * @brief A converter struct for simplify requests.
 *
 * The `simplify_request` struct provides a conversion function that converts native data for
 * simplification (polygons and simplification parameters) to a `proto::SimplifyRequest` message.
 */
struct simplify_request
{
    using value_type = proto::SimplifyRequest;  ///< The protobuf message type.
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
        message.set_max_resolution(max_resolution);
        message.set_max_deviation(max_resolution);
        message.set_max_area_deviation(max_resolution);

        auto* msg_polygons = message.mutable_polygons();
        auto* msg_polygon = msg_polygons->add_polygons();
        auto* msg_outline_path = msg_polygon->mutable_outline()->add_path();

        for (const auto& point : polygons.front())
        {
            msg_outline_path->set_x(point.X);
            msg_outline_path->set_y(point.Y);
        }

        auto* msg_holes = msg_polygon->mutable_holes();
        for (const auto& polygon : polygons.paths  | ranges::views::drop(1))
        {
            auto* msg_path = msg_holes->Add()->add_path();
            for (const auto& point : polygon)
            {
                msg_path->set_x(point.X);
                msg_path->set_y(point.Y);
            }
        }
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
    using value_type = proto::SimplifyResponse; ///< The protobuf message type.
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

/**
 * @brief A converter struct for postprocess requests.
 *
 * The `postprocess_request` struct provides a conversion function that converts a native G-code string
 * to a `proto::PostprocessRequest` message.
 */
struct postprocess_request
{
    using value_type = proto::PostprocessRequest; ///< The protobuf message type.
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
    using value_type = proto::PostprocessResponse;
    using native_value_type = std::string;

    native_value_type operator()(const value_type& message) const
    {
        return message.gcode_word();
    }
};

} // namespace cura::plugins


#endif
