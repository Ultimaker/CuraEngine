// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_CONVERTERS_H
#define PLUGINS_CONVERTERS_H

#include <string>
#include <tuple>

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>

#include "plugins/types.h"

#include "cura/plugins/slots/postprocess/v0/postprocess.grpc.pb.h"
#include "cura/plugins/slots/postprocess/v0/postprocess.pb.h"
#include "cura/plugins/slots/simplify/v0/simplify.grpc.pb.h"
#include "cura/plugins/slots/simplify/v0/simplify.pb.h"

namespace cura::plugins
{

struct simplify_request
{
    using value_type = slots::simplify::v0::SimplifyServiceModifyRequest; ///< The protobuf message type.
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
    using value_type = slots::simplify::v0::SimplifyServiceModifyResponse; ///< The protobuf message type.
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
    using value_type = slots::postprocess::v0::PostprocessServiceModifyRequest; ///< The protobuf message type.
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
    using value_type = slots::postprocess::v0::PostprocessServiceModifyResponse;
    using native_value_type = std::string;

    native_value_type operator()(const value_type& message) const
    {
        return message.gcode_word();
    }
};

} // namespace cura::plugins


#endif
