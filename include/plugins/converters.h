// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_CONVERTERS_H
#define PLUGINS_CONVERTERS_H

#include "Cura.pb.h"
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

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>
#include <spdlog/spdlog.h>

#include <google/protobuf/empty.pb.h>
#include <string>
#include <tuple>


namespace cura
{
class GCodePath;
class LayerIndex;
class ExtrusionLine;
class Polygons;
class Settings;
} // namespace cura

namespace cura::plugins
{


struct empty
{
    using value_type = google::protobuf::Empty; ///< The protobuf message type.
    using native_value_type = std::nullptr_t; ///< The native value type.

    value_type operator()() const;

    constexpr native_value_type operator()(const value_type&) const;
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
    value_type operator()(const native_value_type& slice_message) const;
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

    value_type operator()(const native_value_type& slot_info) const;
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
    native_value_type operator()(const value_type& message, std::string_view peer) const;
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
    value_type operator()(const native_value_type& polygons, const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation) const;
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
    native_value_type operator()(const value_type& message) const;
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
    value_type operator()(const native_value_type& gcode) const;
};

struct postprocess_response
{
    using value_type = slots::postprocess::v0::modify::CallResponse;
    using native_value_type = std::string;

    native_value_type operator()(const value_type& message) const;
};

struct infill_generate_request
{
    using value_type = slots::infill::v0::generate::CallRequest;
    using native_value_type = Polygons;

    value_type operator()(const native_value_type& inner_contour, const std::string& pattern, const Settings& settings) const;
};

struct infill_generate_response
{
    using value_type = slots::infill::v0::generate::CallResponse;
    using native_value_type = std::tuple<std::vector<std::vector<ExtrusionLine>>, Polygons, Polygons>;

    native_value_type operator()(const value_type& message) const;
};

struct gcode_paths_modify_request
{
    using value_type = slots::gcode_paths::v0::modify::CallRequest;
    using native_value_type = std::vector<GCodePath>;

    value_type operator()(const native_value_type& paths, const size_t extruder_nr, const LayerIndex layer_nr) const;
};

struct gcode_paths_modify_response
{
    using value_type = slots::gcode_paths::v0::modify::CallResponse;
    using native_value_type = std::vector<GCodePath>;

    native_value_type operator()(const value_type& message) const;
};

} // namespace cura::plugins


#endif
