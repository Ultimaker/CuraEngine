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
#include "pathPlanning/GCodePath.h"
#include "pathPlanning/SpeedDerivatives.h"
#include "plugins/metadata.h"
#include "plugins/types.h"
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
#include "utils/polygon.h"

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>
#include <spdlog/spdlog.h>

#include <google/protobuf/empty.pb.h>
#include <string>
#include <tuple>


namespace cura::plugins
{

namespace details
{
template<class T, class Msg, class Native>
struct converter
{
    using derived_type = T; ///< The derived type.
    using value_type = Msg; ///< The protobuf message type.
    using native_value_type = Native; ///< The native value type.
    friend derived_type;

    constexpr auto operator()(auto&&... args) const
    {
        return static_cast<const derived_type*>(this)->operator()(std::forward<decltype(args)>(args)...);
    }
};
} // namespace details

struct empty
{
    using value_type = google::protobuf::Empty; ///< The protobuf message type.
    using native_value_type = std::nullptr_t; ///< The native value type.

    value_type operator()() const;

    constexpr native_value_type operator()(const value_type&) const;
};

struct broadcast_settings_request : public details::converter<broadcast_settings_request, slots::broadcast::v0::BroadcastServiceSettingsRequest, cura::proto::Slice>
{
    value_type operator()(const native_value_type& slice_message) const;
};

struct handshake_request : public details::converter<handshake_request, slots::handshake::v0::CallRequest, slot_metadata>
{
    value_type operator()(const std::string& name, const std::string& version, const native_value_type& slot_info) const;
};

struct handshake_response : public details::converter<handshake_response, slots::handshake::v0::CallResponse, plugin_metadata>
{
    native_value_type operator()(const value_type& message, std::string_view peer) const;
};

struct simplify_request : public details::converter<simplify_request, slots::simplify::v0::modify::CallRequest, Polygons>
{
    value_type operator()(const native_value_type& polygons, const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation) const;
};

struct simplify_response : public details::converter<simplify_response, slots::simplify::v0::modify::CallResponse, Polygons>
{
    native_value_type operator()([[maybe_unused]] const native_value_type& original_value, const value_type& message) const;
};

struct postprocess_request : public details::converter<postprocess_request, slots::postprocess::v0::modify::CallRequest, std::string>
{
    value_type operator()(const native_value_type& gcode) const;
};

struct postprocess_response : public details::converter<postprocess_response, slots::postprocess::v0::modify::CallResponse, std::string>
{
    native_value_type operator()([[maybe_unused]] const native_value_type& original_value, const value_type& message) const;
};

struct infill_generate_request : public details::converter<infill_generate_request, slots::infill::v0::generate::CallRequest, Polygons>
{
    value_type operator()(const native_value_type& inner_contour, const std::string& pattern, const Settings& settings) const;
};

struct infill_generate_response
    : public details::converter<infill_generate_response, slots::infill::v0::generate::CallResponse, std::tuple<std::vector<std::vector<ExtrusionLine>>, Polygons, Polygons>>
{
    native_value_type operator()(const value_type& message) const;
};

struct gcode_paths_modify_request : public details::converter<gcode_paths_modify_request, slots::gcode_paths::v0::modify::CallRequest, std::vector<GCodePath>>
{
    [[nodiscard]] static constexpr v0::SpaceFillType getSpaceFillType(const SpaceFillType space_fill_type) noexcept;
    [[nodiscard]] static constexpr v0::PrintFeature getPrintFeature(const PrintFeatureType print_feature_type) noexcept;
    value_type operator()(const native_value_type& gcode, const size_t extruder_nr, const LayerIndex layer_nr) const;
};

struct gcode_paths_modify_response : public details::converter<gcode_paths_modify_response, slots::gcode_paths::v0::modify::CallResponse, std::vector<GCodePath>>
{
    [[nodiscard]] static constexpr PrintFeatureType getPrintFeatureType(const v0::PrintFeature feature) noexcept;
    [[nodiscard]] static GCodePathConfig buildConfig(const v0::GCodePath& path);
    [[nodiscard]] static constexpr SpaceFillType getSpaceFillType(const v0::SpaceFillType space_fill_type) noexcept;
    native_value_type operator()(native_value_type& original_value, const value_type& message) const;
};

} // namespace cura::plugins


#endif
