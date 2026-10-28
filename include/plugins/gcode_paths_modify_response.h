// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <vector>

#include "GCodePathConfig.h"
#include "cura/plugins/slots/gcode_paths/v0/modify.pb.h"
#include "path_planning/GCodePath.h"
#include "plugins/details/converter.h"

namespace curaengine::plugins
{
struct gcode_paths_modify_response : public details::converter<gcode_paths_modify_response, ::cura::plugins::slots::gcode_paths::v0::modify::CallResponse, std::vector<::curaengine::path_planning::GCodePath>>
{
    [[nodiscard]] static constexpr PrintFeatureType getPrintFeatureType(const ::cura::plugins::v0::PrintFeature feature) noexcept;
    [[nodiscard]] static GCodePathConfig buildConfig(const ::cura::plugins::v0::GCodePath& path);
    [[nodiscard]] static constexpr SpaceFillType getSpaceFillType(const ::cura::plugins::v0::SpaceFillType space_fill_type) noexcept;
    native_value_type operator()(native_value_type& original_value, const value_type& message) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
