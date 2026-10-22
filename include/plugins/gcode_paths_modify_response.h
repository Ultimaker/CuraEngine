// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <vector>

#include "GCodePathConfig.h"
#include "cura/plugins/slots/gcode_paths/v0/modify.pb.h"
#include "pathPlanning/GCodePath.h"
#include "plugins/converter.h"

namespace cura::plugins
{
struct gcode_paths_modify_response : public details::converter<gcode_paths_modify_response, slots::gcode_paths::v0::modify::CallResponse, std::vector<GCodePath>>
{
    [[nodiscard]] static constexpr PrintFeatureType getPrintFeatureType(const v0::PrintFeature feature) noexcept;
    [[nodiscard]] static GCodePathConfig buildConfig(const v0::GCodePath& path);
    [[nodiscard]] static constexpr SpaceFillType getSpaceFillType(const v0::SpaceFillType space_fill_type) noexcept;
    native_value_type operator()(native_value_type& original_value, const value_type& message) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
