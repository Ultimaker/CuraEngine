// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <vector>

#include "cura/plugins/slots/gcode_paths/v0/modify.pb.h"
#include "pathPlanning/GCodePath.h"
#include "plugins/converter.h"
#include "settings/types/LayerIndex.h"

namespace curaengine::plugins
{
struct gcode_paths_modify_request : public details::converter<gcode_paths_modify_request, ::cura::plugins::slots::gcode_paths::v0::modify::CallRequest, std::vector<GCodePath>>
{
    [[nodiscard]] static constexpr ::cura::plugins::v0::SpaceFillType getSpaceFillType(const SpaceFillType space_fill_type) noexcept;
    [[nodiscard]] static constexpr ::cura::plugins::v0::PrintFeature getPrintFeature(const PrintFeatureType print_feature_type) noexcept;
    value_type operator()(const native_value_type& gcode, const size_t extruder_nr, const LayerIndex layer_nr) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
