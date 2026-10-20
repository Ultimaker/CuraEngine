// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <vector>

#include "cura/plugins/slots/gcode_paths/v0/modify.pb.h"
#include "pathPlanning/GCodePath.h"
#include "plugins/converter.h"
#include "settings/types/LayerIndex.h"

namespace cura::plugins
{
struct gcode_paths_modify_request : public details::converter<gcode_paths_modify_request, slots::gcode_paths::v0::modify::CallRequest, std::vector<GCodePath>>
{
    [[nodiscard]] static constexpr v0::SpaceFillType getSpaceFillType(const SpaceFillType space_fill_type) noexcept;
    [[nodiscard]] static constexpr v0::PrintFeature getPrintFeature(const PrintFeatureType print_feature_type) noexcept;
    value_type operator()(const native_value_type& gcode, const size_t extruder_nr, const LayerIndex layer_nr) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
