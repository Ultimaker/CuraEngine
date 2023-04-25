// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_TYPES_H
#define CURAENGINE_INCLUDE_PLUGINS_TYPES_H

#include "plugins/pluginproxy.h"
#include "utils/polygon.h"

#include "plugin.pb.h"

#include <variant>

namespace cura::plugins
{
namespace defaults
{
constexpr auto simplify = [](const Polygons& polygons, const size_t max_deviation, const size_t max_angle) -> Polygons { return polygons; };
constexpr auto postprocess = [](const std::string& gcode_word) -> std::string { return gcode_word; };

} // namespace defaults

// Register the plugin types
// TODO: Try to determine the projections from the instantiation of the PluginProxy.
// TODO: Add custom converters for proto calls to CuraEngine types.
using simplify_plugin = PluginProxy<">=1.0.0 <2.0.0 || >3.2.1", plugins::proto::Simplify_args, plugins::proto::Simplify_ret, decltype(defaults::simplify)>;
using postproces_plugin = PluginProxy<">=1.0.0 <2.0.0 || >3.2.1", plugins::proto::Postprocess_args, plugins::proto::Postprocess_ret, decltype(defaults::postprocess)>;

using plugin_t = std::variant<simplify_plugin, postproces_plugin>;

// Register the slots here.
enum class Slot
{
    SIMPLIFY,
    POSTPROCESS
};



} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_TYPES_H
