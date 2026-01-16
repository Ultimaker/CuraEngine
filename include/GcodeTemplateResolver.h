// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODETEMPLATERESOLVER_H
#define GCODETEMPLATERESOLVER_H

#include <optional>
#include <string>
#include <unordered_map>

namespace cura::GcodeTemplateResolver
{

/*!
 * Resolve a raw GCode template that can contains conditional code and complex formulas
 * @param input The raw GCode template text
 * @param context_extruder_nr The default contextual extruder number, which should be the current extruder number when dealing when an extruder start/end GCode, and nullopt when
 *                            dealing with the global machine start/end gcode
 * @param extra_settings Extra settings to be used locally for the resolving, even though they don't exist as actual settings
 * @return The fully resolved GCode template, or the raw template if an error occurred. Also, if the given input is empty, the result will also be fully empty. However, if it
 *         has some content, the function ensures that the result contains and end-of-line at the end so that it can be inserted directly in a GCode without having to insert one.
 */
std::string resolveGCodeTemplate(
    const std::string& input,
    const std::optional<int> context_extruder_nr = std::nullopt,
    const std::unordered_map<std::string, std::string>& extra_settings = {});

} // namespace cura::GcodeTemplateResolver

#endif
