// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcode_export/ResolvedGCodePart.h"

#include <cura-formulae-engine/eval.h>

#include "GcodeTemplateResolver.h"


namespace cura
{

ResolvedGCodePart::ResolvedGCodePart(
    const std::string& raw_string,
    std::optional<int>& context_extruder_nr,
    const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value>& extra_settings)
    : raw_string_(raw_string)
    , context_extruder_nr_(context_extruder_nr)
    , extra_settings_(extra_settings)
{
}

std::string ResolvedGCodePart::str() const
{
    return GcodeTemplateResolver::resolveGCodeTemplate(raw_string_, context_extruder_nr_, extra_settings_);
}

} // namespace cura
