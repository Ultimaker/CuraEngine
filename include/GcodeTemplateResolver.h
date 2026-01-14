// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODETEMPLATERESOLVER_H
#define GCODETEMPLATERESOLVER_H

#include <optional>
#include <string>

namespace cura::GcodeTemplateResolver
{

std::string resolveGCodeTemplate(const std::string& input, const std::optional<int> context_extruder_nr = std::nullopt);

}

#endif
