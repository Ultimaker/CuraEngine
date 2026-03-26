// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_RESOLVEDGCODEPART_H
#define GCODEEXPORT_RESOLVEDGCODEPART_H

#include <memory>
#include <unordered_map>

#include "gcode_export/GCodePart.h"
#include "gcode_export/ResolvingExtruderContext.h"

namespace CuraFormulaeEngine::eval
{
struct Value;
}

namespace cura
{

class GcodeTemplateResolver;

class ResolvedGCodePart : public GCodePart
{
public:
    explicit ResolvedGCodePart(
        const std::shared_ptr<GcodeTemplateResolver>& template_resolver,
        const std::string& raw_string,
        const ResolvingExtruderContext& context_extruder_nr,
        const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value>& extra_settings);

    std::string str() const override;

private:
    const std::shared_ptr<GcodeTemplateResolver> template_resolver_;
    const std::string raw_string_;
    const ResolvingExtruderContext context_extruder_nr_;
    const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value> extra_settings_;
};

} // namespace cura

#endif
