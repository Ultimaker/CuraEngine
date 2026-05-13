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

/*! \brief Contains pieces of GCode that have embedded formulae and have to be resolved at the last moment */
class ResolvedGCodePart : public GCodePart
{
public:
    /*!
     * \brief Constructor
     * \param template_resolver The object to be used to resolve the formulae
     * \param raw_string The unresolved GCode part containing formulae
     * \param context_extruder_nr The contextual extruder number to be used when resolving the formulae
     * \param extra_settings Some extra settings to be applied locally when resolving this GCode part
     */
    explicit ResolvedGCodePart(
        const std::shared_ptr<GcodeTemplateResolver>& template_resolver,
        const std::string& raw_string,
        const ResolvingExtruderContext& context_extruder_nr,
        const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value>& extra_settings);

    /*! \brief Gets the full resolved piece of GCode to be exported */
    std::string str() const override;

private:
    const std::shared_ptr<GcodeTemplateResolver> template_resolver_;
    const std::string raw_string_;
    const ResolvingExtruderContext context_extruder_nr_;
    const std::unordered_map<std::string, CuraFormulaeEngine::eval::Value> extra_settings_;
};

} // namespace cura

#endif
