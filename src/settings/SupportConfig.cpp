//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SupportConfig.h"

#include "types/Ratio.h"

namespace cura
{

bool SupportConfig::LowerLayer(const size_t& layer_no)
{
    return layer_no <= 0;
}

bool SupportConfig::FirstLayer(const size_t& layer_no)
{
    return layer_no == 0;
}

size_t SupportConfig::ExtruderNr(const bool& lower_layers, const Settings& mesh_group_settings)
{
    if (lower_layers)
        return mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr;

    return mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;
}

coord_t SupportConfig::LineDistance(const bool& first_layer, const ExtruderTrain& infill_extruder)
{
    if (first_layer)
        return infill_extruder.settings.get<coord_t>("support_initial_layer_line_distance");
    return infill_extruder.settings.get<coord_t>("support_line_distance");
}

coord_t SupportConfig::LineWidth(const bool& first_layer, const Settings& mesh_group_settings,
                                 const ExtruderTrain& infill_extruder)
{
    auto line_width = infill_extruder.settings.get<coord_t>("support_line_width");
    if (first_layer && mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
        line_width *= infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");

    return line_width;
}

EFillMethod SupportConfig::Pattern(const bool& lower_layers, const ExtruderTrain& infill_extruder)
{
    auto pattern = infill_extruder.settings.get<EFillMethod>("support_pattern");
    if (lower_layers && (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG))
        pattern = EFillMethod::GRID;
    return pattern;
}

AngleDegrees SupportConfig::InfillAngle(const bool& lower_layers, const size_t& layer_no, const SupportStorage& storage)
{
    if (!lower_layers)
        return storage.support_infill_angles.at(layer_no % storage.support_infill_angles.size());

    // handles negative layer numbers
    const auto divisor = storage.support_infill_angles_layer_0.size();
    const auto index = ((layer_no % divisor) + divisor) % divisor;
    return storage.support_infill_angles_layer_0.at(index); // Todo: check if data type index is good
}
}