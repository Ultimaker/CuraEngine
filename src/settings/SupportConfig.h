//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SUPPORTCONFIG_H
#define SUPPORTCONFIG_H

#include "Settings.h"
#include "EnumSettings.h"
#include "types/AngleDegrees.h"
#include "../utils/Coord_t.h"
#include "../ExtruderTrain.h"
#include "../sliceDataStorage.h"

namespace cura
{
struct SupportConfig
{
    const bool first_layer;
    const bool lower_layers;
    const size_t extruder_nr;
    const coord_t line_distance;
    const coord_t infill_overlap;
    const AngleDegrees infill_angle;
    const size_t infill_multiplier;
    const size_t wall_line_count;
    const coord_t line_width;
    const EFillMethod pattern;
    const bool zig_zaggify_infill;
    const bool connect_polygons;
    const bool skip_some_zags;
    const size_t zag_skip_count;
    const bool connect_zigzags;
    const coord_t brim_line_count;

    // Todo document and unittest (maybe inline)
    static size_t ExtruderNr(const bool& lower_layers, const Settings& mesh_group_settings);

    // Todo document and unittest (maybe inline)
    static coord_t LineDistance(const bool& first_layer, const ExtruderTrain& infill_extruder);

    // Todo document function and unittest (maybe inline)
    static AngleDegrees InfillAngle(const bool& lower_layers, const size_t& layer_no, const SupportStorage& storage);

    // Todo document and unittest (maybe inline)
    static coord_t  LineWidth(const bool& first_layer, const Settings& mesh_group_settings, const ExtruderTrain& infill_extruder);

    // Todo document and write unittest (maybe inline)
    static EFillMethod Pattern(const bool& lower_layers, const ExtruderTrain& infill_extruder);

};
}

#endif //SUPPORTCONFIG_H
