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
/*!
 * A struct to store support specific settings used during processing by the FFCcodeWrite.
 * It also contains some helper functions with logic to obtain layer specific settings
 */
struct SupportConfig
{
    const bool first_layer; //<! true if it is the first layer
    const bool lower_layers; //<! true if it is the first layer or below (raft)
    const size_t extruder_nr; //<! the support extruder number
    const coord_t line_distance; //<! the line distance between support infill
    const coord_t infill_overlap; //<! the support infill overlap
    const AngleDegrees infill_angle; //<! the support infill angle
    const size_t infill_multiplier; //<! the support infill line multiplier
    const size_t wall_line_count; //<! the number of walls to generate
    const coord_t line_width; //<! the nominal support line width for walls and the actual support infill line width
    const EFillMethod pattern; //<! the support infill pattern used
    const bool zig_zaggify_infill; //<! is this a zig_zaggify infill
    const bool connect_polygons; //<! should the support infill polygons be used
    const bool skip_some_zags; //<! is the support infill allowed to skip some zags
    const size_t zag_skip_count; //<! how much zags can the support infill skip
    const bool connect_zigzags; //<! should the support zigzags be connected
    const coord_t brim_line_count; //<! what is the support brim line count

    /*!
     * Determines from the layer number if this is a first layer
     * \param[in] layer_no the layer number
     * \return returns true if it is the first layer
     */
    static bool FirstLayer(const size_t& layer_no);

    /*!
     * Determines from the layer number if this is a first layer or lower (raft)
     * \param[in] layer_no the layer number
     * \return returns true if it is a lower layer
     */
    static bool LowerLayer(const size_t& layer_no);

   /*!
    * Gets the extruder number from the mesh group settings
    * \param[in] lower_layers is this on a lower layer
    * \param[in] mesh_group_settings the mesh group settings
    * \return the extruder number
    */
    static size_t ExtruderNr(const bool& lower_layers, const Settings& mesh_group_settings);

    /*!
     * Returns the line distance from the infill extruder
     * \param[in] first_layer is this the first layer
     * \param[in] infill_extruder the infill extruder
     * \return the line distance
     */
    static coord_t LineDistance(const bool& first_layer, const ExtruderTrain& infill_extruder);

    /*!
     * Gets the infill angle from the support storage
     * \param[in] lower_layers is this on a lower layer
     * \param[in] layer_no the layer number
     * \param[in] storage support storage
     * \return the infill angle
     */
    static AngleDegrees InfillAngle(const bool& lower_layers, const size_t& layer_no, const SupportStorage& storage);

    /*!
     * Get the line width from the mesh group settings
     * \param[in] first_layer is this the first layer
     * \param[in] mesh_group_settings the mesh group settings
     * \param[in] infill_extruder the infill extruder
     * \return the line width
     */
    static coord_t  LineWidth(const bool& first_layer, const Settings& mesh_group_settings, const ExtruderTrain& infill_extruder);

    /*!
     * Get the infill pattern from the infill extruder
     * \param[in] lower_layers is this on a lower layer
     * \param[in] infill_extruder the infill extruder
     * \return the infill pattern
     */
    static EFillMethod Pattern(const bool& lower_layers, const ExtruderTrain& infill_extruder);

};
}

#endif //SUPPORTCONFIG_H
