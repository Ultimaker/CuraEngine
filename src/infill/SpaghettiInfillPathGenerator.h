/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPAGHETTI_INFILL_PATH_GENERATOR_H
#define INFILL_SPAGHETTI_INFILL_PATH_GENERATOR_H

#include <list>

#include "../utils/intpoint.h"
#include "../utils/polygon.h"
#include "../sliceDataStorage.h"
#include "../LayerPlan.h"

namespace cura {

class FffGcodeWriter;

/*!
 * Spaghetti infill is a type of infill which fills every so many layers, but extrudes as much filament corresponding to the total unfilled volume under the filling area.
 * 
 * A filling layer is inserted when a a pillar of infill areas is becoming too high, or when the angle between the filling areas is too shallow.
 * 
 * The filling area might be smaller than the actual infill area, so that we fill the pillar from a smaller top area.
 * 
 * Infill pillars can join each other if they are connected on the top. The total volume will then be extruded from the top.
 * 
 * Where the model spits into two from bottom to top, one of the top pieces will be connected to the lower part as one big pillar, while a new pillar will be generated for the other top part.
 * Which part the base will be connected to is arbitrary.
 * 
 */
class SpaghettiInfillPathGenerator
{
public:

    /*!
     * Add spaghetti infill for a given part in a layer plan.
     * 
     * Move over the infill region with a zigzag pattern and
     * extrude as much material as needed for the current part
     * and all parts below which should be filled withthis spaghetti.
     * 
     * \param storage Where to get the secondary brim lines from if we are adding spaghetti on the very first layer and we need to prime first
     * \param fff_gcode_writer The FffGcodeWriter for which the function setExtruder_addPrime needs to be called when on the first layer
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_line_distance The distance between the infill lines
     * \param infill_overlap The distance by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     * \return Whether this function added anything to the layer plan
     */
    static bool processSpaghettiInfill(const SliceDataStorage& storage, const FffGcodeWriter& fff_gcode_writer, LayerPlan& gcode_layer, const SliceMeshStorage* mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int fillAngle);

};

}//namespace cura

#endif//INFILL_SPAGHETTI_INFILL_PATH_GENERATOR_H
