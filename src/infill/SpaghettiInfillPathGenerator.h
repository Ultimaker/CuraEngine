/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPAGHETTI_INFILL_PATH_GENERATOR_H
#define INFILL_SPAGHETTI_INFILL_PATH_GENERATOR_H

#include <list>

#include "../utils/intpoint.h"
#include "../utils/polygon.h"
#include "../sliceDataStorage.h"
#include "../LayerPlan.h"

namespace cura {

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
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param mesh_config the line config with which to print a print feature
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_line_distance The distance between the infill lines
     * \param infill_overlap The distance by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     */
    static void processSpaghettiInfill(LayerPlan& gcodeLayer, const SliceMeshStorage* mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int fillAngle);

};

}//namespace cura

#endif//INFILL_SPAGHETTI_INFILL_PATH_GENERATOR_H
