/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SUPPORT_H
#define SUPPORT_H

#include "sliceDataStorage.h"
#include "modelFile/modelFile.h"

namespace cura {

class AreaSupport {
public:
    /*!
     * Joins the layerpart outlines of all meshes and collects the overhang points (small areas).
     */
    static void joinMeshesAndDetectOverhangPoints(
    SliceDataStorage& storage,
    std::vector<Polygons>& joinedLayers,
    std::vector<std::pair<int, std::vector<Polygons>>>& overhang_points, // stores overhang_points along with the layer index at which the overhang point occurs)
    int layer_count,
    int supportMinAreaSqrt,
    int extrusionWidth
                  );
    /*!
     * Adds tower pieces to the current support layer.
     * From below the roof, the towers are added to the normal support layer and handled as normal support area.
     */
    static void handleTowers(
        Polygons& supportLayer_this,
        std::vector<Polygons>& towerRoofs,
        std::vector<std::pair<int, std::vector<Polygons>>>& overhang_points,
        int& overhang_points_pos,
        int layer_idx,
        int towerRoofExpansionDistance,
        int supportTowerDiameter,
        int supportMinAreaSqrt,
        int layer_count,
        int z_layer_distance_tower
    );
    
    /*!
     * Adds struts (towers against a wall) to the current layer.
     */
    static void handleWallStruts(
        Polygons& supportLayer_this,
        int supportMinAreaSqrt,
        int supportTowerDiameter
    );
};
    
/*!
 * Generate support polygons over all layers.
 * Algorithm:
 * From top layer to bottom layer:
 * - find overhang by looking at the difference between two consucutive layers
 * - join with support areas from layer above
 * - subtract current layer
 * - use the result for the next lower support layer (without doing XY-distance and Z bottom distance, so that a single support beam may move around the model a bit => more stability)
 * - perform inset using X/Y-distance and bottom Z distance
 * 
 * for support buildplate only: purge all support not connected to buildplate
 * 
 * This function also handles small overhang areas (creates towers with larger diameter than just the overhang area) and single walls which could otherwise fall over.
 */
void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count);

}//namespace cura

#endif//SUPPORT_H
