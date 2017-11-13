/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */


#ifndef CURAENGINE_CALCULATEADAPTIVELAYERHEIGHTS_H
#define CURAENGINE_CALCULATEADAPTIVELAYERHEIGHTS_H

#include "../mesh.h"

namespace cura {

/**
 * Adaptive layer heights calculates the desired layer height depending on:
 *      * Steepest triangle slope of the mesh
 *      *
 */
class AdaptiveLayerHeights
{
public:

    /*!
     * The mesh to analyze. Uses it's triangles to calculate the adaptive layer heights.
     */
    const Mesh* mesh = nullptr;

    AdaptiveLayerHeights(Mesh* mesh, int initial_layer_thickness, int allowed_layer_heights[]);

private:

    /*!
     * Calculates the layers based on the given mesh and allowed layer heights
     */
    void calculateLayers();

    /*!
     * Calculates the slopes for each triangle in the mesh.
     * These are uses later by calculateLayers to find the steepest triangle in a potential layer.
     */
    void calculateMeshTriangleSlopes();
};

}

#endif //CURAENGINE_CALCULATEADAPTIVELAYERHEIGHTS_H
