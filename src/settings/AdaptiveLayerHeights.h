/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */

#ifndef CURAENGINE_CALCULATEADAPTIVELAYERHEIGHTS_H
#define CURAENGINE_CALCULATEADAPTIVELAYERHEIGHTS_H

#include "../mesh.h"

namespace cura {

class AdaptiveLayer
{
public:

    /*!
     * Height of the layer in microns.
     */
    int layer_height;

    /*!
     * The absolute z position of the layer.
     */
    int z_position;

    /*!
     * Temperature to use for this layer.
     */
    int temperature;

    /*!
     * The print speed for this layer.
     */
    int print_speed;

    explicit AdaptiveLayer(int layer_height);
};

/**
 * Adaptive layer heights calculates the desired layer heights depending mesh.
 */
class AdaptiveLayerHeights
{
public:

    /*!
     * The mesh to analyse. Uses it's triangles to calculate the adaptive layer heights.
     */
    const Mesh* mesh = nullptr;

    /*!
     * Stores the initial layer height.
     */
    int initial_layer_height;

    /*!
     * Stores the found layer heights
     */
    std::vector<AdaptiveLayer> layers;

    /*!
     * Stores the allowed layer heights in microns.
     */
    std::vector<int> allowed_layer_heights;

    /*!
     * Get the amount of adaptive layers found.
     * @return
     */
    int getLayerCount();

    /*!
     * Get the adaptive layers found.
     * @return
     */
    std::vector<AdaptiveLayer>* getLayers();

    AdaptiveLayerHeights(Mesh* mesh, int initial_layer_thickness, std::vector<int> allowed_layer_heights);

private:

    /*!
     * Stores the found slopes of each face using the same index.
     */
    std::vector<double> face_slopes;
    std::vector<int> face_min_z_values;
    std::vector<int> face_max_z_values;

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
