//Copyright (C) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ADAPTIVELAYERHEIGHTS_H
#define ADAPTIVELAYERHEIGHTS_H

#include "../utils/Coord_t.h"

namespace cura {

class AdaptiveLayer
{
public:
    /*!
     * Height of the layer in microns.
     */
    coord_t layer_height;

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

    explicit AdaptiveLayer(const coord_t layer_height);
};

/**
 * Adaptive layer heights calculates the desired layer heights depending mesh.
 */
class AdaptiveLayerHeights
{
public:
    /**
     * The base layer height.
     */
    int base_layer_height;

    /**
     * The maximum deviation from the base layer height.
     */
    int max_variation;

    /**
     * The layer height change per step to try between min and max deviation from the base layer height.
     */
    int step_size;

    /*!
     * Threshold to compare the tan of the steepest slope to.
     */
    double threshold;

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

    /*!
     * \brief Creates a new adaptive layer height calculator.
     * \param base_layer_height The base layer height to calculate adaptive layers from.
     * \param variation How much variation is allowed in the layer thickness.
     * \param step_size The maximum difference in layer height between two
     * adjacent layers.
     * \param threshold Threshold to compare the tangent of the steepest slope
     * to.
     */
    AdaptiveLayerHeights(const coord_t base_layer_height, const coord_t variation, const coord_t step_size, const double threshold);

private:

    /*!
     * Stores the found slopes of each face using the same index.
     */
    std::vector<double> face_slopes;
    std::vector<int> face_min_z_values;
    std::vector<int> face_max_z_values;

    /*!
     * Calculate the allowed layer heights depending on variation and step input
     */
    void calculateAllowedLayerHeights();

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

#endif //ADAPTIVELAYERHEIGHTS_H
