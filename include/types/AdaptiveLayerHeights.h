// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <vector>

#include "types/AdaptiveLayer.h"
#include "utils/Coord_t.h"

namespace curaengine
{
class MeshGroup;
}

namespace curaengine::types
{


/**
 * Adaptive layer heights calculates the desired layer heights depending mesh.
 */
class AdaptiveLayerHeights
{
public:
    /*!
     * Get the amount of adaptive layers found.
     * @return
     */
    [[nodiscard]] size_t getLayerCount() const;

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
     * \param meshgroup The meshgroup to process.
     */
    AdaptiveLayerHeights(
        const utils::coord_t base_layer_height,
        const utils::coord_t variation,
        const utils::coord_t step_size,
        const utils::coord_t threshold,
        const MeshGroup* meshgroup);

private:
    /*!
     * Stores the found layer heights
     */
    std::vector<AdaptiveLayer> layers_;

    /*!
     * Stores the allowed layer heights in microns.
     */
    std::vector<utils::coord_t> allowed_layer_heights_;

    /**
     * The base layer height.
     */
    utils::coord_t base_layer_height_;

    /**
     * The maximum deviation from the base layer height.
     */
    utils::coord_t max_variation_;

    /**
     * The layer height change per step to try between min and max deviation from the base layer height.
     */
    utils::coord_t step_size_;

    /*!
     * Target topography size. Adaptive layers will try to keep the horizontal
     * distance the same.
     */
    utils::coord_t threshold_;

    /*!
     * Stores the found slopes of each face using the same index.
     */
    std::vector<double> face_slopes_;
    std::vector<int> face_min_z_values_;
    std::vector<int> face_max_z_values_;
    const MeshGroup* meshgroup_;

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

} // namespace curaengine::types
