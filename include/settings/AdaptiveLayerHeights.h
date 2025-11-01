// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher
// 
// This file contains community contributions implementing adaptive layer height algorithms

#ifndef ADAPTIVELAYERHEIGHTS_H
#define ADAPTIVELAYERHEIGHTS_H

#include "MeshGroup.h"
#include "utils/Coord_t.h"
#include "settings/SlicingAdaptive.h"
#include "settings/LayerHeightSmoothing.h"

namespace cura
{

class AdaptiveLayer
{
public:
    /*!
     * Height of the layer in microns.
     */
    coord_t layer_height_;

    /*!
     * The absolute z position of the layer.
     */
    coord_t z_position_;

    /*!
     * Temperature to use for this layer.
     */
    int temperature_;

    explicit AdaptiveLayer(const coord_t layer_height);
};

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
    AdaptiveLayerHeights(const coord_t base_layer_height, const coord_t variation, const coord_t step_size, const coord_t threshold, const MeshGroup* meshgroup);
    
    /*!
     * \brief Creates a new adaptive layer height calculator using advanced quality-based approach.
     * \param base_layer_height The base layer height to calculate adaptive layers from.
     * \param min_layer_height Minimum allowed layer height.
     * \param max_layer_height Maximum allowed layer height.
     * \param quality_factor Quality factor from 0.0 (best quality) to 1.0 (fastest speed).
     * \param meshgroup The meshgroup to process.
     */
    AdaptiveLayerHeights(const coord_t base_layer_height, const coord_t min_layer_height, const coord_t max_layer_height, double quality_factor, const MeshGroup* meshgroup);

private:
    /*!
     * Stores the found layer heights
     */
    std::vector<AdaptiveLayer> layers_;

    /*!
     * Stores the allowed layer heights in microns.
     */
    std::vector<coord_t> allowed_layer_heights_;

    /**
     * The base layer height.
     */
    coord_t base_layer_height_;

    /**
     * The maximum deviation from the base layer height.
     */
    coord_t max_variation_;

    /**
     * The layer height change per step to try between min and max deviation from the base layer height.
     */
    coord_t step_size_;

    /*!
     * Target topography size. Adaptive layers will try to keep the horizontal
     * distance the same.
     */
    coord_t threshold_;

    /*!
     * Quality factor for advanced adaptive layers (0.0 = best quality, 1.0 = fastest speed)
     */
    double quality_factor_;

    /*!
     * Minimum layer height for advanced adaptive layers
     */
    coord_t min_layer_height_;

    /*!
     * Maximum layer height for advanced adaptive layers
     */
    coord_t max_layer_height_;

    /*!
     * Flag to determine which algorithm to use
     */
    bool use_advanced_algorithm_;

    /*!
     * SlicingAdaptive instance for advanced calculations
     */
    SlicingAdaptive slicing_adaptive_;

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
     * Calculates the layers using advanced quality-based approach
     */
    void calculateLayersAdvanced();

    /*!
     * Calculates the slopes for each triangle in the mesh.
     * These are uses later by calculateLayers to find the steepest triangle in a potential layer.
     */
    void calculateMeshTriangleSlopes();
};

} // namespace cura

#endif // ADAPTIVELAYERHEIGHTS_H
