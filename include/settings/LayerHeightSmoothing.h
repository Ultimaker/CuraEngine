// CuraEngine is released under the terms of the AGPLv3 or higher
// 
// This file contains community contributions implementing adaptive layer height algorithms

#ifndef LAYERHEIGHTSMOOTHING_H
#define LAYERHEIGHTSMOOTHING_H

#include <vector>
#include "utils/Coord_t.h"

namespace cura
{

struct LayerHeightSmoothingParams
{
    unsigned int radius = 5;
    bool keep_min = false;
    
    LayerHeightSmoothingParams() = default;
    LayerHeightSmoothingParams(unsigned int r, bool keep_minimum) : radius(r), keep_min(keep_minimum) {}
};

class LayerHeightSmoothing
{
public:
    /*!
     * Smooth a layer height profile using Gaussian blur
     * \param layer_heights Vector of layer heights in microns
     * \param min_layer_height Minimum allowed layer height in microns
     * \param max_layer_height Maximum allowed layer height in microns
     * \param smoothing_params Parameters for smoothing
     * \return Smoothed layer height profile
     */
    static std::vector<coord_t> smooth_layer_heights(
        const std::vector<coord_t>& layer_heights,
        coord_t min_layer_height,
        coord_t max_layer_height,
        const LayerHeightSmoothingParams& smoothing_params);

private:
    /*!
     * Generate Gaussian kernel for blurring
     * \param radius Kernel radius
     * \return Gaussian kernel weights
     */
    static std::vector<double> generateGaussianKernel(unsigned int radius);
    
    /*!
     * Apply Gaussian blur to layer heights
     * \param layer_heights Layer heights to blur
     * \param kernel Gaussian kernel
     * \param min_layer_height Minimum allowed layer height
     * \param max_layer_height Maximum allowed layer height
     * \param keep_min Whether to keep minimum heights unchanged
     * \return Blurred layer heights
     */
    static std::vector<coord_t> applyGaussianBlur(
        const std::vector<coord_t>& layer_heights,
        const std::vector<double>& kernel,
        coord_t min_layer_height,
        coord_t max_layer_height,
        bool keep_min);
};

} // namespace cura

#endif // LAYERHEIGHTSMOOTHING_H