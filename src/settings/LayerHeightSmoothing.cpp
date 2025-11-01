// CuraEngine is released under the terms of the AGPLv3 or higher
// 
// This file contains community contributions implementing adaptive layer height algorithms

#include "settings/LayerHeightSmoothing.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace cura
{

std::vector<double> LayerHeightSmoothing::generateGaussianKernel(unsigned int radius)
{
    unsigned int size = 2 * radius + 1;
    std::vector<double> kernel;
    kernel.reserve(size);

    // Calculate sigma based on radius (from OpenCV approach)
    double sigma = 0.3 * (double)(radius - 1) + 0.8;
    double two_sq_sigma = 2.0 * sigma * sigma;
    double inv_root_two_pi_sq_sigma = 1.0 / std::sqrt(std::numbers::pi * two_sq_sigma);

    for (unsigned int i = 0; i < size; ++i)
    {
        double x = (double)i - (double)radius;
        kernel.push_back(inv_root_two_pi_sq_sigma * std::exp(-x * x / two_sq_sigma));
    }

    return kernel;
}

std::vector<coord_t> LayerHeightSmoothing::applyGaussianBlur(
    const std::vector<coord_t>& layer_heights,
    const std::vector<double>& kernel,
    coord_t min_layer_height,
    coord_t max_layer_height,
    bool keep_min)
{
    if (layer_heights.size() < 6)
        return layer_heights; // Not enough data to smooth

    unsigned int radius = kernel.size() / 2;
    int two_radius = 2 * (int)radius;
    std::vector<coord_t> result;
    result.reserve(layer_heights.size());

    // Keep first few layers unchanged
    result.push_back(layer_heights[0]); // First layer unchanged

    double delta_h = max_layer_height - min_layer_height;
    double inv_delta_h = (delta_h != 0.0) ? 1.0 / delta_h : 1.0;

    for (size_t i = 1; i < layer_heights.size(); ++i)
    {
        coord_t original_height = layer_heights[i];
        double height = 0.0;
        double weight_total = 0.0;

        int begin = std::max((int)i - (int)radius, 1);
        int end = std::min((int)i + (int)radius, (int)layer_heights.size() - 1);

        for (int j = begin; j <= end; ++j)
        {
            int kernel_id = radius + (j - (int)i);
            if (kernel_id >= 0 && kernel_id < (int)kernel.size())
            {
                double dh = std::abs(max_layer_height - layer_heights[j]);
                double weight = kernel[kernel_id] * std::sqrt(dh * inv_delta_h);
                height += weight * layer_heights[j];
                weight_total += weight;
            }
        }

        coord_t smoothed_height = (weight_total == 0) ? original_height : coord_t(height / weight_total);
        smoothed_height = std::clamp(smoothed_height, min_layer_height, max_layer_height);

        if (keep_min)
            smoothed_height = std::min(smoothed_height, original_height);

        result.push_back(smoothed_height);
    }

    return result;
}

std::vector<coord_t> LayerHeightSmoothing::smooth_layer_heights(
    const std::vector<coord_t>& layer_heights,
    coord_t min_layer_height,
    coord_t max_layer_height,
    const LayerHeightSmoothingParams& smoothing_params)
{
    if (layer_heights.size() < 2)
        return layer_heights;

    std::vector<double> kernel = generateGaussianKernel(std::max(smoothing_params.radius, 1u));
    
    // Apply multiple rounds of smoothing
    std::vector<coord_t> result = layer_heights;
    for (int round = 0; round < 6; ++round)
    {
        result = applyGaussianBlur(result, kernel, min_layer_height, max_layer_height, smoothing_params.keep_min);
    }

    return result;
}

} // namespace cura