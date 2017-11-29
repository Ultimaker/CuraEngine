/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */

#include <iterator>
#include <algorithm>
#include <math.h>
#include "AdaptiveLayerHeights.h"

namespace cura
{

AdaptiveLayer::AdaptiveLayer(int layer_height)
{
    this->layer_height = layer_height;
}

AdaptiveLayerHeights::AdaptiveLayerHeights(Mesh* mesh, int layer_thickness, int initial_layer_thickness, coord_t variation, coord_t step_size, double threshold)
{
    // store the required parameters
    this->mesh = mesh;
    this->layer_height = layer_thickness;
    this->initial_layer_height = initial_layer_thickness;
    this->max_variation = static_cast<int>(variation);
    this->step_size = static_cast<int>(step_size);
    this->threshold = threshold;

    // calculate the allowed layer heights from variation and step size
    // note: the order is from thickest to thinnest height!
    for (int allowed_layer_height = this->layer_height + this->max_variation; allowed_layer_height >= this->layer_height - this->max_variation; allowed_layer_height -= this->step_size)
    {
        this->allowed_layer_heights.push_back(allowed_layer_height);
    }

    this->calculateMeshTriangleSlopes();
    this->calculateLayers();
}

int AdaptiveLayerHeights::getLayerCount()
{
    return this->layers.size();
}

std::vector<AdaptiveLayer>* AdaptiveLayerHeights::getLayers()
{
    return &this->layers;
}

void AdaptiveLayerHeights::calculateLayers()
{
    const int minimum_layer_height = *std::min_element(this->allowed_layer_heights.begin(), this->allowed_layer_heights.end());
    SlicingTolerance slicing_tolerance = this->mesh->getSettingAsSlicingTolerance("slicing_tolerance");
    std::vector<int> triangles_of_interest;
    int z_level = 0;
    int previous_layer_height = 0;

    // the first layer has it's own independent height set, so we always add that
    z_level += this->initial_layer_height;

    // compensate first layer thickness depending on slicing mode
    if (slicing_tolerance == SlicingTolerance::MIDDLE)
    {
        z_level += this->initial_layer_height / 2;
        this->initial_layer_height += this->initial_layer_height / 2;
    }

    auto * adaptive_layer = new AdaptiveLayer(this->initial_layer_height);
    adaptive_layer->z_position = z_level;
    previous_layer_height = adaptive_layer->layer_height;
    this->layers.push_back(*adaptive_layer);

    // loop while triangles are found
    while (!triangles_of_interest.empty() || this->layers.size() < 2)
    {
        // loop over all allowed layer heights starting with the largest
        for (auto & layer_height : this->allowed_layer_heights)
        {
            int lower_bound = z_level;
            int upper_bound = z_level + layer_height;

            std::vector<int> min_bounds;
            std::vector<int> max_bounds;

            // calculate all intersecting lower bounds
            for (auto it_lower = this->face_min_z_values.begin(); it_lower != this->face_min_z_values.end(); ++it_lower)
            {
                if (*it_lower <= upper_bound)
                {
                    min_bounds.emplace_back(std::distance(this->face_min_z_values.begin(), it_lower));
                }
            }

            // calculate all intersecting upper bounds
            for (auto it_upper = this->face_max_z_values.begin(); it_upper != this->face_max_z_values.end(); ++it_upper)
            {
                if (*it_upper >= lower_bound)
                {
                    max_bounds.emplace_back(std::distance(this->face_max_z_values.begin(), it_upper));
                }
            }

            // use lower and upper bounds to filter on triangles that are interesting for this potential layer
            triangles_of_interest.clear();
            std::set_intersection(min_bounds.begin(), min_bounds.end(), max_bounds.begin(), max_bounds.end(), std::back_inserter(triangles_of_interest));

            // when there not interesting triangles in this potential layer go to the next one
            if (triangles_of_interest.empty())
            {
                break;
            }

            std::vector<double> slopes;

            // find all slopes for interesting triangles
            for (auto & triangle_index : triangles_of_interest)
            {
                double slope = this->face_slopes.at(triangle_index);
                slopes.push_back(slope);
            }

            double minimum_slope = *std::min_element(slopes.begin(), slopes.end());
            double minimum_slope_tan = std::tan(minimum_slope);

            // check if the maximum step size has been exceeded depending on layer height direction
            bool has_exceeded_step_size = false;
            if (previous_layer_height > layer_height && previous_layer_height - layer_height > this->step_size)
            {
                has_exceeded_step_size = true;
            }
            else if (layer_height - previous_layer_height > this->step_size && layer_height > minimum_layer_height)
            {
                continue;
            }

            // we add the layer in the following cases:
            // 1) the layer angle is below the threshold and the layer height difference with the previous layer is the maximum allowed step size
            // 2) the layer height is the smallest it is allowed
            // 3) the layer is a flat surface (we can't divide by 0)
            if (minimum_slope_tan == 0.0
                || (layer_height / minimum_slope_tan) <= this->threshold
                || layer_height == minimum_layer_height
                || has_exceeded_step_size)
            {
                z_level += layer_height;
                auto * adaptive_layer = new AdaptiveLayer(layer_height);
                adaptive_layer->z_position = z_level;
                previous_layer_height = adaptive_layer->layer_height;
                this->layers.push_back(*adaptive_layer);
                break;
            }
        }

        // stop calculating when we're out of triangles (e.g. above the mesh)
        if (triangles_of_interest.empty())
        {
            break;
        }
    }
}

void AdaptiveLayerHeights::calculateMeshTriangleSlopes()
{
    // loop over all mesh faces (triangles) and find their slopes
    for (const auto &face : this->mesh->faces)
    {
        const MeshVertex& v0 = this->mesh->vertices[face.vertex_index[0]];
        const MeshVertex& v1 = this->mesh->vertices[face.vertex_index[1]];
        const MeshVertex& v2 = this->mesh->vertices[face.vertex_index[2]];

        FPoint3 p0 = v0.p;
        FPoint3 p1 = v1.p;
        FPoint3 p2 = v2.p;

        int32_t minZ = p0.z;
        int32_t maxZ = p0.z;

        if (p1.z < minZ) minZ = p1.z;
        if (p2.z < minZ) minZ = p2.z;
        if (p1.z > maxZ) maxZ = p1.z;
        if (p2.z > maxZ) maxZ = p2.z;

        // calculate the angle of this triangle in the z direction
        FPoint3 n = FPoint3(p1 - p0).cross(p2 - p0);
        FPoint3 normal = n.normalized();
        double z_angle = std::acos(std::abs(normal.z));

        // prevent flat surfaces from influencing the algorithm
        if (z_angle == 0)
        {
            z_angle = M_PI;
        }

        this->face_min_z_values.push_back(minZ * 1000);
        this->face_max_z_values.push_back(maxZ * 1000);
        this->face_slopes.push_back(z_angle);
    }
}

}