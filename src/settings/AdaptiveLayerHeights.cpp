/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */

#include <iterator>
#include <algorithm>
#include <cmath>
#include <limits>
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
    this->layers = {};

    this->calculateAllowedLayerHeights();
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

void AdaptiveLayerHeights::calculateAllowedLayerHeights()
{
    // calculate the allowed layer heights from variation and step size
    // note: the order is from thickest to thinnest height!
    for (int allowed_layer_height = this->layer_height + this->max_variation; allowed_layer_height >= this->layer_height - this->max_variation; allowed_layer_height -= this->step_size)
    {
        // we should only consider using layer_heights that are > 0
        if (allowed_layer_height <= 0)
        {
            break;
        }
        this->allowed_layer_heights.push_back(allowed_layer_height);
    }
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

    auto * adaptive_layer = new AdaptiveLayer(this->initial_layer_height);
    adaptive_layer->z_position = z_level;
    previous_layer_height = adaptive_layer->layer_height;
    this->layers.push_back(*adaptive_layer);

    // loop while triangles are found
    while (!triangles_of_interest.empty() || this->layers.size() < 2)
    {
        double global_min_slope = std::numeric_limits<double>::max();
        int layer_height_for_global_min_slope = 0;
        // loop over all allowed layer heights starting with the largest
        bool has_added_layer = false;
        for (auto & layer_height : this->allowed_layer_heights)
        {
            // use lower and upper bounds to filter on triangles that are interesting for this potential layer
            const int lower_bound = z_level;
            // if slicing tolerance "middle" is used, a layer is interpreted as the middle of the upper and lower bounds.
            const int upper_bound = z_level + ((slicing_tolerance == SlicingTolerance::MIDDLE) ? (layer_height / 2) : layer_height);

            if (layer_height == this->allowed_layer_heights[0])
            {
                // this is the max layer thickness, search through all of the triangles in the mesh to find those
                // that intersect with a layer this thick
                triangles_of_interest.clear();

                for (unsigned int i = 0; i < this->face_min_z_values.size(); ++i)
                {
                    if (this->face_min_z_values[i] <= upper_bound && this->face_max_z_values[i] >= lower_bound)
                    {
                        triangles_of_interest.push_back(i);
                    }
                }
            }
            else
            {
                // this is a reduced thickness layer, just search those triangles that intersected with the layer
                // in the previous iteration
                std::vector<int> last_triangles_of_interest = triangles_of_interest;

                triangles_of_interest.clear();

                for (int i : last_triangles_of_interest)
                {
                    if (this->face_min_z_values[i] <= upper_bound)
                    {
                        triangles_of_interest.push_back(i);
                    }
                }
            }

            // when there not interesting triangles in this potential layer go to the next one
            if (triangles_of_interest.empty())
            {
                break;
            }

            // find the minimum slope of all the interesting triangles
            double minimum_slope = std::numeric_limits<double>::max();
            for (auto & triangle_index : triangles_of_interest)
            {
                double slope = this->face_slopes.at(triangle_index);
                if (minimum_slope > slope)
                {
                    minimum_slope = slope;
                }
            }
            double minimum_slope_tan = std::tan(minimum_slope);
            if (global_min_slope > minimum_slope)
            {
                global_min_slope = minimum_slope;
                layer_height_for_global_min_slope = layer_height;
            }

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
                has_added_layer = true;
                break;
            }
        }

        // stop calculating when we're out of triangles (e.g. above the mesh)
        if (triangles_of_interest.empty())
        {
            break;
        }
        // this means we cannot find a layer height that has an angle lower than the threshold.
        // in this case, we use the layer height with the lowest
        if (!has_added_layer)
        {
            z_level += layer_height_for_global_min_slope;
            auto * adaptive_layer = new AdaptiveLayer(layer_height_for_global_min_slope);
            adaptive_layer->z_position = z_level;
            previous_layer_height = adaptive_layer->layer_height;
            this->layers.push_back(*adaptive_layer);
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

        float minZ = p0.z;
        float maxZ = p0.z;

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