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

AdaptiveLayerHeights::AdaptiveLayerHeights(Mesh* mesh, int initial_layer_thickness, std::vector<int> allowed_layer_heights)
{
    this->mesh = mesh;
    this->calculateMeshTriangleSlopes();
    this->calculateLayers(initial_layer_thickness, allowed_layer_heights);
}

int AdaptiveLayerHeights::getLayerCount()
{
    return layers.size();
}

std::vector<AdaptiveLayer>* AdaptiveLayerHeights::getLayers()
{
    return &layers;
}

void AdaptiveLayerHeights::calculateLayers(int initial_layer_thickness, std::vector<int> allowed_layer_heights)
{
    const int model_height = this->mesh->max().z;
    const int minimum_layer_height = *std::min_element(allowed_layer_heights.begin(), allowed_layer_heights.end());
    const int max_layers = model_height / minimum_layer_height;
    std::vector<int> triangles_of_interest;
    int z_level = 0; // TODO: use initial layer thickness

    // loop over all potential layers
    for (unsigned int layer_index = 0; layer_index < max_layers; layer_index++)
    {
        // loop over all allowed layer heights starting with the largest
        for (auto & layer_height : allowed_layer_heights)
        {
            const int lower_bound = z_level;
            const int upper_bound = z_level + layer_height;

            std::vector<int> min_bounds;
            std::vector<int> max_bounds;

            // calculate all lower bounds
            for (auto it_lower = this->face_min_z_values.begin(); it_lower != this->face_min_z_values.end(); ++it_lower)
            {
                if ((*it_lower >= lower_bound && *it_lower <= upper_bound) || (*it_lower < lower_bound && *it_lower < upper_bound))
                {
                    min_bounds.emplace_back(std::distance(this->face_min_z_values.begin(), it_lower));
                }
            }

            // calculate all upper bounds
            for (auto it_upper = this->face_max_z_values.begin(); it_upper != this->face_max_z_values.end(); ++it_upper)
            {
                if ((*it_upper <= upper_bound && *it_upper >= lower_bound) || (*it_upper > upper_bound && *it_upper > lower_bound))
                {
                    max_bounds.emplace_back(std::distance(this->face_max_z_values.begin(), it_upper));
                }
            }

            // use lower and upper bounds to filter on triangles that are interesting for this potential layer
            triangles_of_interest.clear();
            std::set_intersection(min_bounds.begin(), min_bounds.end(), max_bounds.begin(), max_bounds.end(), std::back_inserter(triangles_of_interest));

            if (triangles_of_interest.empty()) {
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

            // calculate if minimum slope in this potential layer is too steep or not
            // if not, add the layer
            if (minimum_slope_tan == 0.0 || layer_height / minimum_slope_tan <= 100 || layer_height == minimum_layer_height)
            {
                z_level += layer_height;
                auto * adaptive_layer = new AdaptiveLayer(layer_height);
                this->layers.push_back(*adaptive_layer);
                break;
            }
        }

        if (triangles_of_interest.empty()) {
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

        FPoint3 n = FPoint3(p1 - p0).cross(p2 - p0);
        FPoint3 normal = n.normalized();
        double z_angle = std::acos(std::abs(normal.z));

        this->face_min_z_values.push_back(minZ * 1000);
        this->face_max_z_values.push_back(maxZ * 1000);
        this->face_slopes.push_back(z_angle);
    }
}

}