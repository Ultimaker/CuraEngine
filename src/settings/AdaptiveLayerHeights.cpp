/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */

#include "AdaptiveLayerHeights.h"

namespace cura
{

AdaptiveLayer::AdaptiveLayer(int layer_height)
{
    this->layer_height = layer_height;
}

AdaptiveLayerHeights::AdaptiveLayerHeights(Mesh* mesh, int initial_layer_thickness, int allowed_layer_heights[])
{
    this->mesh = mesh;
    this->calculateMeshTriangleSlopes();
}

int AdaptiveLayerHeights::getLayerCount()
{
    return layers.size();
}

std::vector<AdaptiveLayer>* AdaptiveLayerHeights::getLayers()
{
    return &layers;
}

void AdaptiveLayerHeights::calculateLayers()
{

}

void AdaptiveLayerHeights::calculateMeshTriangleSlopes()
{
    // loop over all mesh faces (triangles) and find their slopes
    for (unsigned int face_index = 0; face_index < this->mesh->faces.size(); face_index++)
    {
        const MeshFace& face = this->mesh->faces[face_index];

        // get each vertex
        const MeshVertex& v0 = this->mesh->vertices[face.vertex_index[0]];
        const MeshVertex& v1 = this->mesh->vertices[face.vertex_index[1]];
        const MeshVertex& v2 = this->mesh->vertices[face.vertex_index[2]];

        FPoint3 p0 = v0.p;
        FPoint3 p1 = v1.p;
        FPoint3 p2 = v2.p;

        FPoint3 n = FPoint3(p1 - p0).cross(p2 - p0);
        FPoint3 normal = n.normalized();
        int z_angle = acos(normal.z);

        this->face_slopes[face_index] = z_angle;
    }
}

}