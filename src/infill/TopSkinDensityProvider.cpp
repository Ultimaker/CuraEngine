/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "TopSkinDensityProvider.h"

#include <string>
#include <sstream>

#include "../utils/polygon.h"

#include "../utils/logoutput.h"
#include "../sliceDataStorage.h"

namespace cura {

TopSkinDensityProvider::TopSkinDensityProvider(const SliceMeshStorage& mesh_data)
: print_aabb(mesh_data.bounding_box)
, mesh_data(mesh_data)
{
}

TopSkinDensityProvider::~TopSkinDensityProvider()
{
}

float TopSkinDensityProvider::operator()(const AABB3D& aabb, const int_fast8_t) const
{

    if (aabb.max.z >= mesh_data.bounding_box.max.z - 10) // TODO magic number
    {
        return 1.0;
    }

    size_t first_layer_idx = mesh_data.layers.size();
    size_t last_layer_idx = 0;

    // determine start and end layer
    for (size_t layer_idx = 0; layer_idx < mesh_data.layers.size(); layer_idx++)
    {
        const SliceLayer& layer = mesh_data.layers[layer_idx];
        if (layer.printZ > aabb.min.z)
        {
            first_layer_idx = layer_idx;
            break;
        }
    }
    for (size_t layer_idx = first_layer_idx; layer_idx < mesh_data.layers.size(); layer_idx++)
    {
        const SliceLayer& layer = mesh_data.layers[layer_idx];
        if (layer.printZ >= aabb.max.z) // the layer printed from a height just above the aabb also coveres the top because of the layer thickness
        {
            last_layer_idx = layer_idx;
            break;
        }
    }

    const Point middle = aabb.flatten().getMiddle();

    // check whether the box is inside somewhere
    size_t first_inside_layer_idx = std::numeric_limits<size_t>::max();
    for (size_t layer_idx = first_layer_idx; layer_idx <= last_layer_idx; layer_idx++)
    {
        const SliceLayer& layer = mesh_data.layers[layer_idx];
        Polygons layer_outlines = layer.getOutlines();
        if (layer_outlines.inside(middle))
        {
            first_inside_layer_idx = layer_idx;
            break;
        }
    }
    if (first_inside_layer_idx == std::numeric_limits<size_t>::max())
    {
        return 0.0;
    }

    // check whether the box is outside above where it was last seen to be inside
    for (size_t layer_idx = first_inside_layer_idx + 1; layer_idx <= last_layer_idx; layer_idx++)
    {
        const SliceLayer& layer = mesh_data.layers[layer_idx];
        Polygons layer_outlines = layer.getOutlines();
        if (!layer_outlines.inside(middle))
        {
            return 1.0;
        }
    }
    return 0.0;
}

}; // namespace cura
