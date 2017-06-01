//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TopSurface.h"

namespace cura
{

TopSurface::TopSurface()
{
    //Do nothing. Areas stays empty.
}

TopSurface::TopSurface(SliceMeshStorage& mesh, size_t layer_number, size_t part_number)
{
    //The top surface is all parts of the mesh where there's no mesh above it, so find the layer above it first.
    Polygons mesh_above;
    if (layer_number < mesh.layers.size() - 1)
    {
        mesh_above = mesh.layers[layer_number].parts[part_number].print_outline;
    } //If this is the top-most layer, mesh_above stays empty.

    Polygons mesh_this = mesh.layers[layer_number].getOutlines();
    areas = mesh_this.difference(mesh_above);
}

bool TopSurface::sand(const SettingsBaseVirtual* settings, LayerPlan& layer)
{
    return false; //TODO.
}

}