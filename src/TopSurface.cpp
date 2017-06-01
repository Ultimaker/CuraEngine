//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill.h"
#include "LayerPlan.h"
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
    if (areas.empty())
    {
        return false; //Nothing to do.
    }
    //Generate the lines to cover the surface.
    EFillMethod pattern = settings->getSettingAsFillMethod("sanding_pattern");
    Infill infill_generator(pattern, areas, 0, 0, 350, 0, 45.0, layer.z - 10, 0);
    Polygons sand_polygons;
    Polygons sand_lines;
    infill_generator.generate(sand_polygons, sand_lines);

    bool added = false;
    //Add the lines as travel moves to the layer plan.
    for (std::vector<Point> polygon : sand_polygons)
    {
        for (Point point : polygon)
        {
            layer.addTravel_simple(point);
            added = true;
        }
    }
    for (std::vector<Point> line : sand_lines)
    {
        for (Point point : line)
        {
            layer.addTravel_simple(point);
            added = true;
        }
    }

    return added;
}

}