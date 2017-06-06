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

bool TopSurface::sand(const SliceMeshStorage& mesh, const GCodePathConfig& line_config, LayerPlan& layer)
{
    if (areas.empty())
    {
        return false; //Nothing to do.
    }
    //Generate the lines to cover the surface.
    const EFillMethod pattern = mesh.getSettingAsFillMethod("sanding_pattern");
    const coord_t line_spacing = mesh.getSettingInMicrons("sanding_line_spacing");
    const coord_t outline_offset = -mesh.getSettingInMicrons("sanding_inset");
    const coord_t line_width = line_config.getLineWidth();
    const double direction = mesh.skin_angles[layer.getLayerNr() % mesh.skin_angles.size()] + 90.0; //Always perpendicular to the skin lines.
    constexpr coord_t infill_overlap = 0;
    constexpr coord_t shift = 0;
    Infill infill_generator(pattern, areas, outline_offset, line_width, line_spacing, infill_overlap, direction, layer.z - 10, shift);
    Polygons sand_polygons;
    Polygons sand_lines;
    infill_generator.generate(sand_polygons, sand_lines);

    //Add the lines as travel moves to the layer plan.
    bool added = false;
    float sanding_flow = mesh.getSettingAsRatio("sanding_flow");
    if (!sand_polygons.empty())
    {
        layer.addPolygonsByOptimizer(sand_polygons, &line_config, nullptr, EZSeamType::SHORTEST, Point(0, 0), 0, false, sanding_flow);
        added = true;
    }
    if (!sand_lines.empty())
    {
        layer.addLinesByOptimizer(sand_lines, &line_config, SpaceFillType::PolyLines, 0, sanding_flow);
        added = true;
    }
    return added;
}

}