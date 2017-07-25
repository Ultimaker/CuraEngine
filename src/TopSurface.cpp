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

TopSurface::TopSurface(SliceMeshStorage& mesh, size_t layer_number)
{
    //The top surface is all parts of the mesh where there's no mesh above it, so find the layer above it first.
    Polygons mesh_above;
    if (layer_number < mesh.layers.size() - 1)
    {
        mesh_above = mesh.layers[layer_number + 1].getOutlines();
    } //If this is the top-most layer, mesh_above stays empty.

    areas = mesh.layers[layer_number].getOutlines().difference(mesh_above);
}

bool TopSurface::ironing(const SliceMeshStorage& mesh, const GCodePathConfig& line_config, LayerPlan& layer)
{
    if (areas.empty())
    {
        return false; //Nothing to do.
    }
    //Generate the lines to cover the surface.
    const EFillMethod pattern = mesh.getSettingAsFillMethod("ironing_pattern");
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const coord_t line_spacing = mesh.getSettingInMicrons("ironing_line_spacing");
    const coord_t outline_offset = -mesh.getSettingInMicrons("ironing_inset");
    const coord_t line_width = line_config.getLineWidth();
    const std::vector<int>& top_most_skin_angles = (mesh.getSettingAsCount("roofing_layer_count") > 0) ? mesh.roofing_angles : mesh.skin_angles;
    assert(top_most_skin_angles.size() > 0);
    const double direction = top_most_skin_angles[layer.getLayerNr() % top_most_skin_angles.size()] + 90.0; //Always perpendicular to the skin lines.
    constexpr coord_t infill_overlap = 0;
    constexpr coord_t shift = 0;
    Infill infill_generator(pattern, zig_zaggify_infill, areas, outline_offset, line_width, line_spacing, infill_overlap, direction, layer.z - 10, shift);
    Polygons ironing_polygons;
    Polygons ironing_lines;
    infill_generator.generate(ironing_polygons, ironing_lines);

    if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
    {
        //Move to a corner of the area that is perpendicular to the ironing lines, to reduce the number of seams.
        const AABB bounding_box(areas);
        PointMatrix rotate(-direction + 90);
        const Point center = bounding_box.getMiddle();
        const Point far_away = rotate.apply(Point(0, vSize(bounding_box.max - center) * 100)); //Some direction very far away in the direction perpendicular to the ironing lines, relative to the centre.
        //Two options to start, both perpendicular to the ironing lines. Which is closer?
        const Point front_side = PolygonUtils::findNearestVert(center + far_away, areas).p();
        const Point back_side = PolygonUtils::findNearestVert(center - far_away, areas).p();
        if (vSize2(layer.getLastPlannedPositionOrStartingPosition() - front_side) < vSize2(layer.getLastPlannedPositionOrStartingPosition() - back_side))
        {
            layer.addTravel(front_side);
        }
        else
        {
            layer.addTravel(back_side);
        }
    }

    //Add the lines as travel moves to the layer plan.
    bool added = false;
    const float ironing_flow = mesh.getSettingAsRatio("ironing_flow");
    if (!ironing_polygons.empty())
    {
        layer.addPolygonsByOptimizer(ironing_polygons, line_config, nullptr, ZSeamConfig(), 0, false, ironing_flow);
        added = true;
    }
    if (!ironing_lines.empty())
    {
        layer.addLinesByOptimizer(ironing_lines, line_config, SpaceFillType::PolyLines, 0, ironing_flow);
        added = true;
    }
    return added;
}

}
