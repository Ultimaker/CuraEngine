// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TopSurface.h"

#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "infill.h"
#include "sliceDataStorage.h"

namespace cura
{

TopSurface::TopSurface()
{
    // Do nothing. Areas stays empty.
}

void TopSurface::setAreasFromMeshAndLayerNumber(SliceMeshStorage& mesh, size_t layer_number)
{
    // The top surface is all parts of the mesh where there's no mesh above it, so find the layer above it first.
    Polygons mesh_above;
    if (layer_number < mesh.layers.size() - 1)
    {
        mesh_above = mesh.layers[layer_number + 1].getOutlines();
    } // If this is the top-most layer, mesh_above stays empty.

    if (mesh.settings.get<bool>("magic_spiralize"))
    {
        // when spiralizing, the model is often solid so it's no good trying to determine if there is air above or not
        // in this situation, just iron the topmost of the bottom layers
        if (layer_number == mesh.settings.get<size_t>("initial_bottom_layers") - 1)
        {
            areas = mesh.layers[layer_number].getOutlines();
        }
    }
    else
    {
        areas = mesh.layers[layer_number].getOutlines().difference(mesh_above);
    }
}

bool TopSurface::ironing(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const GCodePathConfig& line_config, LayerPlan& layer, const FffGcodeWriter& gcode_writer)
    const
{
    if (areas.empty())
    {
        return false; // Nothing to do.
    }
    // Generate the lines to cover the surface.
    const int extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    const EFillMethod pattern = mesh.settings.get<EFillMethod>("ironing_pattern");
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    constexpr bool connect_polygons = false; // midway connections can make the surface less smooth
    const coord_t line_spacing = mesh.settings.get<coord_t>("ironing_line_spacing");
    const coord_t line_width = line_config.getLineWidth();
    const size_t roofing_layer_count = std::min(mesh.settings.get<size_t>("roofing_layer_count"), mesh.settings.get<size_t>("top_layers"));
    const std::vector<AngleDegrees>& top_most_skin_angles = (roofing_layer_count > 0) ? mesh.roofing_angles : mesh.skin_angles;
    assert(top_most_skin_angles.size() > 0);
    const AngleDegrees direction = top_most_skin_angles[layer.getLayerNr() % top_most_skin_angles.size()] + AngleDegrees(90.0); // Always perpendicular to the skin lines.
    constexpr coord_t infill_overlap = 0;
    constexpr int infill_multiplier = 1;
    constexpr coord_t shift = 0;
    const coord_t max_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t max_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    const Ratio ironing_flow = mesh.settings.get<Ratio>("ironing_flow");
    const bool enforce_monotonic_order = mesh.settings.get<bool>("ironing_monotonic");
    constexpr size_t wall_line_count = 0;
    const coord_t small_area_width = 0; // This shouldn't be on for ironing.
    const Point infill_origin = Point();
    const bool skip_line_stitching = enforce_monotonic_order;

    coord_t ironing_inset = -mesh.settings.get<coord_t>("ironing_inset");
    if (pattern == EFillMethod::ZIG_ZAG)
    {
        // Compensate for the outline_offset decrease that takes place when using the infill generator to generate ironing with the zigzag pattern
        const Ratio width_scale = (float)mesh.settings.get<coord_t>("layer_height") / mesh.settings.get<coord_t>("infill_sparse_thickness");
        ironing_inset += width_scale * line_width / 2;
        // Align the edge of the ironing line with the edge of the outer wall
        ironing_inset -= ironing_flow * line_width / 2;
    }
    else if (pattern == EFillMethod::CONCENTRIC)
    {
        // Counteract the outline_offset increase that takes place when using the infill generator to generate ironing with the concentric pattern
        ironing_inset += line_spacing - line_width / 2;
        // Align the edge of the ironing line with the edge of the outer wall
        ironing_inset -= ironing_flow * line_width / 2;
    }
    Polygons ironed_areas = areas.offset(ironing_inset);

    Infill infill_generator(
        pattern,
        zig_zaggify_infill,
        connect_polygons,
        ironed_areas,
        line_width,
        line_spacing,
        infill_overlap,
        infill_multiplier,
        direction,
        layer.z - 10,
        shift,
        max_resolution,
        max_deviation,
        wall_line_count,
        small_area_width,
        infill_origin,
        skip_line_stitching);
    std::vector<VariableWidthLines> ironing_paths;
    Polygons ironing_polygons;
    Polygons ironing_lines;
    infill_generator.generate(ironing_paths, ironing_polygons, ironing_lines, mesh.settings, layer.getLayerNr(), SectionType::IRONING);

    if (ironing_polygons.empty() && ironing_lines.empty() && ironing_paths.empty())
    {
        return false; // Nothing to do.
    }

    layer.mode_skip_agressive_merge = true;

    bool added = false;
    if (! ironing_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        layer.addTravel(ironing_polygons[0][0], force_comb_retract);
        layer.addPolygonsByOptimizer(ironing_polygons, line_config, ZSeamConfig());
        added = true;
    }
    if (! ironing_lines.empty())
    {
        if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
        {
            // Move to a corner of the area that is perpendicular to the ironing lines, to reduce the number of seams.
            const AABB bounding_box(ironed_areas);
            PointMatrix rotate(-direction + 90);
            const Point center = bounding_box.getMiddle();
            const Point far_away = rotate.apply(
                Point(0, vSize(bounding_box.max - center) * 100)); // Some direction very far away in the direction perpendicular to the ironing lines, relative to the centre.
            // Two options to start, both perpendicular to the ironing lines. Which is closer?
            const Point front_side = PolygonUtils::findNearestVert(center + far_away, ironed_areas).p();
            const Point back_side = PolygonUtils::findNearestVert(center - far_away, ironed_areas).p();
            if (vSize2(layer.getLastPlannedPositionOrStartingPosition() - front_side) < vSize2(layer.getLastPlannedPositionOrStartingPosition() - back_side))
            {
                layer.addTravel(front_side);
            }
            else
            {
                layer.addTravel(back_side);
            }
        }

        if (! enforce_monotonic_order)
        {
            layer.addLinesByOptimizer(ironing_lines, line_config, SpaceFillType::PolyLines);
        }
        else
        {
            const coord_t max_adjacent_distance
                = line_spacing * 1.1; // Lines are considered adjacent - meaning they need to be printed in monotonic order - if spaced 1 line apart, with 10% extra play.
            layer.addLinesMonotonic(Polygons(), ironing_lines, line_config, SpaceFillType::PolyLines, AngleRadians(direction), max_adjacent_distance);
        }
        added = true;
    }
    if (! ironing_paths.empty())
    {
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0u;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
        InsetOrderOptimizer wall_orderer(
            gcode_writer,
            storage,
            layer,
            mesh.settings,
            extruder_nr,
            line_config,
            line_config,
            line_config,
            line_config,
            retract_before_outer_wall,
            wipe_dist,
            wipe_dist,
            extruder_nr,
            extruder_nr,
            z_seam_config,
            ironing_paths);
        wall_orderer.addToLayer();
        added = true;
    }

    layer.mode_skip_agressive_merge = false;
    return added;
}

} // namespace cura
