//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
#include "SpaghettiInfillPathGenerator.h"
#include "../infill.h"
#include "../FffGcodeWriter.h"

namespace cura {


bool SpaghettiInfillPathGenerator::processSpaghettiInfill(const SliceDataStorage& storage, const FffGcodeWriter& fff_gcode_writer, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, int infill_line_distance, int infill_overlap, int infill_angle, const Point& infill_origin)
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    bool added_something = false;
    const GCodePathConfig& config = mesh_config.infill_config[0]; //Don't use gradual infill, so always take the 0th element.
    const EFillMethod pattern = mesh.getSettingAsFillMethod("infill_pattern");
    const bool zig_zaggify_infill = mesh.getSettingBoolean("zig_zaggify_infill");
    const unsigned int infill_line_width = config.getLineWidth();
    const int64_t infill_shift = 0;
    const int64_t outline_offset = 0;
    const double layer_height_mm = (gcode_layer.getLayerNr() == 0) ? mesh.getSettingInMillimeters("layer_height_0") : mesh.getSettingInMillimeters("layer_height");

    // For each part on this layer which is used to fill that part and parts below:
    for (const std::pair<Polygons, double>& filling_area : part.spaghetti_infill_volumes)
    {
        Polygons infill_lines;
        Polygons infill_polygons;

        const Polygons& area = filling_area.first; // Area of the top within which to move while extruding (might be empty if the spaghetti_inset was too large)
        const double total_volume = filling_area.second * mesh.getSettingAsRatio("spaghetti_flow") + mesh.getSettingInCubicMillimeters("spaghetti_infill_extra_volume"); // volume to be extruded
        if (total_volume <= 0.0)
        {
            continue;
        }

        // generate zigzag print head paths
        Polygons* perimeter_gaps_output = nullptr;
        const bool connected_zigzags = true;
        const bool use_endpieces = false;
        Infill infill_comp(pattern, zig_zaggify_infill, area, outline_offset
            , infill_line_width, infill_line_distance, infill_overlap, infill_angle, gcode_layer.z, infill_shift, infill_origin, perimeter_gaps_output, connected_zigzags, use_endpieces
            , mesh.getSettingBoolean("cross_infill_apply_pockets_alternatingly"), mesh.getSettingInMicrons("cross_infill_pocket_size"));
        // cross_fill_patterns is only generated when spaghetti infill is not used,
        // so we pass nullptr here.
        infill_comp.generate(infill_polygons, infill_lines, nullptr, &mesh);

        // add paths to plan with a higher flow ratio in order to extrude the required amount.
        const coord_t total_length = infill_polygons.polygonLength() + infill_lines.polyLineLength();
        if (total_length > 0)
        { // zigzag path generation actually generated paths
            // calculate the normal volume extruded when using the layer height and line width to calculate extrusion
            const double normal_volume = INT2MM(INT2MM(total_length * infill_line_width)) * layer_height_mm;
            assert(normal_volume > 0.0);
            const float flow_ratio = total_volume / normal_volume;
            assert(flow_ratio / mesh.getSettingAsRatio("spaghetti_flow") >= 0.9);
            assert(!std::isnan(flow_ratio) && !std::isinf(flow_ratio));

            if (!infill_polygons.empty() || !infill_lines.empty())
            {
                added_something = true;
                fff_gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.addPolygonsByOptimizer(infill_polygons, config, nullptr, ZSeamConfig(), 0, false, flow_ratio);
                switch(pattern)
                {
                    case EFillMethod::GRID:
                    case EFillMethod::LINES:
                    case EFillMethod::TRIANGLES:
                    case EFillMethod::CUBIC:
                    case EFillMethod::TETRAHEDRAL:
                    case EFillMethod::QUARTER_CUBIC:
                    case EFillMethod::CUBICSUBDIV:
                        gcode_layer.addLinesByOptimizer(infill_lines, config, SpaceFillType::Lines, mesh.getSettingInMicrons("infill_wipe_dist"), flow_ratio);
                        break;
                    case EFillMethod::CROSS:
                    case EFillMethod::CROSS_3D:
                        if (mesh.getSettingBoolean("zig_zaggify_infill"))
                        {
                            gcode_layer.addLinesByOptimizer(infill_lines, config, SpaceFillType::PolyLines, 0, flow_ratio);
                        }
                        else
                        {
                            gcode_layer.addLinesByOptimizer(infill_lines, config, SpaceFillType::Lines, mesh.getSettingInMicrons("infill_wipe_dist"), flow_ratio);
                        }
                        break;
                    case EFillMethod::ZIG_ZAG:
                        gcode_layer.addLinesByOptimizer(infill_lines, config, SpaceFillType::PolyLines, 0, flow_ratio);
                        break;
                    default:
                        gcode_layer.addLinesByOptimizer(infill_lines, config, SpaceFillType::Lines, 0, flow_ratio);
                        break;
                }
            }
        }
        else
        { // zigzag path generation couldn't generate paths, probably because the area was too small
            // generate small path near the middle of the filling area
            // note that we need a path with positive length because that is currently the only way to insert an extrusion in a layer plan
            constexpr int path_length = 10;
            Point middle = AABB(area).getMiddle();
            if (!area.inside(middle))
            {
                PolygonUtils::ensureInsideOrOutside(area, middle, infill_line_width / 2);
            }
            const double normal_volume = INT2MM(INT2MM(path_length * infill_line_width)) * layer_height_mm;
            const float flow_ratio = total_volume / normal_volume;
            gcode_layer.addTravel(middle);
            gcode_layer.addExtrusionMove(middle + Point(0, path_length), config, SpaceFillType::Lines, flow_ratio);
        }
    }
    return added_something;
}

}//namespace cura
