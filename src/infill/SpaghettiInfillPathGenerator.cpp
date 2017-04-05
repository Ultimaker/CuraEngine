/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "SpaghettiInfillPathGenerator.h"
#include "../infill.h"

namespace cura {


void SpaghettiInfillPathGenerator::processSpaghettiInfill(LayerPlan& gcode_layer, const SliceMeshStorage* mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int infill_angle)
{
    const GCodePathConfig& config = mesh_config.infill_config[0];
    const EFillMethod pattern = mesh->getSettingAsFillMethod("infill_pattern");
    const unsigned int infill_line_width = config.getLineWidth();
    const int64_t z = layer_nr * mesh->getSettingInMicrons("layer_height");
    const int64_t infill_shift = 0;
    const int64_t outline_offset = 0;
    const double layer_height_mm = (layer_nr == 0)? mesh->getSettingInMillimeters("layer_height_0") : mesh->getSettingInMillimeters("layer_height");

    // For each part on this layer which is used to fill that part and parts below:
    for (const std::pair<Polygons, double>& filling_area : part.spaghetti_infill_volumes)
    {
        Polygons infill_lines;
        Polygons infill_polygons;

        const Polygons& area = filling_area.first; // Area of the top within which to move while extruding (might be empty if the spaghetti_inset was too large)
        const double total_volume = filling_area.second * mesh->getSettingAsRatio("spaghetti_flow"); // volume to be extruded
        assert(total_volume > 0.0);

        // generate zigzag print head paths
        Polygons* perimeter_gaps_output = nullptr;
        const bool connected_zigzags = true;
        const bool use_endpieces = false;
        Infill infill_comp(pattern, area, outline_offset, infill_line_width, infill_line_distance, infill_overlap, infill_angle, z, infill_shift, perimeter_gaps_output, connected_zigzags, use_endpieces);
        infill_comp.generate(infill_polygons, infill_lines, mesh);

        // add paths to plan with a higher flow ratio in order to extrude the required amount.
        const coord_t total_length = infill_polygons.polygonLength() + infill_lines.polyLineLength();
        if (total_length > 0)
        { // zigzag path generation actually generated paths
            // calculate the normal volume extruded when using the layer height and line width to calculate extrusion
            const double normal_volume = INT2MM(INT2MM(total_length * infill_line_width)) * layer_height_mm;
            assert(normal_volume > 0.0);
            const float flow_ratio = total_volume / normal_volume;
            assert(flow_ratio / mesh->getSettingAsRatio("spaghetti_flow") >= 0.9);
            assert(!isnan(flow_ratio) && !isinf(flow_ratio));

            gcode_layer.addPolygonsByOptimizer(infill_polygons, &config, nullptr, EZSeamType::SHORTEST, Point(0, 0), 0, false, flow_ratio);
            if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::CUBICSUBDIV)
            {
                gcode_layer.addLinesByOptimizer(infill_lines, &config, SpaceFillType::Lines, mesh->getSettingInMicrons("infill_wipe_dist"), flow_ratio);
            }
            else
            {
                gcode_layer.addLinesByOptimizer(infill_lines, &config, (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines, 0, flow_ratio);
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
            gcode_layer.addExtrusionMove(middle + Point(0, path_length), &config, SpaceFillType::Lines, flow_ratio);
        }
    }
}

}//namespace cura
