//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SpaghettiInfillPathGenerator.h"
#include "../infill.h"
#include "../FffGcodeWriter.h"
#include "../utils/math.h" //For round_divide.

namespace cura {


bool SpaghettiInfillPathGenerator::processSpaghettiInfill(const SliceDataStorage& storage, const FffGcodeWriter& fff_gcode_writer, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part)
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    bool added_something = false;
    const GCodePathConfig& config = mesh_config.infill_config[0];
    const EFillMethod pattern = mesh.settings.get<EFillMethod>("infill_pattern");
    const bool zig_zaggify_infill = mesh.settings.get<bool>("zig_zaggify_infill");
    const coord_t infill_overlap = mesh.settings.get<coord_t>("infill_overlap_mm");
    const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
    const bool connect_polygons = true; //Spaghetti infill should have as few as possible travel moves.
    const unsigned int infill_line_width = config.getLineWidth();
    constexpr int infill_multiplier = 1;
    const int64_t infill_shift = 0;
    constexpr int wall_line_count = 0;
    const int64_t outline_offset = 0;
    const double layer_height_mm = (gcode_layer.getLayerNr() == 0) ? mesh.settings.get<double>("layer_height_0") : mesh.settings.get<double>("layer_height");
    AngleDegrees infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.
    if (mesh.infill_angles.size() > 0)
    {
        const size_t combined_infill_layers = std::max(unsigned(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y + mesh.settings.get<coord_t>("infill_offset_y"));

    // For each part on this layer which is used to fill that part and parts below:
    for (const std::pair<Polygons, double>& filling_area : part.spaghetti_infill_volumes)
    {
        Polygons infill_lines;
        Polygons infill_polygons;

        const Polygons& area = filling_area.first; // Area of the top within which to move while extruding (might be empty if the spaghetti_inset was too large)
        const double total_volume = filling_area.second * mesh.settings.get<Ratio>("spaghetti_flow") + mesh.settings.get<double>("spaghetti_infill_extra_volume"); // volume to be extruded
        if (total_volume <= 0.0)
        {
            continue;
        }

        // generate zigzag print head paths
        Polygons* perimeter_gaps_output = nullptr;
        const bool connected_zigzags = true;
        const bool use_endpieces = false;
        Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, area, outline_offset
            , infill_line_width, infill_line_distance, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z,
            infill_shift, wall_line_count, infill_origin, perimeter_gaps_output, connected_zigzags, use_endpieces
            , mesh.settings.get<coord_t>("cross_infill_pocket_size"));
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
            assert(flow_ratio / mesh.settings.get<Ratio>("spaghetti_flow") >= 0.9);
            assert(!std::isnan(flow_ratio) && !std::isinf(flow_ratio));

            if (!infill_polygons.empty() || !infill_lines.empty())
            {
                added_something = true;
                fff_gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                if (!infill_polygons.empty())
                {
                    constexpr bool force_comb_retract = false;
                    gcode_layer.addTravel(infill_polygons[0][0], force_comb_retract);
                    gcode_layer.addPolygonsByOptimizer(infill_polygons, config, nullptr, ZSeamConfig(), 0, false, flow_ratio);
                }
                const bool is_zigzag = mesh.settings.get<bool>("zig_zaggify_infill") || pattern == EFillMethod::ZIG_ZAG;
                const coord_t wipe_dist = is_zigzag ? 0 : -mesh.settings.get<coord_t>("infill_wipe_dist");
                const SpaceFillType line_type = is_zigzag ? SpaceFillType::Lines : SpaceFillType::PolyLines;
                gcode_layer.addLinesByOptimizer(infill_lines, config, line_type, false, wipe_dist, flow_ratio);
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
