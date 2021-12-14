//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "utils/logoutput.h"
#include "WallToolPaths.h"

namespace cura
{

InsetOrderOptimizer::InsetOrderOptimizer(const FffGcodeWriter& gcode_writer, const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const VariableWidthPaths& paths, unsigned int layer_nr) :
    gcode_writer(gcode_writer),
    storage(storage),
    gcode_layer(gcode_layer),
    mesh(mesh),
    extruder_nr(extruder_nr),
    mesh_config(mesh_config),
    paths(paths),
    layer_nr(layer_nr),
    z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"), mesh.settings.get<coord_t>("wall_line_width_0") * 2),
    added_something(false),
    retraction_region_calculated(false)
{
}

bool InsetOrderOptimizer::optimize(const WallType& wall_type)
{
    // Settings & configs:
    const GCodePathConfig& skin_or_infill_config = wall_type == WallType::EXTRA_SKIN ? mesh_config.skin_config : mesh_config.infill_config[0];
    const bool do_outer_wall = wall_type == WallType::OUTER_WALL;
    const GCodePathConfig& inset_0_non_bridge_config = do_outer_wall ? mesh_config.inset0_config : skin_or_infill_config;
    const GCodePathConfig& inset_X_non_bridge_config = do_outer_wall ? mesh_config.insetX_config : skin_or_infill_config;
    const GCodePathConfig& inset_0_bridge_config = do_outer_wall ? mesh_config.bridge_inset0_config : skin_or_infill_config;
    const GCodePathConfig& inset_X_bridge_config = do_outer_wall ? mesh_config.bridge_insetX_config : skin_or_infill_config;

    const size_t wall_0_extruder_nr = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr;
    const size_t wall_x_extruder_nr = mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr;
    const size_t top_bottom_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    const size_t infill_extruder_nr = mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr;

    const bool pack_by_inset = !mesh.settings.get<bool>("optimize_wall_printing_order");
    const InsetDirection inset_direction = mesh.settings.get<InsetDirection>("inset_direction");
    const bool center_last = inset_direction == InsetDirection::CENTER_LAST;

    //Bin the insets in order to print the inset indices together, and to optimize the order of each bin to reduce travels.
    std::set<size_t> bins_with_index_zero_insets;
    BinJunctions insets = variableWidthPathToBinJunctions(paths, pack_by_inset, center_last, &bins_with_index_zero_insets);

    size_t start_inset;
    size_t end_inset;
    int direction;
    //If the entire wall is printed with the current extruder, print all of it.
    if((wall_type == WallType::OUTER_WALL && wall_0_extruder_nr == wall_x_extruder_nr && wall_x_extruder_nr == extruder_nr) ||
            (wall_type == WallType::EXTRA_SKIN && extruder_nr == top_bottom_extruder_nr) ||
            (wall_type == WallType::EXTRA_INFILL && extruder_nr == infill_extruder_nr))
    {
        //If printing the outer inset first, start with the lowest inset.
        //Otherwise start with the highest inset and iterate backwards.
        if(inset_direction == InsetDirection::OUTSIDE_IN)
        {
            start_inset = 0;
            end_inset = insets.size();
            direction = 1;
        }
        else //INSIDE_OUT or CENTER_LAST.
        {
            start_inset = insets.size() - 1;
            end_inset = -1;
            direction = -1;
        }
    }
    //If the wall is partially printed with the current extruder, print the correct part.
    else if(wall_type == WallType::OUTER_WALL && wall_0_extruder_nr != wall_x_extruder_nr)
    {
        //If the wall_0 and wall_x extruders are different, then only include the insets that should be printed by the
        //current extruder_nr.
        if(extruder_nr == wall_0_extruder_nr)
        {
            start_inset = 0;
            end_inset = 1; // Ignore inner walls
            direction = 1;
        }
        else if(extruder_nr == wall_x_extruder_nr)
        {
            start_inset = insets.size() - 1;
            end_inset = 0; // Ignore outer wall
            direction = -1;
        }
        else
        {
            return added_something;
        }
    }
    else //The wall is not printed with this extruder, not even in part. Don't print anything then.
    {
        return added_something;
    }


    //Add all of the insets one by one.
    constexpr Ratio flow = 1.0_r;
    const bool retract_before_outer_wall = mesh.settings.get<bool>("travel_retract_before_outer_wall");
    const coord_t wall_0_wipe_dist = mesh.settings.get<coord_t>("wall_0_wipe_dist");
    for(size_t inset = start_inset; inset != end_inset; inset += direction)
    {
        if(insets[inset].empty())
        {
            continue; //Don't switch extruders either, etc.
        }
        added_something = true;
        gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); //Going to print walls, which are always inside.
        ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"), mesh.settings.get<coord_t>("wall_line_width_0") * 2);

        if(bins_with_index_zero_insets.count(inset) > 0) //Print using outer wall config.
        {
            gcode_layer.addWalls(insets[inset], mesh.settings, inset_0_non_bridge_config, inset_0_bridge_config, z_seam_config, wall_0_wipe_dist, flow, retract_before_outer_wall);
        }
        else
        {
            gcode_layer.addWalls(insets[inset], mesh.settings, inset_X_non_bridge_config, inset_X_bridge_config, z_seam_config, 0, flow, false);
        }
    }
    return added_something;
}

size_t InsetOrderOptimizer::getOuterRegionId(const VariableWidthPaths& toolpaths, size_t& out_max_region_id)
{
    // Polygons show up here one by one, so there are always only a) the outer lines and b) the lines that are part of the holes.
    // Therefore, the outer-regions' lines will always have the region-id that is larger then all of the other ones.

    // First, build the bounding boxes:
    std::map<size_t, AABB> region_ids_to_bboxes; //Use a sorted map, ordered by region_id, so that we can find the largest region_id quickly.
    for (const VariableWidthLines& path : toolpaths)
    {
        for (const ExtrusionLine& line : path)
        {
            AABB& aabb = region_ids_to_bboxes[line.region_id]; // Empty AABBs are default initialized when region_ids are encountered for the first time.
            for (const auto& junction : line.junctions)
            {
                aabb.include(junction.p);
            }
        }
    }

    // Then, the largest of these will be the one that's needed for the outer region, the others' all belong to hole regions:
    AABB outer_bbox;
    size_t outer_region_id = 0; // Region-ID 0 is reserved for 'None'.
    for (const auto& region_id_bbox_pair : region_ids_to_bboxes)
    {
        if (region_id_bbox_pair.second.contains(outer_bbox))
        {
            outer_bbox = region_id_bbox_pair.second;
            outer_region_id = region_id_bbox_pair.first;
        }
    }

    // Maximum Region-ID (using the ordering of the map)
    out_max_region_id = region_ids_to_bboxes.empty() ? 0 : region_ids_to_bboxes.rbegin()->first;
    return outer_region_id;
}

BinJunctions InsetOrderOptimizer::variableWidthPathToBinJunctions(const VariableWidthPaths& toolpaths, const bool pack_regions_by_inset, const bool center_last, std::set<size_t>* p_bins_with_index_zero_insets)
{
    // Find the largest inset-index:
    size_t max_inset_index = 0;
    for (const VariableWidthLines& path : toolpaths)
    {
        max_inset_index = std::max(path.front().inset_idx, max_inset_index);
    }

    // Find which regions are associated with the outer-outer walls (which region is the one the rest is holes inside of):
    size_t max_region_id = 0;
    const size_t outer_region_id = getOuterRegionId(toolpaths, max_region_id);

    //Since we're (optionally!) splitting off in the outer and inner regions, it may need twice as many bins as inset-indices.
    //Add two extra bins for the center-paths, if they need to be stored separately. One bin for inner and one for outer walls.
    const size_t max_bin = (pack_regions_by_inset ? (max_region_id * 2) + 2 : (max_inset_index + 1) * 2) + center_last * 2;
    BinJunctions insets(max_bin + 1);
    for (const VariableWidthLines& path : toolpaths)
    {
        if (path.empty()) // Don't bother printing these.
        {
            continue;
        }
        const size_t inset_index = path.front().inset_idx;

        // Convert list of extrusion lines to vectors of extrusion junctions, and add those to the binned insets.
        for (const ExtrusionLine& line : path)
        {
            // Sort into the right bin, ...
            size_t bin_index;
            const bool in_hole_region = line.region_id != outer_region_id && line.region_id != 0;
            if(center_last && line.is_odd)
            {
                bin_index = inset_index > 0;
            }
            else if(pack_regions_by_inset)
            {
                bin_index = std::min(inset_index, static_cast<size_t>(1)) + 2 * (in_hole_region ? line.region_id : 0) + center_last * 2;
            }
            else
            {
                bin_index = inset_index + (in_hole_region ? (max_inset_index + 1) : 0) + center_last * 2;
            }
            insets[bin_index].emplace_back(line.junctions.begin(), line.junctions.end());

            // Collect all bins that have zero-inset indices in them, if needed:
            if (inset_index == 0 && p_bins_with_index_zero_insets != nullptr)
            {
                p_bins_with_index_zero_insets->insert(bin_index);
            }
        }
    }
    return insets;
}

}//namespace cura
