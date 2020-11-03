//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "utils/logoutput.h"
#include "WallToolPaths.h"

namespace cura
{

InsetOrderOptimizer::InsetOrderOptimizer(const FffGcodeWriter& gcode_writer, const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr) :
    gcode_writer(gcode_writer),
    storage(storage),
    gcode_layer(gcode_layer),
    mesh(mesh),
    extruder_nr(extruder_nr),
    mesh_config(mesh_config),
    part(part),
    layer_nr(layer_nr),
    z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner")),
    added_something(false),
    retraction_region_calculated(false)
{
}

bool InsetOrderOptimizer::optimize()
{
    const bool ignore_inner_insets = !mesh.settings.get<bool>("optimize_wall_printing_order");
    const bool outer_inset_first = mesh.settings.get<bool>("outer_inset_first");

    //Bin the insets in order to print the inset indices together, and to optimize the order of each bin to reduce travels.
    std::set<size_t> bins_with_index_zero_insets;
    BinJunctions insets = variableWidthPathToBinJunctions(part.wall_toolpaths, ignore_inner_insets, outer_inset_first, &bins_with_index_zero_insets);

    //If printing the outer inset first, start with the lowest inset.
    //Otherwise start with the highest inset and iterate backwards.
    size_t start_inset;
    size_t end_inset;
    int direction;
    if(outer_inset_first)
    {
        start_inset = 0;
        end_inset = insets.size();
        direction = 1;
    }
    else
    {
        start_inset = insets.size() - 1;
        end_inset = -1;
        direction = -1;
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
        ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));

        if(bins_with_index_zero_insets.count(inset) > 0) //Print using outer wall config.
        {
            gcode_layer.addWalls(insets[inset], mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, z_seam_config, wall_0_wipe_dist, flow, retract_before_outer_wall);
        }
        else
        {
            gcode_layer.addWalls(insets[inset], mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, z_seam_config, 0, flow, false);
        }
    }
    return added_something;
}

size_t InsetOrderOptimizer::getOuterRegionId(const VariableWidthPaths& toolpaths, size_t& out_number_of_regions)
{
    // Polygons show up here one by one, so there are always only a) the outer lines and b) the lines that are part of the holes.
    // Therefore, the outer-regions' lines will always have the region-id that is larger then all of the other ones.

    // First, build the bounding boxes:
    std::map<size_t, AABB> region_ids_to_bboxes;
    for (const VariableWidthLines& path : toolpaths)
    {
        for (const ExtrusionLine& line : path)
        {
            if (region_ids_to_bboxes.count(line.region_id) == 0)
            {
                region_ids_to_bboxes[line.region_id] = AABB();
            }
            AABB& aabb = region_ids_to_bboxes[line.region_id];
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

    out_number_of_regions = region_ids_to_bboxes.size();
    return outer_region_id;
}

BinJunctions InsetOrderOptimizer::variableWidthPathToBinJunctions(const VariableWidthPaths& toolpaths, const bool& ignore_inner_inset_order, const bool& pack_regions_by_inset, std::set<size_t>* p_bins_with_index_zero_insets)
{
    // Find the largest inset-index:
    size_t max_inset_index = 0;
    for (const VariableWidthLines& path : toolpaths)
    {
        max_inset_index = std::max(path.front().inset_idx, max_inset_index);
    }

    // Find which regions are associated with the outer-outer walls (which region is the one the rest is holes inside of):
    size_t number_of_regions = 0;
    const size_t outer_region_id = getOuterRegionId(toolpaths, number_of_regions);

    // Since we're (optionally!) splitting off in the outer and inner regions, it may need twice as many bins as inset-indices.
    const size_t max_bin = ignore_inner_inset_order ? (number_of_regions * 2) + 2 : (max_inset_index + 1) * 2;
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
            if(ignore_inner_inset_order)
            {
                bin_index = std::min(inset_index, static_cast<size_t>(1)) + 2 * (in_hole_region ? line.region_id : 0);
            }
            else
            {
                if (pack_regions_by_inset)  // <- inset-0 region-A, inset-0 B, inset-1 A, inset-1 B, inset-2 A, inset-2 B, ..., etc.
                {
                    bin_index = (inset_index * 2) + (in_hole_region ? 1 : 0);
                }
                else  // <- inset-0 region-A, inset-1 A, inset-2 A, ..., inset-0 B, inset-1 B, inset-2 B, ..., etc.
                {
                    bin_index = inset_index + (in_hole_region ? (max_inset_index + 1) : 0);
                }
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
