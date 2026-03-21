// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "slice_data/SupportLayer.h"

#include "settings/Settings.h"
#include "slice_data/SliceMeshStorage.h"


namespace cura
{

void SupportLayer::excludeAreasFromSupportInfillAreas(const Shape& exclude_polygons, const AABB& exclude_polygons_boundary_box)
{
    // record the indexes that need to be removed and do that after
    std::list<size_t> to_remove_part_indices; // LIFO for removing

    unsigned int part_count_to_check = support_infill_parts.size(); // note that support_infill_parts.size() changes during the computation below
    for (size_t part_idx = 0; part_idx < part_count_to_check; ++part_idx)
    {
        SupportInfillPart& support_infill_part = support_infill_parts[part_idx];

        // if the areas don't overlap, do nothing
        if (! exclude_polygons_boundary_box.hit(support_infill_part.outline_boundary_box_))
        {
            continue;
        }

        Shape result_polygons = support_infill_part.outline_.difference(exclude_polygons);

        // if no smaller parts get generated, this mean this part should be removed.
        if (result_polygons.empty())
        {
            to_remove_part_indices.push_back(part_idx);
            continue;
        }

        std::vector<SingleShape> smaller_support_islands = result_polygons.splitIntoParts();

        if (smaller_support_islands.empty())
        { // extra safety guard in case result_polygons consists of too small polygons which are automatically removed in splitIntoParts
            to_remove_part_indices.push_back(part_idx);
            continue;
        }

        // there are one or more smaller parts.
        // we first replace the current part with one of the smaller parts,
        // the rest we add to the support_infill_parts (but after part_count_to_check)
        support_infill_part.outline_ = smaller_support_islands[0];

        for (size_t support_island_idx = 1; support_island_idx < smaller_support_islands.size(); ++support_island_idx)
        {
            const SingleShape& smaller_island = smaller_support_islands[support_island_idx];
            support_infill_parts.emplace_back(smaller_island, support_infill_part.support_line_width_, support_infill_part.inset_count_to_generate_);
        }
    }

    // remove the ones that need to be removed (LIFO)
    while (! to_remove_part_indices.empty())
    {
        const size_t remove_idx = to_remove_part_indices.back();
        to_remove_part_indices.pop_back();
        if (support_infill_parts.empty())
        {
            continue;
        }
        if (remove_idx < support_infill_parts.size() - 1)
        { // move last element to the to-be-removed element so that we can erase the last place in the vector
            support_infill_parts[remove_idx] = std::move(support_infill_parts.back());
        }
        support_infill_parts.pop_back(); // always erase last place in the vector
    }
}

void SupportLayer::fillInfillParts(
    const LayerIndex layer_nr,
    const std::vector<Shape>& support_fill_per_layer,
    const coord_t infill_layer_height,
    const std::vector<std::shared_ptr<SliceMeshStorage>>& meshes,
    const coord_t support_line_width,
    const coord_t wall_line_count,
    const coord_t grow_layer_above /*has default 0*/,
    const bool unionAll /*has default false*/,
    const coord_t custom_line_distance /*has default 0*/)
{
    // Find the model exactly z-distance above the support layer.
    Shape overhang_z_dist_above;
    for (const auto& mesh : meshes)
    {
        const coord_t mesh_z_distance_top = mesh->settings.get<coord_t>("support_top_distance");
        const size_t overhang_layer_nr = layer_nr + (mesh_z_distance_top / infill_layer_height) + 1;
        if (overhang_layer_nr < mesh->overhang_areas.size())
        {
            overhang_z_dist_above.push_back(mesh->overhang_areas[overhang_layer_nr]);
        }
    }
    overhang_z_dist_above = overhang_z_dist_above.unionPolygons();

    // Split the support outline into areas that are directly under the overhang and areas that are not.
    const auto all_support_areas_in_layer
        = { support_fill_per_layer[layer_nr].intersection(overhang_z_dist_above), support_fill_per_layer[layer_nr].difference(overhang_z_dist_above) };
    bool use_fractional_config = true;
    for (auto& support_areas : all_support_areas_in_layer)
    {
        for (const SingleShape& island_outline : support_areas.splitIntoParts(unionAll))
        {
            support_infill_parts.emplace_back(island_outline, support_line_width, use_fractional_config, wall_line_count, custom_line_distance);
        }
        use_fractional_config = false;
    }
}

} // namespace cura
