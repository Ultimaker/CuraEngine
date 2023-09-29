// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InterlockingGenerator.h"

#include "Application.h"
#include "Slice.h"
#include "settings/types/LayerIndex.h"
#include "slicer.h"
#include "utils/VoxelUtils.h"
#include "utils/polygonUtils.h"

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/view.hpp>
#include <range/v3/view/zip.hpp>

#include <algorithm> // max

namespace cura
{

// TODO: optimization: only go up to the min layer count + a couple of layers instead of max_layer_count

void InterlockingGenerator::generateInterlockingStructure(std::vector<Slicer*>& volumes)
{
    Settings& global_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const PointMatrix rotation(global_settings.get<AngleDegrees>("interlocking_orientation"));
    const coord_t beam_layer_count = global_settings.get<int>("interlocking_beam_layer_count");
    const int interface_depth = global_settings.get<int>("interlocking_depth");
    const int boundary_avoidance = global_settings.get<int>("interlocking_boundary_avoidance");

    for (size_t mesh_a_idx = 0; mesh_a_idx < volumes.size(); mesh_a_idx++)
    {
        Slicer& mesh_a = *volumes[mesh_a_idx];
        size_t extruder_nr_a = mesh_a.mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr;
        for (size_t mesh_b_idx = mesh_a_idx + 1; mesh_b_idx < volumes.size(); mesh_b_idx++)
        {
            Slicer& mesh_b = *volumes[mesh_b_idx];
            size_t extruder_nr_b = mesh_b.mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr;

            if (! mesh_a.mesh->canInterlock() || ! mesh_b.mesh->canInterlock())
            {
                continue;
            }

            if (extruder_nr_a == extruder_nr_b || ! mesh_a.mesh->getAABB().expand(ignored_gap).hit(mesh_b.mesh->getAABB()))
            {
                // early out for when meshes don't share any overlap in their bounding box
                continue;
            }

            coord_t beam_width_a = mesh_a.mesh->settings.get<coord_t>("interlocking_beam_width");
            coord_t beam_width_b = mesh_b.mesh->settings.get<coord_t>("interlocking_beam_width");

            // TODO: why are these two kernels different kernel types?!
            const DilationKernel interface_dilation(GridPoint3(interface_depth, interface_depth, interface_depth), DilationKernel::Type::PRISM);

            const bool air_filtering = boundary_avoidance > 0;
            const DilationKernel air_dilation(GridPoint3(boundary_avoidance, boundary_avoidance, boundary_avoidance), DilationKernel::Type::PRISM);

            const coord_t cell_width = beam_width_a + beam_width_b;
            const Point3 cell_size(cell_width, cell_width, 2 * beam_layer_count);

            InterlockingGenerator gen(mesh_a, mesh_b, beam_width_a, beam_width_b, rotation, cell_size, beam_layer_count, interface_dilation, air_dilation, air_filtering);
            gen.generateInterlockingStructure();
        }
    }
}

std::pair<Polygons, Polygons> InterlockingGenerator::growBorderAreasPerpendicular(const Polygons& a, const Polygons& b, const coord_t& detect) const
{
    const coord_t min_line = std::min(mesh_a.mesh->settings.get<coord_t>("min_wall_line_width"), mesh_b.mesh->settings.get<coord_t>("min_wall_line_width"));

    const Polygons total_shrunk = a.offset(min_line).unionPolygons(b.offset(min_line)).offset(2 * -min_line);

    Polygons from_border_a = a.difference(total_shrunk);
    Polygons from_border_b = b.difference(total_shrunk);

    Polygons temp_a, temp_b;
    for (auto _ : ranges::views::iota(0, (detect / min_line) + 2))
    {
        temp_a = from_border_a.offset(min_line);
        temp_b = from_border_b.offset(min_line);
        from_border_a = temp_a.difference(temp_b);
        from_border_b = temp_b.difference(temp_a);
    }

    return { from_border_a, from_border_b };
}

void InterlockingGenerator::handleThinAreas(const std::unordered_set<GridPoint3>& has_all_meshes) const
{
    Settings& global_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t boundary_avoidance = global_settings.get<int>("interlocking_boundary_avoidance");

    const coord_t number_of_beams_detect = boundary_avoidance;
    const coord_t number_of_beams_expand = boundary_avoidance - 1;
    constexpr coord_t rounding_errors = 5;

    const coord_t max_beam_width = std::max(beam_width_a, beam_width_b);
    const coord_t detect = (max_beam_width * number_of_beams_detect) + rounding_errors;
    const coord_t expand = (max_beam_width * number_of_beams_expand) + rounding_errors;
    const coord_t close_gaps = std::min(mesh_a.mesh->settings.get<coord_t>("line_width"), mesh_b.mesh->settings.get<coord_t>("line_width")) / 4;

    // Make an inclusionary polygon, to only actually handle thin areas near actual microstructures (so not in skin for example).
    std::vector<Polygons> near_interlock_per_layer;
    near_interlock_per_layer.assign(std::min(mesh_a.layers.size(), mesh_b.layers.size()), Polygons());
    for (const auto& cell : has_all_meshes)
    {
        const Point3 bottom_corner = vu.toLowerCorner(cell);
        for (int layer_nr = bottom_corner.z; layer_nr < bottom_corner.z + cell_size.z && layer_nr < near_interlock_per_layer.size(); ++layer_nr)
        {
            near_interlock_per_layer[layer_nr].add(vu.toPolygon(cell));
        }
    }
    for (auto& near_interlock : near_interlock_per_layer)
    {
        near_interlock = near_interlock.offset(rounding_errors).offset(-rounding_errors).unionPolygons().offset(detect);
        near_interlock.applyMatrix(rotation.inverse());
    }

    // Only alter layers when they are present in both meshes, zip should take care if that.
    for (auto [layer_nr, layer] : ranges::views::zip(mesh_a.layers, mesh_b.layers) | ranges::views::enumerate)
    {
        Polygons& polys_a = std::get<0>(layer).polygons;
        Polygons& polys_b = std::get<1>(layer).polygons;

        const auto [from_border_a, from_border_b] = growBorderAreasPerpendicular(polys_a, polys_b, detect);

        // Get the areas of each mesh that are _not_ thin (large), by performing a morphological open.
        const Polygons large_a{ polys_a.offset(-detect).offset(detect) };
        const Polygons large_b{ polys_b.offset(-detect).offset(detect) };

        // Derive the area that the thin areas need to expand into (so the added areas to the thin strips) from the information we already have.
        const Polygons thin_expansion_a{
            large_b.intersection(polys_a.difference(large_a).offset(expand)).intersection(near_interlock_per_layer[layer_nr]).intersection(from_border_a).offset(rounding_errors)
        };
        const Polygons thin_expansion_b{
            large_a.intersection(polys_b.difference(large_b).offset(expand)).intersection(near_interlock_per_layer[layer_nr]).intersection(from_border_b).offset(rounding_errors)
        };

        // Expanded thin areas of the opposing polygon should 'eat into' the larger areas of the polygon,
        // and conversely, add the expansions to their own thin areas.
        polys_a = polys_a.unionPolygons(thin_expansion_a).difference(thin_expansion_b).offset(close_gaps).offset(-close_gaps);
        polys_b = polys_b.unionPolygons(thin_expansion_b).difference(thin_expansion_a).offset(close_gaps).offset(-close_gaps);
    }
}

void InterlockingGenerator::generateInterlockingStructure() const
{
    std::vector<std::unordered_set<GridPoint3>> voxels_per_mesh = getShellVoxels(interface_dilation);

    std::unordered_set<GridPoint3>& has_any_mesh = voxels_per_mesh[0];
    std::unordered_set<GridPoint3>& has_all_meshes = voxels_per_mesh[1];
    has_any_mesh.merge(has_all_meshes); // perform union and intersection simultaneously. Cannibalizes voxels_per_mesh

    const std::vector<Polygons> layer_regions = computeUnionedVolumeRegions();

    if (air_filtering)
    {
        std::unordered_set<GridPoint3> air_cells;
        addBoundaryCells(layer_regions, air_dilation, air_cells);

        for (const GridPoint3& p : air_cells)
        {
            has_all_meshes.erase(p);
        }

        handleThinAreas(has_all_meshes);
    }

    applyMicrostructureToOutlines(has_all_meshes, layer_regions);
}

std::vector<std::unordered_set<GridPoint3>> InterlockingGenerator::getShellVoxels(const DilationKernel& kernel) const
{
    std::vector<std::unordered_set<GridPoint3>> voxels_per_mesh(2);

    // mark all cells which contain some boundary
    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        Slicer* mesh = (mesh_idx == 0) ? &mesh_a : &mesh_b;
        std::unordered_set<GridPoint3>& mesh_voxels = voxels_per_mesh[mesh_idx];

        std::vector<Polygons> rotated_polygons_per_layer(mesh->layers.size());
        for (size_t layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            SlicerLayer& layer = mesh->layers[layer_nr];
            rotated_polygons_per_layer[layer_nr] = layer.polygons;
            rotated_polygons_per_layer[layer_nr].applyMatrix(rotation);
        }

        addBoundaryCells(rotated_polygons_per_layer, kernel, mesh_voxels);
    }

    return voxels_per_mesh;
}

void InterlockingGenerator::addBoundaryCells(const std::vector<Polygons>& layers, const DilationKernel& kernel, std::unordered_set<GridPoint3>& cells) const
{
    auto voxel_emplacer = [&cells](GridPoint3 p)
    {
        cells.emplace(p);
        return true;
    };

    for (size_t layer_nr = 0; layer_nr < layers.size(); layer_nr++)
    {
        const coord_t z = layer_nr;
        vu.walkDilatedPolygons(layers[layer_nr], z, kernel, voxel_emplacer);
        Polygons skin = layers[layer_nr];
        if (layer_nr > 0)
        {
            skin = skin.xorPolygons(layers[layer_nr - 1]);
        }
        skin = skin.offset(-cell_size.x / 2).offset(cell_size.x / 2); // remove superfluous small areas, which would anyway be included because of walkPolygons
        vu.walkDilatedAreas(skin, z, kernel, voxel_emplacer);
    }
}

std::vector<Polygons> InterlockingGenerator::computeUnionedVolumeRegions() const
{
    const size_t max_layer_count = std::max(mesh_a.layers.size(), mesh_b.layers.size()) + 1; // introduce ghost layer on top for correct skin computation of topmost layer.
    std::vector<Polygons> layer_regions(max_layer_count);

    for (LayerIndex layer_nr = 0; layer_nr < max_layer_count; layer_nr++)
    {
        Polygons& layer_region = layer_regions[layer_nr];
        for (Slicer* mesh : { &mesh_a, &mesh_b })
        {
            if (layer_nr >= mesh->layers.size())
            {
                break;
            }
            const SlicerLayer& layer = mesh->layers[layer_nr];
            layer_region.add(layer.polygons);
        }
        layer_region = layer_region.offset(ignored_gap).offset(-ignored_gap); // Morphological close to merge meshes into single volume
        layer_region.applyMatrix(rotation);
    }
    return layer_regions;
}

std::vector<std::vector<Polygons>> InterlockingGenerator::generateMicrostructure() const
{
    std::vector<std::vector<Polygons>> cell_area_per_mesh_per_layer;
    cell_area_per_mesh_per_layer.resize(2);
    cell_area_per_mesh_per_layer[0].resize(2);
    const coord_t beam_w_sum = beam_width_a + beam_width_b;
    const coord_t middle = cell_size.x * beam_width_a / beam_w_sum;
    const coord_t width[2] = { middle, cell_size.x - middle };
    for (size_t mesh_idx : { 0, 1 })
    {
        Point offset(mesh_idx ? middle : 0, 0);
        Point area_size(width[mesh_idx], cell_size.y);

        PolygonRef poly = cell_area_per_mesh_per_layer[0][mesh_idx].newPoly();
        poly.emplace_back(offset);
        poly.emplace_back(offset + Point(area_size.X, 0));
        poly.emplace_back(offset + area_size);
        poly.emplace_back(offset + Point(0, area_size.Y));
    }
    cell_area_per_mesh_per_layer[1] = cell_area_per_mesh_per_layer[0];
    for (Polygons& polys : cell_area_per_mesh_per_layer[1])
    {
        for (PolygonRef poly : polys)
        {
            for (Point& p : poly)
            {
                std::swap(p.X, p.Y);
            }
        }
    }
    return cell_area_per_mesh_per_layer;
}

void InterlockingGenerator::applyMicrostructureToOutlines(const std::unordered_set<GridPoint3>& cells, const std::vector<Polygons>& layer_regions) const
{
    std::vector<std::vector<Polygons>> cell_area_per_mesh_per_layer = generateMicrostructure();

    const PointMatrix unapply_rotation = rotation.inverse();
    const size_t max_layer_count = std::max(mesh_a.layers.size(), mesh_b.layers.size());

    std::vector<Polygons> structure_per_layer[2]; // for each mesh the structure on each layer

    // Every `beam_layer_count` number of layers are combined to an interlocking beam layer
    // to store these we need ceil(max_layer_count / beam_layer_count) of these layers
    // the formula is rewritten as (max_layer_count + beam_layer_count - 1) / beam_layer_count, so it works for integer division
    size_t num_interlocking_layers = (max_layer_count + beam_layer_count - 1) / beam_layer_count;
    structure_per_layer[0].resize(num_interlocking_layers);
    structure_per_layer[1].resize(num_interlocking_layers);

    // Only compute cell structure for half the layers, because since our beams are two layers high, every odd layer of the structure will be the same as the layer below.
    for (const GridPoint3& grid_loc : cells)
    {
        Point3 bottom_corner = vu.toLowerCorner(grid_loc);
        for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
        {
            for (LayerIndex layer_nr = bottom_corner.z; layer_nr < bottom_corner.z + cell_size.z && layer_nr < max_layer_count; layer_nr += beam_layer_count)
            {
                Polygons areas_here = cell_area_per_mesh_per_layer[(layer_nr / beam_layer_count) % cell_area_per_mesh_per_layer.size()][mesh_idx];
                areas_here.translate(Point(bottom_corner.x, bottom_corner.y));
                structure_per_layer[mesh_idx][layer_nr / beam_layer_count].add(areas_here);
            }
        }
    }

    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        for (size_t layer_nr = 0; layer_nr < structure_per_layer[mesh_idx].size(); layer_nr++)
        {
            Polygons& layer_structure = structure_per_layer[mesh_idx][layer_nr];
            layer_structure = layer_structure.unionPolygons();
            layer_structure.applyMatrix(unapply_rotation);
        }
    }

    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        Slicer* mesh = (mesh_idx == 0) ? &mesh_a : &mesh_b;
        for (size_t layer_nr = 0; layer_nr < max_layer_count; layer_nr++)
        {
            if (layer_nr >= mesh->layers.size())
            {
                break;
            }

            Polygons layer_outlines = layer_regions[layer_nr];
            layer_outlines.applyMatrix(unapply_rotation);

            const Polygons areas_here = structure_per_layer[mesh_idx][layer_nr / beam_layer_count].intersection(layer_outlines);
            const Polygons& areas_other = structure_per_layer[! mesh_idx][layer_nr / beam_layer_count];

            SlicerLayer& layer = mesh->layers[layer_nr];
            layer.polygons = layer.polygons
                                 .difference(areas_other) // reduce layer areas inward with beams from other mesh
                                 .unionPolygons(areas_here); // extend layer areas outward with newly added beams
        }
    }
}

} // namespace cura
