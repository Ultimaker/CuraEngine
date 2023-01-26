//Copyright (c) 2023 UltiMaker
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InterlockingGenerator.h"

#include <algorithm> // max

#include "Application.h"
#include "Slice.h"
#include "slicer.h"
#include "utils/polygonUtils.h"
#include "utils/VoxelUtils.h"

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

            if (extruder_nr_a == extruder_nr_b || !mesh_a.mesh->getAABB().expand(ignored_gap).hit(mesh_b.mesh->getAABB()))
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

void InterlockingGenerator::generateInterlockingStructure()
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
    }

    applyMicrostructureToOutlines(has_all_meshes, layer_regions);
}

std::vector<std::unordered_set<GridPoint3>> InterlockingGenerator::getShellVoxels(const DilationKernel& kernel) const
{
    std::vector<std::unordered_set<GridPoint3>> voxels_per_mesh(2);

    // mark all cells which contain some boundary
    for (size_t mesh_idx = 0; mesh_idx < 2; mesh_idx++)
    {
        Slicer* mesh = (mesh_idx == 0)? &mesh_a : &mesh_b;
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
    auto voxel_emplacer = [&cells](GridPoint3 p) { cells.emplace(p); return true; };

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

    for (unsigned int layer_nr = 0; layer_nr < max_layer_count; layer_nr++)
    {
        Polygons& layer_region = layer_regions[layer_nr];
        for (Slicer* mesh : {&mesh_a, &mesh_b})
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
    for (size_t mesh_idx : {0, 1})
    {
        Point offset(mesh_idx? middle : 0, 0);
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
            for (unsigned int layer_nr = bottom_corner.z; layer_nr < bottom_corner.z + cell_size.z && layer_nr < max_layer_count; layer_nr += beam_layer_count)
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
        Slicer* mesh = (mesh_idx == 0)? &mesh_a : &mesh_b;
        for (size_t layer_nr = 0; layer_nr < max_layer_count; layer_nr++)
        {
            if (layer_nr >= mesh->layers.size())
            {
                break;
            }

            Polygons layer_outlines = layer_regions[layer_nr];
            layer_outlines.applyMatrix(unapply_rotation);

            const Polygons areas_here = structure_per_layer[mesh_idx][layer_nr / beam_layer_count].intersection(layer_outlines);
            const Polygons& areas_other = structure_per_layer[ ! mesh_idx][layer_nr / beam_layer_count];

            SlicerLayer& layer = mesh->layers[layer_nr];
            layer.polygons = layer.polygons.difference(areas_other) // reduce layer areas inward with beams from other mesh
                                            .unionPolygons(areas_here); // extend layer areas outward with newly added beams
        }
    }
}

}//namespace cura
