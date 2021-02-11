//Copyright (c) 2018 Ultimaker B.V.
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

void InterlockingGenerator::generateInterlockingStructure(std::vector<Slicer*>& volumes)
{
    /*
    if ( ! Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("interlocking_structure_gen"))
    {
        return;
    }
    */

    
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    if (extruder_count > 2)
    {
        // TODO: fix for > 2 extruders (apply first to extruder 0 & 1, then to 0 & 2, etc.)
        logError("generateInterlockingStructure not implemented for >2 extruders!\n");
        return;
    }
    
    const std::vector<ExtruderTrain>& extruders = Application::getInstance().current_slice->scene.extruders;
    std::vector<coord_t> line_width_per_extruder(extruder_count);
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        line_width_per_extruder[extruder_nr] = extruders[extruder_nr].settings.get<coord_t>("wall_line_width_0");
    }
    coord_t cell_width = (line_width_per_extruder[0] + line_width_per_extruder[1]) * 2;
    Point3 cell_size(cell_width, cell_width, 4 * Application::getInstance().current_slice->scene.settings.get<coord_t>("layer_height"));
    // TODO make robust against if there's only 1 extruder
    
    
    std::vector<coord_t> layer_heights;
    {
        size_t layer_idx = 0;
        for (Slicer* mesh : volumes)
        {
            layer_heights.resize(std::max(layer_heights.size(), mesh->layers.size()));
            for ( ; layer_idx < mesh->layers.size() ; layer_idx++)
            {
                layer_heights[layer_idx] = mesh->layers[layer_idx].z;
            }
        }
    }
    
    PointMatrix rotation(45.0);
    
    InterlockingGenerator gen(volumes, line_width_per_extruder, layer_heights, rotation, cell_size);
    
    // TODO: implement lesser dilation based on translated polygons
    // TODO: make dilation user parameter
    
    // TODO: adjust voxel height to 2x layer height? or 4x? make adjustable?
    
    // TODO: implement oscillating fingers
    
    // TODO: option for different amount of dilation for shell removal
    

    DilationKernel interface_dilation(GridPoint3(2,2,4), true);
    std::vector<std::unordered_set<GridPoint3>> voxels_per_extruder = gen.getShellVoxels(interface_dilation);

    std::vector<Polygons> layer_regions(layer_heights.size());
    gen.computeLayerRegions(layer_regions);

    DilationKernel air_dilation(GridPoint3(1,1,1), true);
    std::unordered_set<GridPoint3> air_cells;
    gen.addBoundaryCells(layer_regions, air_dilation, air_cells);

    std::unordered_set<GridPoint3>& has_any_extruder = voxels_per_extruder[0];
    std::unordered_set<GridPoint3>& has_all_extruders = voxels_per_extruder[1];
    has_any_extruder.merge(has_all_extruders);

    for (const GridPoint3& p : air_cells)
    {
        has_all_extruders.erase(p);
    }

    std::vector<std::vector<Polygon>> cell_area_per_extruder_per_layer;
    gen.generateMicrostructure(cell_area_per_extruder_per_layer);

    gen.applyMicrostructureToOutlines(has_all_extruders, cell_area_per_extruder_per_layer, layer_regions);
}

InterlockingGenerator::Cell::Cell()
: has_extruder(Application::getInstance().current_slice->scene.extruders.size(), false)
{
    assert(has_extruder.size() >= 1);
}

std::vector<std::unordered_set<GridPoint3>> InterlockingGenerator::getShellVoxels(const DilationKernel& kernel)
{
    std::vector<std::unordered_set<GridPoint3>> voxels_per_extruder(2);

    // mark all cells which contain some boundary
    for (Slicer* mesh : volumes)
    {
        size_t extruder_nr = mesh->mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<size_t>("extruder_nr");
        assert(extruder_nr < 2);
        std::unordered_set<GridPoint3>& mesh_voxels = voxels_per_extruder[extruder_nr];
        
        
        std::vector<Polygons> rotated_polygons_per_layer(mesh->layers.size());
        for (size_t layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            SlicerLayer& layer = mesh->layers[layer_nr];
            rotated_polygons_per_layer[layer_nr] = layer.polygons;
            rotated_polygons_per_layer[layer_nr].applyMatrix(rotation);
        }
        
        addBoundaryCells(rotated_polygons_per_layer, kernel, mesh_voxels);
    }
    
    return voxels_per_extruder;
}

void InterlockingGenerator::addBoundaryCells(std::vector<Polygons>& layers, const DilationKernel& kernel, std::unordered_set<GridPoint3>& cells)
{
    auto voxel_emplacer = [&cells](GridPoint3 p) { cells.emplace(p); return true; };

    for (size_t layer_nr = 0; layer_nr < layers.size(); layer_nr++)
    {
        coord_t z = layer_heights[layer_nr];
        vu.walkDilatedPolygons(layers[layer_nr], z, kernel, voxel_emplacer);
        Polygons skin = layers[layer_nr];
        if (layer_nr > 0)
        {
            skin = skin.xorPolygons(layers[layer_nr - 1]);
        }
        skin = skin.offset(-cell_size.x / 2); // remove superfluous small areas
        vu.walkDilatedAreas(skin, z, kernel, voxel_emplacer);
    }
}

void InterlockingGenerator::computeLayerRegions(std::vector<Polygons>& layer_regions)
{
    for (unsigned int layer_nr = 0; layer_nr < layer_regions.size(); layer_nr++)
    {
        Polygons& layer_region = layer_regions[layer_nr];
        coord_t z;
        for (Slicer* mesh : volumes)
        {
            if (layer_nr >= mesh->layers.size()) break;
            SlicerLayer& layer = mesh->layers[layer_nr];
            z = layer.z;
            layer_region.add(layer.polygons);
        }
        layer_region = layer_region.offset(100).offset(-100); // TODO hardcoded value
        layer_region.applyMatrix(rotation);
        layer_heights[layer_nr] = z;
    }
}

void InterlockingGenerator::generateMicrostructure(std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer)
{
    cell_area_per_extruder_per_layer.resize(4);
    for (size_t layer_nr : {0, 1, 2, 3})
    {
        cell_area_per_extruder_per_layer[layer_nr].resize(2);
        for (size_t extruder_nr : {0, 1})
        {
            Point offset((extruder_nr == layer_nr / 2)? line_width_per_extruder[ ! extruder_nr] * 2 : 0, 0);
            Point area_size(line_width_per_extruder[extruder_nr] * 2, cell_size.x);
            if (layer_nr % 2)
            {
                std::swap(offset.X, offset.Y);
                std::swap(area_size.X, area_size.Y);
            }
            PolygonRef poly = cell_area_per_extruder_per_layer[layer_nr][extruder_nr];
            poly.emplace_back(offset);
            poly.emplace_back(offset + Point(area_size.X, 0));
            poly.emplace_back(offset + area_size);
            poly.emplace_back(offset + Point(0, area_size.Y));
        }
    }
}

void InterlockingGenerator::applyMicrostructureToOutlines(const std::unordered_set<GridPoint3>& cells, std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer, const std::vector<Polygons>& layer_regions)
{
    PointMatrix unapply_rotation = rotation.inverse();

    for (const GridPoint3& grid_loc : cells)
    {
        Point3 bottom_corner = vu.toLowerCorner(grid_loc);
        for (Slicer* mesh : volumes)
        {
            size_t extruder_nr = mesh->mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<size_t>("extruder_nr");
            for (unsigned int layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
            {
                SlicerLayer& layer = mesh->layers[layer_nr];
                coord_t z = layer.z;
                if (z < bottom_corner.z) continue;
                if (z > bottom_corner.z + cell_size.z) break;

                Polygon area_here = cell_area_per_extruder_per_layer[layer_nr % cell_area_per_extruder_per_layer.size()][extruder_nr];
                Polygon area_other = cell_area_per_extruder_per_layer[layer_nr % cell_area_per_extruder_per_layer.size()][ ! extruder_nr];

                area_here.translate(Point(bottom_corner.x, bottom_corner.y));
                area_other.translate(Point(bottom_corner.x, bottom_corner.y));

                Polygons areas_here;
                areas_here.add(area_here);
                Polygons areas_other;
                areas_other.add(area_other);

                const Polygons& layer_region = layer_regions[layer_nr];
                areas_here = layer_region.intersection(areas_here);

                areas_here.applyMatrix(unapply_rotation);
                areas_other.applyMatrix(unapply_rotation);

                layer.polygons = layer.polygons.unionPolygons(areas_here).difference(areas_other);
            }
        }
    }
}

}//namespace cura
