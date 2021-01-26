//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InterlockingGenerator.h"

#include <algorithm>
#include <unordered_set>

#include "Application.h"
#include "Slice.h"
#include "slicer.h"
#include "utils/polygonUtils.h"
#include "utils/VoxelUtils.h"

#include "utils/SparseCellGrid3D.h"

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
    
    
    size_t max_layer_count = 0;
    for (Slicer* mesh : volumes)
    {
        max_layer_count = std::max(max_layer_count, mesh->layers.size());
    }
    
    PointMatrix rotation(45.0);
    
    InterlockingGenerator gen(volumes, line_width_per_extruder, max_layer_count, rotation, cell_size);
    
    // TODO: implement lesser dilation based on translated polygons
    // TODO: make dilation user parameter
    
    // TODO: adjust voxel height to 2x layer height? or 4x? make adjustable?
    
    // TODO: implement oscillating fingers
    
    // TODO: option for different amount of dilation for shell removal
    
    
    SparseCellGrid3D<Cell> grid(cell_size);

    gen.populateGridWithBoundaryVoxels(grid);
    
    std::vector<Polygons> layer_regions(max_layer_count);
    std::vector<coord_t> layer_heights(max_layer_count);
    gen.computeLayerRegions(layer_regions, layer_heights);

    std::vector<Polygons> layer_skins(max_layer_count);
    gen.computeLayerSkins(layer_regions, layer_skins);
    
    gen.removeBoundaryCells(grid, layer_regions, layer_heights);
    
    gen.removeSkinCells(grid, layer_skins, layer_heights);
    
    gen.dilateCells(grid, extruder_count);
    
    gen.cleanUpNonInterface(grid);
    
    std::vector<std::vector<Polygon>> cell_area_per_extruder_per_layer;
    gen.generateMicrostructure(cell_area_per_extruder_per_layer);

    gen.applyMicrostructureToOutlines(grid, cell_area_per_extruder_per_layer);
}

InterlockingGenerator::Cell::Cell()
: has_extruder(Application::getInstance().current_slice->scene.extruders.size(), false)
{
    assert(has_extruder.size() >= 1);
}

void InterlockingGenerator::populateGridWithBoundaryVoxels(SparseCellGrid3D<Cell>& grid)
{
    const Cell default_cell;

    // mark all cells which contain some boundary
    for (Slicer* mesh : volumes)
    {
        size_t extruder_nr = mesh->mesh->settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<size_t>("extruder_nr");
        std::vector<Polygons> rotated_polygons_per_layer(mesh->layers.size());
        for (size_t layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            SlicerLayer& layer = mesh->layers[layer_nr];
            rotated_polygons_per_layer[layer_nr] = layer.polygons;
            rotated_polygons_per_layer[layer_nr].applyMatrix(rotation);
        }
        assert(extruder_nr < 2);
        
        for (size_t layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            coord_t z = mesh->layers[layer_nr].z;
            for (ConstPolygonRef poly : rotated_polygons_per_layer[layer_nr])
            {
                Point last = poly.back();
                for (Point p : poly)
                {
                    grid.processLineCells(std::make_pair(Point3(last.X, last.Y, z), Point3(p.X, p.Y, z)),
                                          [extruder_nr, &grid, &default_cell](SparseCellGrid3D<Cell>::GridPoint3 grid_loc) -> bool
                                          {
                                              Cell& cell = grid.getCell(grid_loc, default_cell);
                                              assert(extruder_nr < cell.has_extruder.size());
                                              cell.has_extruder[extruder_nr] = true;
                                              return true; // keep going marking cells along this line
                                          }
                    );
                    
                    last = p;
                }
            }
        }
    }
}

void InterlockingGenerator::computeLayerRegions(std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights)
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

void InterlockingGenerator::computeLayerSkins(const std::vector<Polygons>& layer_regions, std::vector<Polygons>& layer_skin)
{
    for (size_t layer_nr = 0; layer_nr < layer_regions.size(); layer_nr++)
    {
        Polygons empty;
        const Polygons& below = (layer_nr >= 1)? layer_regions[layer_nr - 1] : empty;
        const Polygons& above = (layer_nr + 1 < layer_regions.size())? layer_regions[layer_nr + 1] : empty;
        Polygons bottom_skin = layer_regions[layer_nr].difference(below);
        Polygons top_skin = layer_regions[layer_nr].difference(above);
        layer_skin[layer_nr] = bottom_skin.unionPolygons(top_skin);
    }
}

void InterlockingGenerator::removeBoundaryCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights)
{
    for (size_t layer_nr = 0; layer_nr < layer_regions.size(); layer_nr++)
    {
        coord_t z = layer_heights[layer_nr];
        for (ConstPolygonRef poly : layer_regions[layer_nr])
        {
            Point last = poly.back();
            for (Point p : poly)
            {
                grid.processLineCells(std::make_pair(Point3(last.X, last.Y, z), Point3(p.X, p.Y, z)),
                    [&grid](SparseCellGrid3D<Cell>::GridPoint3 grid_loc) -> bool
                    {
                        for (coord_t offset_x : {-1, 0, 1})
                        {
                            for (coord_t offset_y : {-1, 0, 1})
                            {
                                for (coord_t offset_z : {-1, 0, 1})
                                {
                                    grid.removeCell(grid_loc + Point3(offset_x, offset_y, offset_z));
                        }   }   }
                        return true; // keep going removing cells
                    }
                );
                last = p;
            }
        }
    }
}

void InterlockingGenerator::removeSkinCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_skin, std::vector<coord_t>& layer_heights)
{
    for (size_t layer_nr = 0; layer_nr < layer_skin.size(); layer_nr++)
    {
        std::vector<Point> skin_points = PolygonUtils::spreadDotsArea(layer_skin[layer_nr], cell_size.x);
        for (Point skin_point : skin_points)
        {
            for (coord_t offset_x : {-1, 0, 1})
                for (coord_t offset_y : {-1, 0, 1})
                    for (coord_t offset_z : {-1, 0, 1})
                    {
                        grid.removeCell(grid.toGridPoint(Point3(skin_point.X, skin_point.Y, layer_heights[layer_nr])) + Point3(offset_x, offset_y, offset_z));
                    }
        }
    }
}

void InterlockingGenerator::dilateCells(SparseCellGrid3D<Cell>& grid, size_t extruder_count)
{
    std::vector<std::unordered_set<Point3>> dilated_cells_per_extruder(extruder_count);
    const std::vector<Point3> rel_dilation_locations = {
        Point3(-1,0,0), Point3(1,0,0),
        Point3(0,-1,0), Point3(0,1,0),
        Point3(0,0,-1), Point3(0,0,1) };
    for (auto& pair : grid.m_grid)
    {
        const Cell& cell = pair.second;
        for (size_t extruder_nr = 0; extruder_nr < cell.has_extruder.size(); extruder_nr++)
        {
            if (cell.has_extruder[extruder_nr])
            {
                for (Point3 rel_dilation_location : rel_dilation_locations)
                {
                    dilated_cells_per_extruder[extruder_nr].insert(pair.first + rel_dilation_location);
                }
            }
        }
    }

    const Cell default_cell;

    // apply dilation
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const std::unordered_set<Point3>& dilated_cells = dilated_cells_per_extruder[extruder_nr];
        for (Point3 grid_loc : dilated_cells)
        {
            grid.getCell(grid_loc, default_cell).has_extruder[extruder_nr] = true;
        }
    }
}

// remove cells which are not in the intersection region
void InterlockingGenerator::cleanUpNonInterface(SparseCellGrid3D<Cell>& grid)
{
    for (auto it = grid.m_grid.begin(); it != grid.m_grid.end(); )
    {
        int occupied_extruder_count = 0;
        const Cell& cell = it->second;
        for (bool has_extr : cell.has_extruder)
        {
            occupied_extruder_count += has_extr;
        }
        if (occupied_extruder_count <= 1)
        {
            it = grid.m_grid.erase(it);
        }
        else
        {
            ++it;
        }
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

void InterlockingGenerator::applyMicrostructureToOutlines(SparseCellGrid3D<Cell>& grid, std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer)
{
    PointMatrix unapply_rotation = rotation.inverse();

    for (auto key_val : grid.m_grid)
    {
        Point3 grid_loc = key_val.first;
        Point3 bottom_corner = grid.toLowerCorner(grid_loc);
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

                areas_here.applyMatrix(unapply_rotation);
                areas_other.applyMatrix(unapply_rotation);

                layer.polygons = layer.polygons.unionPolygons(areas_here).difference(areas_other);
            }
        }
    }
}

}//namespace cura
