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

// TODO make robust against if there's only 1 extruder

// TODO: make dilation user parameter

// TODO: option for different amount of dilation for shell removal

// TODO: allow for multiple models on the same extruder:
// dont add all patterns to each mesh. This leads to each mesh printing the interfaces of all other meshes.


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
    if (volumes.size() > 2)
    {
        logError("Currently too buggy to operate on multiple models! Each voxel needs to be associated to 2 meshes.");
        std::exit(-1);
    }



    const std::vector<ExtruderTrain>& extruders = Application::getInstance().current_slice->scene.extruders;
    std::vector<coord_t> line_width_per_extruder(extruder_count);
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        line_width_per_extruder[extruder_nr] = extruders[extruder_nr].settings.get<coord_t>("wall_line_width_0");
    }

    coord_t layer_height = Application::getInstance().current_slice->scene.settings.get<coord_t>("layer_height");



    // TODO: make settigns for these:
    PatternType type = PatternType::ZIGZAG;
    coord_t cell_width = (line_width_per_extruder[0] + line_width_per_extruder[1]) * 2 * 1.1;
    coord_t bulge_straight = (line_width_per_extruder[0] + line_width_per_extruder[1]) * 3 / 4;
    const bool alternating_offset = cell_width > (line_width_per_extruder[0] + line_width_per_extruder[1]) * 2;
    coord_t beam_layer_count = round_divide((line_width_per_extruder[0] + line_width_per_extruder[1]) * 2 / 3, layer_height);

    PointMatrix rotation(0.0);

    DilationKernel interface_dilation(GridPoint3(3,3,3), DilationKernel::Type::PRISM);

    constexpr bool air_filtering = true;
    DilationKernel air_dilation(GridPoint3(1,1,1), DilationKernel::Type::DIAMOND);


    Point3 cell_size(cell_width, cell_width, 2 * beam_layer_count * layer_height);
    
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



    InterlockingGenerator gen(volumes, line_width_per_extruder, layer_heights, rotation, cell_size, type, bulge_straight, beam_layer_count);

    std::vector<std::unordered_set<GridPoint3>> voxels_per_extruder = gen.getShellVoxels(interface_dilation);

    std::vector<Polygons> layer_regions(layer_heights.size());
    gen.computeLayerRegions(layer_regions);

    std::unordered_set<GridPoint3>& has_any_extruder = voxels_per_extruder[0];
    std::unordered_set<GridPoint3>& has_all_extruders = voxels_per_extruder[1];
    has_any_extruder.merge(has_all_extruders);

    if (air_filtering)
    {
        std::unordered_set<GridPoint3> air_cells;
        gen.addBoundaryCells(layer_regions, air_dilation, air_cells);

        for (const GridPoint3& p : air_cells)
        {
            has_all_extruders.erase(p);
        }
    }

    std::vector<std::vector<Polygons>> cell_area_per_extruder_per_layer;
    gen.generateMicrostructure(cell_area_per_extruder_per_layer, type, alternating_offset);

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
        skin = skin.offset(-cell_size.x / 2).offset(cell_size.x / 2); // remove superfluous small areas, which would anyway be included because of walkPolygons
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

void InterlockingGenerator::generateMicrostructure(std::vector<std::vector<Polygons>>& cell_area_per_extruder_per_layer, PatternType type, bool alternating_offset)
{
    cell_area_per_extruder_per_layer.resize(alternating_offset? 4 : 2);
    cell_area_per_extruder_per_layer[0].resize(2);
    const coord_t line_w_sum = line_width_per_extruder[0] + line_width_per_extruder[1];
    const coord_t middle = cell_size.x * line_width_per_extruder[0] / line_w_sum;
    const coord_t width[2] = { middle, cell_size.x - middle };
    if (type == PatternType::STRAIGHT || cell_size.x <= (line_width_per_extruder[0] + line_width_per_extruder[1]) * 2)
    {
        for (size_t extruder_nr : {0, 1})
        {
            Point offset(extruder_nr? middle : 0, 0);
            Point area_size(width[extruder_nr], cell_size.y);

            PolygonRef poly = cell_area_per_extruder_per_layer[0][extruder_nr].newPoly();
            poly.emplace_back(offset);
            poly.emplace_back(offset + Point(area_size.X, 0));
            poly.emplace_back(offset + area_size);
            poly.emplace_back(offset + Point(0, area_size.Y));
        }
    }
    else if (type == PatternType::BULGING)
    {
        const coord_t h = cell_size.x / 4;
        const coord_t r = (cell_size.x - line_w_sum * 2) / 4;
        const coord_t& w = middle;
        const coord_t s = bulge_straight / 2;
        {
            PolygonRef poly = cell_area_per_extruder_per_layer[0][0].newPoly();
            poly.emplace_back(0, 4*h);
            poly.emplace_back(r, 3*h+s);
            if (s > 0)
            {
                poly.emplace_back(r, 3*h-s);
                poly.emplace_back(-r, h+s);
            }
            poly.emplace_back(-r, h-s);
            poly.emplace_back(0, 0);
            poly.emplace_back(w, 0);
            poly.emplace_back(w+r, h-s);
            if (s > 0)
            {
                poly.emplace_back(w+r, h+s);
                poly.emplace_back(w-r, 3*h-s);
            }
            poly.emplace_back(w-r, 3*h+s);
            poly.emplace_back(w, 4*h);
        }
        {
            PolygonRef poly = cell_area_per_extruder_per_layer[0][1].newPoly();
            coord_t e = cell_size.x;
            poly.emplace_back(w, 4*h);
            poly.emplace_back(w-r, 3*h+s);
            if (s > 0)
            {
                poly.emplace_back(w-r, 3*h-s);
                poly.emplace_back(w+r, h+s);
            }
            poly.emplace_back(w+r, h-s);
            poly.emplace_back(w, 0);
            poly.emplace_back(e, 0);
            poly.emplace_back(e-r, h-s);
            if (s > 0)
            {
                poly.emplace_back(e-r, h+s);
                poly.emplace_back(e+r, 3*h-s);
            }
            poly.emplace_back(e+r, 3*h+s);
            poly.emplace_back(e, 4*h);
        }
    }
    else if (type == PatternType::ZIGZAG)
    {
        const coord_t h = cell_size.x / 2;
        const coord_t& w = middle;
        const double inside = 1.0 - double(line_w_sum*line_w_sum) / double(h*h);
        const coord_t r = (h*h * std::sqrt(inside)) / (2 * line_w_sum);
        {
            PolygonRef poly = cell_area_per_extruder_per_layer[0][0].newPoly();
            poly.emplace_back(r, 2*h);
            poly.emplace_back(-r, h);
            poly.emplace_back(r, 0);
            poly.emplace_back(w + r, 0);
            poly.emplace_back(w - r, h);
            poly.emplace_back(w + r, 2*h);
        }
        {
            PolygonRef poly = cell_area_per_extruder_per_layer[0][1].newPoly();
            poly.emplace_back(w + r, 2*h);
            poly.emplace_back(w - r, h);
            poly.emplace_back(w + r, 0);
            poly.emplace_back(2*h + r, 0);
            poly.emplace_back(2*h - r, h);
            poly.emplace_back(2*h + r, 2*h);
        }
    }
    cell_area_per_extruder_per_layer[1] = cell_area_per_extruder_per_layer[0];
    for (Polygons& polys : cell_area_per_extruder_per_layer[1])
    {
        for (PolygonRef poly : polys)
        {
            for (Point& p : poly)
            {
                std::swap(p.X, p.Y);
            }
        }
    }
    if (type == PatternType::ZIGZAG)
    { // swap top and bottom area
        cell_area_per_extruder_per_layer[1][0].translate(Point(0, cell_size.x - middle));
        cell_area_per_extruder_per_layer[1][1].translate(Point(0, - middle));
    }
    if (alternating_offset)
    {
        cell_area_per_extruder_per_layer[2] = cell_area_per_extruder_per_layer[0];
        cell_area_per_extruder_per_layer[3] = cell_area_per_extruder_per_layer[1];
        for (size_t layer_nr : {2, 3})
        {
            for (Polygons& polys : cell_area_per_extruder_per_layer[layer_nr])
            {
                for (PolygonRef poly : polys)
                {
                    for (Point& p : poly)
                    {
                        p = Point(cell_size.x, cell_size.y) - p;
                    }
                }
            }
        }
    }
}

void InterlockingGenerator::applyMicrostructureToOutlines(const std::unordered_set<GridPoint3>& cells, std::vector<std::vector<Polygons>>& cell_area_per_extruder_per_layer, const std::vector<Polygons>& layer_regions)
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

                Polygons areas_here = cell_area_per_extruder_per_layer[(layer_nr / beam_layer_count) % cell_area_per_extruder_per_layer.size()][extruder_nr];
                Polygons areas_other = cell_area_per_extruder_per_layer[(layer_nr / beam_layer_count) % cell_area_per_extruder_per_layer.size()][ ! extruder_nr];

                areas_here.translate(Point(bottom_corner.x, bottom_corner.y));
                areas_other.translate(Point(bottom_corner.x, bottom_corner.y));

                const Polygons& layer_region = layer_regions[layer_nr];
                areas_here = layer_region.intersection(areas_here);

                areas_here.applyMatrix(unapply_rotation);
                areas_other.applyMatrix(unapply_rotation);

                layer.polygons = layer.polygons.unionPolygons(areas_here).difference(areas_other);
            }
        }
    }
    // TODO: remove this code and prevent any bug which this code solves in thefirst place
    for (Slicer* mesh : volumes)
    {
        for (unsigned int layer_nr = 0; layer_nr < mesh->layers.size(); layer_nr++)
        {
            SlicerLayer& layer = mesh->layers[layer_nr];
            layer.polygons = layer.polygons.offset(10).offset(-10);
        }
        if (false)
        {
            Polygons& layer = mesh->layers[0].polygons;
            SVG svg("interlock.svg", AABB(layer));
            svg.writePolygons(layer);
        }
    }
}

}//namespace cura
