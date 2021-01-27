//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INTERLOCKING_GENERATOR_H
#define INTERLOCKING_GENERATOR_H

#include <vector>
#include <cassert>
#include <unordered_set>

#include "utils/polygon.h"
#include "utils/VoxelUtils.h"

namespace cura
{

class Slicer;

class InterlockingGenerator
{
public:
    static void generateInterlockingStructure(std::vector<Slicer*>& volumes);

    /*!
     * Voxel cell for interlocking microstructure
     */
    struct Cell
    {
        std::vector<bool> has_extruder;
        Cell();
    };

protected:
    InterlockingGenerator(std::vector<Slicer*>& volumes, std::vector<coord_t>& line_width_per_extruder, const std::vector<coord_t>& layer_heights, const PointMatrix& rotation, Point3 cell_size)
    : volumes(volumes)
    , line_width_per_extruder(line_width_per_extruder)
    , layer_heights(layer_heights)
    , vu(cell_size)
    , rotation(rotation)
    , cell_size(cell_size)
    {}

    std::vector<std::unordered_set<GridPoint3>> getShellVoxels(const DilationKernel& kernel);
    
    
    void addBoundaryCells(std::vector<Polygons>& layers, const DilationKernel& kernel, std::unordered_set<GridPoint3>& cells);

    void computeLayerRegions(std::vector<Polygons>& layer_regions);

    void generateMicrostructure(std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer);

    void applyMicrostructureToOutlines(const std::unordered_set<GridPoint3>& cells, std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer);

    std::vector<Slicer*>& volumes;
    std::vector<coord_t> line_width_per_extruder;
    std::vector<coord_t> layer_heights;

    VoxelUtils vu;

    PointMatrix rotation;
    Point3 cell_size;
};

}//namespace cura

#endif//INTERLOCKING_GENERATOR_H
