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
        std::vector<bool> has_mesh;
        Cell();
    };

protected:
    
    static void generateInterlockingStructure(Slicer& mesh_a, Slicer& mesh_b);

    InterlockingGenerator(Slicer& mesh_a, Slicer& mesh_b, coord_t (& line_width_per_mesh)[2], const std::vector<coord_t>& layer_heights, const PointMatrix& rotation, Point3 cell_size, coord_t beam_layer_count)
    : mesh_a(mesh_a)
    , mesh_b(mesh_b)
    , line_width_per_mesh(line_width_per_mesh)
    , layer_heights(layer_heights)
    , vu(cell_size)
    , rotation(rotation)
    , cell_size(cell_size)
    , beam_layer_count(beam_layer_count)
    {}

    /*!
     * 
     * \return The shell voxels for mesh a and those for mesh b
     */
    std::vector<std::unordered_set<GridPoint3>> getShellVoxels(const DilationKernel& kernel) const;
    
    
    void addBoundaryCells(std::vector<Polygons>& layers, const DilationKernel& kernel, std::unordered_set<GridPoint3>& cells) const;

    void computeLayerRegions(std::vector<Polygons>& layer_regions) const;

    void generateMicrostructure(std::vector<std::vector<Polygons>>& cell_area_per_mesh_per_layer) const;

    void applyMicrostructureToOutlines(const std::unordered_set<GridPoint3>& cells, std::vector<std::vector<Polygons>>& cell_area_per_mesh_per_layer, const std::vector<Polygons>& layer_regions) const;

    static const coord_t ignored_gap = 100u; //!< Distance between models to be considered next to each other so that an interlocking structure will be generated there

    Slicer& mesh_a;
    Slicer& mesh_b;
    coord_t (& line_width_per_mesh)[2]; // reference to an array of length 2
    std::vector<coord_t> layer_heights;

    VoxelUtils vu;

    PointMatrix rotation;
    Point3 cell_size;
    coord_t beam_layer_count;
};

}//namespace cura

#endif//INTERLOCKING_GENERATOR_H
