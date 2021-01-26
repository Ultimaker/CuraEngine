//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INTERLOCKING_GENERATOR_H
#define INTERLOCKING_GENERATOR_H

#include <vector>
#include <cassert>

#include "utils/polygon.h"
#include "utils/SparseCellGrid3D.h"

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
    InterlockingGenerator(std::vector<Slicer*>& volumes, std::vector<coord_t>& line_width_per_extruder, coord_t max_layer_count, const PointMatrix& rotation, Point3 cell_size)
    : volumes(volumes)
    , line_width_per_extruder(line_width_per_extruder)
    , max_layer_count(max_layer_count)
    , rotation(rotation)
    , cell_size(cell_size)
    {}

    void populateGridWithBoundaryVoxels(SparseCellGrid3D<Cell>& grid);

    void computeLayerRegions(std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights);

    void computeLayerSkins(const std::vector<Polygons>& layer_regions, std::vector<Polygons>& layer_skins);

    void removeBoundaryCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights);

    void removeSkinCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_skin, std::vector<coord_t>& layer_heights);

    void dilateCells(SparseCellGrid3D<Cell>& grid, size_t extruder_count);

    //! remove cells which are not in the intersection region
    void cleanUpNonInterface(SparseCellGrid3D<Cell>& grid);

    void generateMicrostructure(std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer);

    void applyMicrostructureToOutlines(SparseCellGrid3D<Cell>& grid, std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer);

    std::vector<Slicer*>& volumes;
    std::vector<coord_t> line_width_per_extruder;
    size_t max_layer_count;

    PointMatrix rotation;
    Point3 cell_size;
};

}//namespace cura

#endif//INTERLOCKING_GENERATOR_H
