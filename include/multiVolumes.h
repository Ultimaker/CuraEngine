//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MULTIVOLUMES_H
#define MULTIVOLUMES_H

#include <vector>
#include <cassert>

#include "utils/polygon.h"
#include "utils/SparseCellGrid3D.h"

/* This file contains code to help fixing up and changing layers that are built from multiple volumes. */
namespace cura
{

class Mesh;
class Slicer;

void carveMultipleVolumes(std::vector<Slicer*> &meshes);

/*!
 * Expand each layer a bit and then keep the extra overlapping parts that overlap with other volumes.
 * This generates some overlap in dual extrusion, for better bonding in touching parts.
 */
void generateMultipleVolumesOverlap(std::vector<Slicer*> &meshes);

class MultiVolumes
{
public:
    /*!
     * Carve all cutting meshes.
     * 
     * Make a cutting mesh limited to within the volume of all other meshes
     * and carve this volume from those meshes.
     * 
     * Don't carve other cutting meshes.
     * 
     * \warning Overlapping cutting meshes result in overlapping volumes. \ref carveMultipleVolumes still needs to be called
     * 
     * \param[in,out] volumes The outline data of each mesh
     * \param meshes The meshes which contain the settings for each volume
     */
    static void carveCuttingMeshes(std::vector<Slicer*>& volumes, const std::vector<Mesh>& meshes);

    static void generateInterlockingStructure(std::vector<Slicer*>& volumes);

protected:
    /*!
     * Voxel cell for interlocking microstructure
     */
    struct Cell
    {
        std::vector<bool> has_extruder;
        Cell();
    };

    static void populateGridWithBoundaryVoxels(const std::vector<Slicer*>& volumes, SparseCellGrid3D<Cell>& grid);
    
    static void computeLayerRegions(const std::vector<Slicer*>& volumes, std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights);
    
    static void computeLayerSkins(const std::vector<Polygons>& layer_regions, std::vector<Polygons>& layer_skins);
    
    static void removeBoundaryCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_regions, std::vector<coord_t>& layer_heights);
    
    static void removeSkinCells(SparseCellGrid3D<Cell>& grid, const std::vector<Polygons>& layer_skin, std::vector<coord_t>& layer_heights, coord_t cell_size);
    
    static void dilateCells(SparseCellGrid3D<Cell>& grid, size_t extruder_count);
    
    //! remove cells which are not in the intersection region
    static void cleanUpNonInterface(SparseCellGrid3D<Cell>& grid);
    
    static void generateMicrostructure(std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer, const std::vector<coord_t>& line_width_per_extruder, coord_t cell_size);
    
    static void applyMicrostructureToOutlines(SparseCellGrid3D<Cell>& grid, std::vector<std::vector<Polygon>>& cell_area_per_extruder_per_layer, std::vector<Slicer*>& volumes, coord_t cell_size);
};

}//namespace cura

#endif//MULTIVOLUMES_H
