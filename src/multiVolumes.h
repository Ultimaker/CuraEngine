#ifndef MULTIVOLUMES_H
#define MULTIVOLUMES_H

#include "sliceDataStorage.h"
#include "slicer.h"

/* This file contains code to help fixing up and changing layers that are build from multiple volumes. */
namespace cura {

/*!
 * 
 * \param alternate_carve_order Whether to switch which model carves out of which with every layer
 */
void carveMultipleVolumes(std::vector<Slicer*> &meshes, bool alternate_carve_order);

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
};

}//namespace cura

#endif//MULTIVOLUMES_H
