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

}//namespace cura

#endif//MULTIVOLUMES_H
