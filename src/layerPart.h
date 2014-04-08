/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef LAYERPART_H
#define LAYERPART_H

#include "sliceDataStorage.h"
#include "slicer.h"

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundery of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura {

void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer, int unionAllType);

void createLayerParts(SliceVolumeStorage& storage, Slicer* slicer, int unionAllType);

void dumpLayerparts(SliceDataStorage& storage, const char* filename);

}//namespace cura

#endif//LAYERPART_H
