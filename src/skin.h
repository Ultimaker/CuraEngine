/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura {

void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int insetCount, int avoidOverlappingPerimeters);
void generateSkinAreas(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount);
void generateSkinInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters);
void generateSparse(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int avoidOverlappingPerimeters);
void combineSparseLayers(int layerNr, SliceMeshStorage& storage, int amount);

}//namespace cura

#endif//SKIN_H
