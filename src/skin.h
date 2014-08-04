/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura {

void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int infillOverlap);
void generateSparse(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount);
void combineSparseLayers(int layerNr, SliceMeshStorage& storage, int amount);

}//namespace cura

#endif//SKIN_H
