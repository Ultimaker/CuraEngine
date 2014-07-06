/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIN_H
#define SKIN_H

#include "sliceDataStorage.h"

namespace cura {

  void generateSparseAndSkins(int layerNr, SliceVolumeStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int infillOverlap);

}//namespace cura

#endif//SKIN_H
