/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIRT_H
#define SKIRT_H

#include "sliceDataStorage.h"

namespace cura {

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count, int minLength, int initialLayerHeight);

}//namespace cura

#endif//SKIRT_H
