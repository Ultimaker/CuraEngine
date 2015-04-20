/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INSET_H
#define INSET_H

#include "sliceDataStorage.h"

namespace cura {

void generateInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters);

void generateInsets(SliceLayer* layer, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters);

}//namespace cura

#endif//INSET_H
