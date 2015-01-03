/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INSET_H
#define INSET_H

#include "sliceDataStorage.h"

namespace cura {

void generateInsets(SliceLayerPart* part, int offset, int insetCount, int shrink = 0);

void generateInsets(SliceLayer* layer, int offset, int insetCount, int shrink = 0);

}//namespace cura

#endif//INSET_H
