/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SUPPORT_H
#define SUPPORT_H

#include "sliceDataStorage.h"
#include "modelFile/modelFile.h"

namespace cura {


void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count);



}//namespace cura

#endif//SUPPORT_H
