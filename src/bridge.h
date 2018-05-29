/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef BRIDGE_H
#define BRIDGE_H

#include "sliceDataStorage.h"

namespace cura {
    class Polygons;
    class SliceLayer;

int bridgeAngle(const Polygons& skin_outline, const SliceLayer* prev_layer, const SupportLayer* support_layer, Polygons& supported_regions, const double support_threshold);

}//namespace cura

#endif//BRIDGE_H
