/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INFILL_H
#define INFILL_H

#include "utils/polygon.h"

namespace cura {

void generateConcentricInfill(Polygons outline, Polygons& result, int offsets[], int offsetsSize);
void generateLineInfill(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, int infillOverlap, double rotation);

}//namespace cura

#endif//INFILL_H
