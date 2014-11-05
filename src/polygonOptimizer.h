/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef POLYGON_OPTIMIZER_H
#define POLYGON_OPTIMIZER_H

#include "utils/polygon.h"

namespace cura {

void optimizePolygon(PolygonRef poly);

void optimizePolygons(Polygons& polys);

}//namespace cura

#endif//POLYGON_OPTIMIZER_H
