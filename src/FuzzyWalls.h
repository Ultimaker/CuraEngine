/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef FUZZY_WALLS_H
#define FUZZY_WALLS_H

#include "sliceDataStorage.h"

namespace cura {

class FuzzyWalls
{
public:
    Polygons makeFuzzy(const SliceMeshStorage& mesh, const unsigned int layer_nr, const Polygons& in);

};

}//namespace cura

#endif//FUZZY_WALLS_H
