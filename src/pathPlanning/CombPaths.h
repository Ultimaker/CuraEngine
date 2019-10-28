//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATH_PLANNING_COMB_PATHS_H
#define PATH_PLANNING_COMB_PATHS_H

#include "CombPath.h"

namespace cura 
{

struct CombPaths : public std::vector<CombPath> //!< A list of paths alternating between inside a part and outside a part
{
    bool throughAir = false; //!< Whether the path is one which moves through air.
}; 

}//namespace cura

#endif//PATH_PLANNING_COMB_PATHS_H
