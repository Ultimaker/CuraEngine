//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATH_PLANNING_COMB_PATH_H
#define PATH_PLANNING_COMB_PATH_H

#include "../utils/IntPoint.h"

namespace cura
{

class CombPath : public  std::vector<Point> //!< A single path either inside or outise the parts
{
public:
    bool cross_boundary = false; //!< Whether the path crosses a boundary.
};

}//namespace cura

#endif//PATH_PLANNING_COMB_PATH_H
