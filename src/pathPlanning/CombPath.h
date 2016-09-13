/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef PATH_PLANNING_COMB_PATH_H
#define PATH_PLANNING_COMB_PATH_H

#include "../utils/intpoint.h"

namespace cura 
{

struct CombPath : public  std::vector<Point> //!< A single path either inside or outise the parts
{
    bool cross_boundary = false; //!< Whether the path crosses a boundary.
};

}//namespace cura

#endif//PATH_PLANNING_COMB_PATH_H
