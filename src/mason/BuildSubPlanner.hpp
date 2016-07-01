/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_BUILD_SUB_PLANNER_HPP
#define INCLUDED_MASON_BUILD_SUB_PLANNER_HPP

#include "BuildPlan.hpp"

namespace mason {

class BuildSubPlanner {
public:
    virtual ~BuildSubPlanner() {}

    virtual void process(BuildPlan *buildPlan) = 0;
};

}

#endif
