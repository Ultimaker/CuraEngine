/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_SKIRT_PLANNER_HPP
#define INCLUDED_SKIRT_PLANNER_HPP

#include "BuildSubPlanner.hpp"

namespace mason {

class SkirtPlanner : public BuildSubPlanner {
public:
    virtual ~SkirtPlanner();

    virtual void process(BuildPlan *build_plan);
};

}

#endif
