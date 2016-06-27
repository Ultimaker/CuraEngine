/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_BUILD_PLAN_HPP
#define INCLUDED_BUILD_PLAN_HPP

#include "PrintPlan.hpp"
#include "VolumeStore.hpp"
#include "WirePlan.hpp"

namespace mason {

struct BuildPlan {
public:
    VolumeStore target;
    VolumeStore volume_plan;
    WirePlan wire_plan;
    PrintPlan print_plan;
};

}

#endif
