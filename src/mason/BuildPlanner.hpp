/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_BUILD_PLANNER_HPP
#define INCLUDED_MASON_BUILD_PLANNER_HPP

#include "BuildPlan.hpp"

namespace mason {

/** \brief Implements algorithm for going from target volume to tool path plan.
 */
class BuildPlanner {
public:
    /** \brief Create tool path plan to generate target volume.
     *
     * \param[in,out] build_plan "target" must be set in build_plan.  All other field will
     *    are considered output.
     */
    void process(BuildPlan *build_plan);

private:
    void createWirePlan();
    void createPrintPlan();

    BuildPlan *m_build_plan;
};

}

#endif
