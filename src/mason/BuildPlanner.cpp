/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "BuildPlanner.hpp"

#include "SkirtPlanner.hpp"
#include "Time.hpp"
#include "WirePlanSender.hpp"
#include "WireToPrintPlanner.hpp"

namespace mason {

void BuildPlanner::process(BuildPlan *build_plan)
{
    m_build_plan = build_plan;

    std::shared_ptr<BuildSubPlanner> sub_planner;

    m_sub_planners.emplace_back(new SkirtPlanner);
    //m_sub_planners.emplace_back(new WirePlanSender);
    m_sub_planners.emplace_back(new WireToPrintPlanner);

    size_t num_sub_planners = m_sub_planners.size();
    TimeKeeper part_timer;
    for (size_t sub_planner_idx=0U; sub_planner_idx!=num_sub_planners; ++sub_planner_idx) {
        m_sub_planners[sub_planner_idx]->process(m_build_plan);
        std::cout << m_sub_planners[sub_planner_idx]->getName() << " took " <<
            part_timer.restart() << " secs" << std::endl;
    }
}

}

