/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_PLAN_TO_GCODE_HPP
#define INCLUDED_MASON_PLAN_TO_GCODE_HPP

#include "GCodeExport.hpp"
#include "PrintPlan.hpp"
#include "Settings.hpp"

namespace mason {

class PlanToGcode {
public:
    PlanToGcode();

    void process(const SettingsStore *settings, const PrintPlan *plan, GCodeExport *gcode_out);

private:
    void writeHeader();
    void writePlan();
    void writeFooter();

    const SettingsStore *m_settings;
    const PrintPlan *m_plan;
    GCodeExport *m_gcode_out;
};

}

#endif
