/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_PLAN_TO_GCODE_HPP
#define INCLUDED_MASON_PLAN_TO_GCODE_HPP

#include "GCodeExport.hpp"
#include "Settings.hpp"
#include "PrintPlan.hpp"

namespace mason {

class PlanToGcode {
public:
    PlanToGcode();

    void process(const SettingsBaseVirtual *settings, const PrintPlan *plan, GCodeExport *gcode_out);

private:
    void writeHeader(const SettingsBaseVirtual *settings, GCodeExport *gcode_out);
    void writePlan(const PrintPlan *plan, GCodeExport *gcode_out);
    void writeFooter(const SettingsBaseVirtual *settings, GCodeExport *gcode_out);
};

}

#endif
