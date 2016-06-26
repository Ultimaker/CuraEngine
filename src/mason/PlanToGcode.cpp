/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "PlanToGcode.hpp"

namespace mason {

PlanToGcode::PlanToGcode()
{
}

void PlanToGcode::process(const SettingsBaseVirtual *settings, const PrintPlan *plan, GCodeExport *gcode_out)
{
   writeHeader(settings,gcode_out);

   writePlan(plan, gcode_out);
   
   writeFooter(settings,gcode_out);
}

void PlanToGcode::writeHeader(const SettingsBaseVirtual *settings, GCodeExport *gcode_out)
{
   gcode_out->writeComment("Created by Mason backend.");
   
   gcode_out->writeCode(settings->getSettingString("machine_start_gcode").c_str());
}

void PlanToGcode::writePlan(const PrintPlan *plan, GCodeExport *gcode_out)
{
    size_t num_elems = plan->numElements();
    for (size_t elem_idx=0U; elem_idx!=num_elems; ++elem_idx) {
        plan->getElement(elem_idx)->addGcode(gcode_out);
    }
}

void PlanToGcode::writeFooter(const SettingsBaseVirtual *settings, GCodeExport *gcode_out)
{
   // Most of footer is written by FffGcodeWriter::finalize()
   // which is called by FffProcessor::finalize().
}

}
