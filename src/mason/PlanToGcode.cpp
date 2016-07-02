/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "PlanToGcode.hpp"

namespace mason {

PlanToGcode::PlanToGcode()
{
}

void PlanToGcode::process(const SettingsStore *settings, const PrintPlan *plan, GCodeExport *gcode_out)
{
    m_settings = settings;
    m_plan = plan;
    m_gcode_out = gcode_out;
    
    writeHeader();
    writePlan();
    writeFooter();
}

void PlanToGcode::writeHeader()
{
   m_gcode_out->writeComment("Created by Mason backend.");
   
   m_gcode_out->writeCode(m_settings->getSettingString("machine_start_gcode").c_str());
}

void PlanToGcode::writePlan()
{
    size_t num_elems = m_plan->numElements();
    for (size_t elem_idx=0U; elem_idx!=num_elems; ++elem_idx) {
        m_plan->getElement(elem_idx)->addGcode(m_gcode_out);
    }
}

void PlanToGcode::writeFooter()
{
   // Most of footer is written by FffGcodeWriter::finalize()
   // which is called by FffProcessor::finalize().
}

}
