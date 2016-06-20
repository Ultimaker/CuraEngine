/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "PrintPlan.hpp"

namespace cura {
namespace mason {

HeadMove::HeadMove(Point3 to, double speed) :
    m_to(to),
    m_speed(speed)
{
}

void HeadMove::addGcode(GCodeExport *gcode_out)
{
    gcode_out->writeMove(m_to, m_speed, 0.0);
}

ExtrudedSegment::ExtrudedSegment(Point3 from, Point3 to, double speed, float mm3_per_mm) :
    m_from(from),
    m_to(to),
    m_speed(speed),
    m_mm3_per_mm(mm3_per_mm)
{
}

void ExtrudedSegment::addGcode(GCodeExport *gcode_out)
{
    gcode_out->writeMove(m_from, m_speed, 0.0);
    gcode_out->writeMove(m_to, m_speed, m_mm3_per_mm);
}

PrintPlan::PrintPlan()
{
}

void PrintPlan::addElement(std::shared_ptr<PrintPlanElement> element)
{
    m_plan_elements.push_back(element);
}

size_t PrintPlan::numElements() const
{
    return m_plan_elements.size();
}

const std::shared_ptr<PrintPlanElement> &
PrintPlan::getElement(size_t elem_idx) const
{
    assert(elem_idx < m_plan_elements.size());
    
    return m_plan_elements[elem_idx];
}

}
}
