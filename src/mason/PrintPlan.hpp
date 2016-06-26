/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_PRINT_PLAN_HPP
#define INCLUDED_MASON_PRINT_PLAN_HPP

#include "GCodeExport.hpp"
#include "Point3.hpp"

namespace mason {

class PrintPlanElement {
public:
    virtual void addGcode(GCodeExport *gcode_out) = 0;
};

class HeadMove : public PrintPlanElement {
public:
    HeadMove(Point3 to,double speed);

    virtual void addGcode(GCodeExport *gcode_out);

private:
    Point3 m_to;
    double m_speed;
};

class ExtrudedSegment : public PrintPlanElement {
public:
    ExtrudedSegment(Point3 from, Point3 to, double speed, float mm3_per_mm);
    
    virtual void addGcode(GCodeExport *gcode_out);

private:
    Point3 m_from;
    Point3 m_to;
    double m_speed;
    float m_mm3_per_mm;
};

class PrintPlan {
public:
    PrintPlan();

    void addElement(std::shared_ptr<PrintPlanElement> element);

    size_t numElements() const;
    const std::shared_ptr<PrintPlanElement> &getElement(size_t elem_idx) const;
    
private:
    std::vector<std::shared_ptr<PrintPlanElement> > m_plan_elements;
};

}

#endif
