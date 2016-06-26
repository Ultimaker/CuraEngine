/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "GCodeExport.hpp"

#include "Point3.hpp"

namespace mason {

GCodeExport::GCodeExport()
{
    m_gcode_out = NULL;
}

void GCodeExport::setGCodeExporter(cura::GCodeExport *gcode_out)
{
    m_gcode_out = gcode_out;
}

void GCodeExport::preSetup(const MeshGroup* settings)
{
    m_gcode_out->preSetup(settings);
}

void GCodeExport::writeComment(std::string comment)
{
    m_gcode_out->writeComment(comment);
}

void GCodeExport::writeCode(const char* str)
{
    m_gcode_out->writeCode(str);
}

void GCodeExport::writeMove(Point3 p, double speed, double extrusion_per_mm)
{
    m_gcode_out->writeMove(p.toCuraPoint3(),speed,extrusion_per_mm);
}

}
