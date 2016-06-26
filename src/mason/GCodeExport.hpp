/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_GCODE_EXPORT_HPP
#define INCLUDED_MASON_GCODE_EXPORT_HPP

#include "../gcodeExport.h"

#include "Mesh.hpp"
#include "Point3.hpp"

namespace mason {

class GCodeExport {
public:
    GCodeExport();

    void setGCodeExporter(cura::GCodeExport *gcode_out);
    
    void preSetup(const MeshGroup* settings);
    void writeComment(std::string comment);
    void writeCode(const char* str);
    void writeMove(Point3 p, double speed, double extrusion_per_mm);
private:
    cura::GCodeExport *m_gcode_out;
};

}

#endif
