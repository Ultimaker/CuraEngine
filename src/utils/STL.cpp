/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#include "STL.h"
#include "floatpoint.h"

namespace cura {



STL::STL(const std::string filename_str)
{
    strcpy(filename, filename_str.c_str());
    
    out = fopen(filename, "w");
    if(!out)
    {
        logError("The file %s could not be opened for writing.",filename);
    }
    printf("solid CuraEngineSTLWriter\n");
}

void STL::writeFace(FPoint3 a, FPoint3 b, FPoint3 c)
{
    if ((a - b).vSize2() < INT2MM(10) * INT2MM(10)
        || (a - c).vSize2() < INT2MM(10) * INT2MM(10)
        || (b - c).vSize2() < INT2MM(10) * INT2MM(10))
    {
        return;
    }
    printf("facet normal 0.0 0.0 0.0\n");
    printf("    outer loop\n");
    printf("        vertex %f %f %f\n", a.x, a.y, a.z);
    printf("        vertex %f %f %f\n", b.x, b.y, b.z);
    printf("        vertex %f %f %f\n", c.x, c.y, c.z);
    printf("    endloop\n");
    printf("endfacet\n");
}

STL::~STL()
{
    printf("endsolid CuraEngineSTLWriter\n");
    fclose(out);
    logAlways("Written STL output to '%s'.\n", filename);
}

} // namespace cura 
