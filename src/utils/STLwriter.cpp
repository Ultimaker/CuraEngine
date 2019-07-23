//Copyright (c) 2018 Ultimaker B.V.


#include "STLwriter.h"

namespace arachne {



STLwriter::STLwriter(std::string filename, double scaler)
: out(filename)
, scaler(scaler)
{
    out << "solid STLwriter\n";
}


STLwriter::~STLwriter()
{
    out << "endsolid STLwriter\n";
    out.close();
}

void STLwriter::writeTriangle(Point3 a, Point3 b, Point3 c)
{
    a *= scaler;
    b *= scaler;
    c *= scaler;
    out << "facet normal 1 0 0\n";
    out << "    outer loop\n";
    out << "        vertex " << INT2MM(a.x) << " " << INT2MM(a.y) << " " << INT2MM(a.z) << "\n";
    out << "        vertex " << INT2MM(b.x) << " " << INT2MM(b.y) << " " << INT2MM(b.z) << "\n";
    out << "        vertex " << INT2MM(c.x) << " " << INT2MM(c.y) << " " << INT2MM(c.z) << "\n";
    out << "    endloop\n";
    out << "endfacet\n";
}

} // namespace arachne 
