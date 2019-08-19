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
    if (a == b || b == c || c == a) return;
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


void STLwriter::writeQuad(Point3 low_a, Point3 high_a, Point3 low_b, Point3 high_b, coord_t discretization_dist)
{
    Point3 a_vec = high_a - low_a;
    Point3 b_vec = high_b - low_b;
    coord_t a_length = a_vec.vSize();
    coord_t b_length = b_vec.vSize();
    coord_t step_count = std::max(coord_t(1), std::max(a_length, b_length) / discretization_dist);
    Point3 a_prev = low_a;
    Point3 b_prev = low_b;
    for (coord_t step = 0; step < step_count; step++)
    {
        Point3 a_now = low_a + a_vec * (step + 1) / step_count;
        Point3 b_now = low_b + b_vec * (step + 1) / step_count;
        writeTriangle(a_prev, b_prev, a_now);
        writeTriangle(a_now, b_prev, b_now);
        a_prev = a_now;
        b_prev = b_now;
    }
}

} // namespace arachne 
