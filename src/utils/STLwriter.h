//Copyright (c) 2018 Ultimaker B.V.


#ifndef STL_WRITER_H
#define STL_WRITER_H

#include <stdio.h> // for file output
#include <fstream> // for file output

#include "AABB.h"
#include "IntPoint.h"
#include "NoCopy.h"

namespace arachne
{


class STLwriter : NoCopy
{
public:

    std::ofstream out; // the output file

    double scaler;

    STLwriter(std::string filename, double scaler = 10.0);
    ~STLwriter();

    void writeTriangle(Point3 a, Point3 b, Point3 c);
};

} // namespace arachne
#endif // STL_WRITER_H
