/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef SLICER_CLOSE_POLYGON_RESULT_H
#define SLICER_CLOSE_POLYGON_RESULT_H

#include "../utils/intpoint.h"

namespace cura
{

class ClosePolygonResult
{   //The result of trying to find a point on a closed polygon line. This gives back the point index, the polygon index, and the point of the connection.
    //The line on which the point lays is between pointIdx-1 and pointIdx
public:
    Point intersectionPoint;
    int polygonIdx;
    unsigned int pointIdx;
};

} // namespace cura

#endif // SLICER_CLOSE_POLYGON_RESULT_H