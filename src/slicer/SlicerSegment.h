/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef SLICER_SLICER_SEGMENT_H
#define SLICER_SLICER_SEGMENT_H

#include "../utils/intpoint.h"

namespace cura
{

class SlicerSegment
{
public:
    Point start, end;
    int faceIndex;
    bool addedToPolygon;
};

} // namespace cura

#endif // SLICER_SLICER_SEGMENT_H