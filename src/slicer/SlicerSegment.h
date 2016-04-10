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
    int faceIndex = -1;
    // The index of the other face connected via the edge that created end
    int endOtherFaceIdx = -1;
    // If end corresponds to a vertex of the mesh, then this is populated
    // with the vertex that it ended on.
    const MeshVertex *endVertex = nullptr;
    bool addedToPolygon = false;
};

} // namespace cura

#endif // SLICER_SLICER_SEGMENT_H