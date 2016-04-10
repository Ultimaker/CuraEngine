/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef SLICER_SLICER_LAYER_H
#define SLICER_SLICER_LAYER_H

#include <unordered_map>

#include "../mesh.h"
#include "../utils/intpoint.h"
#include "../utils/polygon.h"

#include "SlicerSegment.h"
#include "GapCloserResult.h"
#include "ClosePolygonResult.h"


namespace cura
{


class SlicerLayer
{
public:
    std::vector<SlicerSegment> segmentList;
    std::unordered_map<int, int> face_idx_to_segment_index; // topology

    int z;
    Polygons polygonList;
    Polygons openPolylines;

    void makePolygons(Mesh* mesh, bool keepNoneClosed, bool extensiveStitching);

private:
    GapCloserResult findPolygonGapCloser(Point ip0, Point ip1);

    ClosePolygonResult findPolygonPointClosestTo(Point input);

};

} // namespace cura

#endif // SLICER_SLICER_LAYER_H