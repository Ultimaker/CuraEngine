/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSOR_H
#define TEXTURE_PROCESSOR_H

#include <vector>

#include "slicer/Slicer.h"
#include "mesh.h"

namespace cura
{

class TextureProcessor
{
public:
//     static void process(std::vector<Slicer*>& slicer_list);
    static void processBumpMap(const Mesh* mesh, SlicerLayer& layer);
protected:

    static void processSegmentBumpMap(const Mesh* mesh, const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, PolygonRef result);
};

} // namespace cura

#endif // TEXTURE_PROCESSOR_H