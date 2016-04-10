/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICER_SLICER_H
#define SLICER_SLICER_H

#include "../mesh.h"
#include "../utils/polygon.h"

#include "SlicerSegment.h"
#include "ClosePolygonResult.h"
#include "SlicerLayer.h"

#include "../MatSegment.h"

/*
    The Slicer creates layers of polygons from an optimized 3D model.
    The result of the Slicer is a list of polygons without any order or structure.
*/
namespace cura {

class Slicer
{
public:
    std::vector<SlicerLayer> layers;
    
    Slicer(Mesh* mesh, int initial, int thickness, int layer_count, bool keepNoneClosed, bool extensiveStitching);

    void dumpSegmentsToHTML(const char* filename);

protected:
    
    Mesh* mesh;
    int layer_height_0;
    int layer_height;
    
    /*!
     * Create a SlicerSegment along the lines going through p0p1 (Start) and p0p2 (End)
     * 
     * \warning \p p0 may not have the same z as either \p p1 or \p p2
     * 
     * \param p The face vertice locations in the order the vertices are given in the face
     */
    SlicerSegment project2D(unsigned int face_idx, Point3 p[3], unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z) const;
};

}//namespace cura

#endif//SLICER_SLICER_H
