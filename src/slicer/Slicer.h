/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SLICER_SLICER_H
#define SLICER_SLICER_H

#include <queue>

#include "../mesh.h"
#include "../utils/polygon.h"

#include "SlicerSegment.h"
#include "ClosePolygonResult.h"
#include "SlicerLayer.h"

#include "../textureProcessing/MatSegment.h"
#include "../textureProcessing/TextureProximityProcessor.h"

/*
    The Slicer creates layers of polygons from an optimized 3D model.
    The result of the Slicer is a list of polygons without any order or structure.
*/
namespace cura {

class Slicer
{
public:
    std::vector<SlicerLayer> layers;

    const Mesh* mesh = nullptr; //!< The sliced mesh

    const TexturedMesh* textured_mesh; //!< Pointer to the textured mesh if \ref Slicer::mesh is a TexturedMesh

    TextureProximityProcessor* texture_proximity_processor; //!< Containers for each layer for fast lookup of textures being defined in the proximity of the lookup point

    /*!
     * 
     * \param texture_proximity_processors (optional) A TextureProximityProcessor for all layers in the mesh
     */
    Slicer(Mesh* mesh, int initial, int thickness, unsigned int slice_layer_count, bool keepNoneClosed, bool extensiveStitching, TextureProximityProcessor* texture_proximity_processors);

    
    
    /*!
     * Linear interpolation
     *
     * Get the Y of a point with X \p x in the line through (\p x0, \p y0) and (\p x1, \p y1)
     * 
     * \param p The face vertice locations in the order the vertices are given in the face
     */
    int64_t interpolate(int64_t x, int64_t x0, int64_t x1, int64_t y0, int64_t y1) const
    {
        int64_t dx_01 = x1 - x0;
        int64_t num = (y1 - y0) * (x - x0);
        num += num > 0 ? dx_01/2 : -dx_01/2; // add in offset to round result
        int64_t y = y0 + num / dx_01;
        return y;
    }

    /*!
     * 
     * \warning this function requires result.faceIndex to be correctly set already
     * 
     * \p result where to store the start and end of the sliced segment
     */
    void project2D(unsigned int face_idx, const Point3 p[3], unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z, int32_t layer_nr, SlicerSegment& result);

    void dumpSegmentsToHTML(const char* filename);
};

}//namespace cura

#endif//SLICER_SLICER_H
