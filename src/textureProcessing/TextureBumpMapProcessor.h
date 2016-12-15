/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H
#define TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H

#include <vector>

#include "../utils/polygon.h"

#include "../slicer/SlicerSegment.h"
#include "../mesh.h"

namespace cura
{

class TextureBumpMapProcessor
{
public:
    /*!
     * Process the texture bump map.
     * Change the polygons in a layer
     * 
     * \param mesh TODO
     * \param[in,out] layer_polygons The polygons to be offsetted by texture color values
     */
    void processBumpMap(const Mesh* mesh, Polygons& layer_polygons);

    std::unordered_map<SlicerSegment, MatSegment> segment_to_material_segment;
protected:

    void processSegmentBumpMap(const Mesh* mesh, const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, PolygonRef result);
};

} // namespace cura

#endif // TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H