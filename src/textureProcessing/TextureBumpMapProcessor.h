/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H
#define TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H

#include <vector>

#include "../utils/polygon.h"

#include "../slicer/SlicerSegment.h"
#include "TexturedMesh.h"

namespace cura
{

class TextureBumpMapProcessor
{
public:
    /*!
     * Process the texture bump map.
     * Change the polygons in a layer
     * 
     * \param[in,out] layer_polygons The polygons to be offsetted by texture color values
     */
    void processBumpMap(Polygons& layer_polygons);

    /*!
     * Register that a particular face was sliced to a particular texture segment.
     * \param face_segment The geometrical segment of the face
     * \param texture_segment The corresponding texture coordinates
     */
    void registerTextureFaceSlice(SlicerSegment face_segment, MatSegment texture_segment);
protected:

    std::unordered_map<SlicerSegment, MatSegment> segment_to_material_segment;

    void processSegmentBumpMap(const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, PolygonRef result);
};

} // namespace cura

#endif // TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H