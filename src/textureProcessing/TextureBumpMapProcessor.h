/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H
#define TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H

#include <vector>

#include "../utils/polygon.h"
#include "../utils/optional.h"
#include "../utils/SparsePointGrid.h"

#include "../settings/settings.h"

#include "../slicer/SlicerSegment.h"
#include "TexturedMesh.h"

namespace cura
{

class TextureBumpMapProcessor
{
public:
    /*!
     * Helper class to retrieve and store texture to bump map settings
     */
    struct Settings
    {
        coord_t point_distance;
        coord_t amplitude;
        coord_t offset;
        Settings(SettingsBaseVirtual* settings_base)
        : point_distance(settings_base->getSettingInMicrons("bump_map_point_dist"))
        , amplitude(settings_base->getSettingInMicrons("bump_map_amplitude"))
        , offset(settings_base->getSettingInMicrons("bump_map_offset"))
        {
        }
    };
    /*!
     * default constructor
     * 
     * initializes the \ref SparseGrid::cell_size of \ref TextureBumpMapProcessor::loc_to_slice
     * 
     * \param settings The settings with which to \ref TextureBumpMapProcessor::processBumpMap
     */
    TextureBumpMapProcessor(const Settings settings);

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
    void registerTexturedFaceSlice(SlicerSegment face_segment, MatSegment texture_segment);
protected:
    /*!
     * A sliced segment in combination with the corresponding texture slice.
     */
    struct TexturedFaceSlice
    {
        SlicerSegment face_segment;
        MatSegment mat_segment;
    };

    /*!
     * The settings with which to \ref TextureBumpMapProcessor::processBumpMap
     */
    Settings settings;

    /*!
     * A grid to efficiently look op which texture segment best fits the slicer segment.
     */
    SparseGrid<TexturedFaceSlice> loc_to_slice;

    /*!
     * Get the TexturedFaceSlice corresponding to an outline segment
     * 
     * Note that due to snapping in the \ref Slicer::makePolygons function, an outline segment may be a bit different from the originally sliced SlicerSegment
     * 
     * \param p0 The start of the segment
     * \param p1 The end of the segment
     */
    std::optional<TexturedFaceSlice> getTexturedFaceSlice(Point p0, Point p1);

    void processSegmentBumpMap(const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, PolygonRef result);
};

} // namespace cura

#endif // TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H