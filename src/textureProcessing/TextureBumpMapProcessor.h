/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H
#define TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H

#include <vector>
#include <math.h> // tan

#include "../utils/polygon.h"
#include "../utils/optional.h"
#include "../utils/SparsePointGrid.h"

#include "../settings/settings.h"

#include "../slicer/SlicerSegment.h"
#include "TexturedMesh.h"
#include "FaceNormalStorage.h"

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
        coord_t layer_height;
        coord_t point_distance;
        coord_t amplitude;
        coord_t offset;
        bool alternate;
        float face_angle_correction;
        float max_tan_correction_angle;
        Settings(SettingsBaseVirtual* settings_base)
        : layer_height(settings_base->getSettingInMicrons("layer_height"))
        , point_distance(settings_base->getSettingInMicrons("bump_map_point_dist"))
        , amplitude(settings_base->getSettingInMicrons("bump_map_amplitude"))
        , offset(settings_base->getSettingInMicrons("bump_map_offset"))
        , alternate(settings_base->getSettingBoolean("bump_map_alternate"))
        , face_angle_correction(settings_base->getSettingAsRatio("bump_map_face_angle_correction"))
        , max_tan_correction_angle(std::tan(0.5 * M_PI - settings_base->getSettingInAngleRadians("bump_map_angle_correction_min")))
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
    TextureBumpMapProcessor(TexturedMesh* mesh, const Settings settings, FaceNormalStorage* face_normal_storage);

    /*!
     * Process the texture bump map.
     * Change the polygons in a layer
     * 
     * \warning Where no texture is present, no offset is applied to the outer boundary!
     * Such segments are copied to the result as is
     * 
     * \param[in,out] layer_polygons The polygons to be offsetted by texture color values
     * \param layer_nr The layer nr for which we are processing the bump map
     */
    void processBumpMap(Polygons& layer_polygons, unsigned int layer_nr);

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

    TexturedMesh* mesh;

    /*!
     * The settings with which to \ref TextureBumpMapProcessor::processBumpMap
     */
    Settings settings;

    /*!
     * The face normal statistics to correct offsets for slanted faces - if provided
     * 
     * This is stored as a pointer so that the default assignment operator = can be defined automatically.
     */
    FaceNormalStorage* face_normal_storage;

    /*!
     * A grid to efficiently look op which texture segment best fits the slicer segment.
     */
    SparseGrid<TexturedFaceSlice> loc_to_slice;

    /*!
     * Get the offset to be applied at a given location
     */
    coord_t getOffset(const float color, const int face_idx);

    /*!
     * Get the offset to be applied at a given corner
     * 
     * Computes the average offset from the end of \p textured_face_slice and start of \p next_textured_face_slice
     * If either of those is not present, the \ref TextureBumpMapProcessor::Settings::default_color is used for that segment
     * 
     * \warning Where no texture is present, no offset is applied to the outer boundary!
     * 
     * \param textured_face_slice From which to determine the offset at the end of the line segment - or default to zero
     * \param next_textured_face_slice From which to determine the offset at the start of the line segment - or default to zero
     */
    coord_t getCornerOffset(std::optional<TextureBumpMapProcessor::TexturedFaceSlice>& textured_face_slice, std::optional<TextureBumpMapProcessor::TexturedFaceSlice>& next_textured_face_slice);

    /*!
     * Get how much of a corner to skip generating offsetted indices for inner corners,
     * because those points would be removed by the offset itseld
     */
    coord_t getCornerDisregard(Point p0, Point p1, Point p2, std::optional<TexturedFaceSlice>& textured_face_slice, std::optional<TexturedFaceSlice>& next_textured_face_slice);

    /*!
     * Get the TexturedFaceSlice corresponding to an outline segment
     * 
     * Note that due to snapping in the \ref Slicer::makePolygons function, an outline segment may be a bit different from the originally sliced SlicerSegment
     * 
     * \param p0 The start of the segment
     * \param p1 The end of the segment
     */
    std::optional<TexturedFaceSlice> getTexturedFaceSlice(Point p0, Point p1);

    /*!
     * 
     * \param layer_nr The layer number for which we process the bump map
     * \param slicer_segment The segment closest matching \p p0 - \p p1
     * \param corner_disregard_p1 The distance at the end of p0p1 in which not to place offsetted points
     */
    void processSegmentBumpMap(unsigned int layer_nr, const SlicerSegment& slicer_segment, const MatSegment& mat, const Point p0, const Point p1, coord_t& dist_left_over, coord_t corner_disregard_p1, PolygonRef result);
};

} // namespace cura

#endif // TEXTURE_PROCESSING_TEXTURE_BUMP_MAP_PROCESSOR_H