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
        ColourUsage color_usage;
        Settings(SettingsBaseVirtual* settings_base)
        : layer_height(settings_base->getSettingInMicrons("layer_height"))
        , point_distance(settings_base->getSettingInMicrons("bump_map_point_dist"))
        , amplitude(settings_base->getSettingInMicrons("bump_map_amplitude"))
        , offset(settings_base->getSettingInMicrons("bump_map_offset"))
        , alternate(settings_base->getSettingBoolean("bump_map_alternate"))
        , face_angle_correction(settings_base->getSettingAsRatio("bump_map_face_angle_correction"))
        , max_tan_correction_angle(std::tan(0.5 * M_PI - settings_base->getSettingInAngleRadians("bump_map_angle_correction_min")))
        , color_usage(settings_base->getSettingAsColourUsage("bump_map_texture_color"))
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

    /*!
     * Instructions on how to handle a corner.
     * A corner which has an outward offset on both sides is a detoured corner,
     * which will be offsetted in the average direction of both normals with the average offset.
     *                   ,.-~^~-.,                                        .
     *                  /    ,    \                                       .
     *                 /    / \    \                                      .
     *                /    /   \    \                                     .
     *               /    /     \    \ offsetted                          .
     *                   original
     * 
     * Other corner types (shortcutting on both sides or on either side)
     * are offsetted toward the location where parallel lines cross which are offsetted by the given amounts from the
     * line segments connected to the corner.
     * 
     * The amount by which they shortcut the line segments is recorded as the disregard distance.
     * Locations within the disregard distance on a line segment to a corner don't need to be offsetted.
     * Negative disregard distances are maxed to zero: no piece of the original line segment needs to be disregarded.
     * 
     *                                                                              offsetted
     *                                                                              |   original
     *                          |   |                                               |   |
     *                          |   |                                               |   |
     *                          |   |                                               |   |
     *                          |...|_______offsetted                               |___|_______offsetted
     *        disregard dist  { |   :                             disregard dist  {    :|
     *                        { |___:_______original                              {    :|_______original
     *                           vvv                                                 vvv
     *                         disregard dist                                      disregard dist = 0
     */
    struct CornerHandle
    {
        Point corner_offset_vector;
        bool is_detour_corner;
        coord_t prev_segment_disregard_distance;
        coord_t next_segment_disregard_distance;
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
     * Get how to handle variable offsetting of a corner.
     * The result contains information on whether it is a detour corner or not.
     * (Only corners with outward offsets on both line segments are considered detour corners)
     * The result contains information on how much of the line segments before and after are not used in the result polygon
     * 
     * \param p0 The first point of the first line segment
     * \param p1 The corner point shared by the two line segments
     * \param p2 The second point of the second line segment
     * \param textured_face_slice From which to determine the offset at the end of the first line segment - or default to zero
     * \param next_textured_face_slice From which to determine the offset at the start of the second line segment - or default to zero
     * \return Parameters which determine how to handle variable offsetting a corner
     */
    CornerHandle getCornerHandle(Point p0, Point p1, Point p2, std::optional<TextureBumpMapProcessor::TexturedFaceSlice>& textured_face_slice, std::optional<TextureBumpMapProcessor::TexturedFaceSlice>& next_textured_face_slice);

    /*!
     * Get how to handle variable offsetting of a corner.
     * The result contains information on whether it is a detour corner or not.
     * (Only corners with outward offsets on both line segments are considered detour corners)
     * The result contains information on how much of the line segments before and after are not used in the result polygon
     * 
     * \param p0 The first point of the first line segment
     * \param p1 The corner point shared by the two line segments
     * \param p2 The second point of the second line segment
     * \param offset0 THe offset used on the segment from \p p0 to \p p1 in the direction of the CW normal
     * \param offset1 THe offset used on the segment from \p p1 to \p p2 in the direction of the CW normal
     * \return Parameters which determine how to handle variable offsetting a corner
     */
    static CornerHandle getCornerHandle(Point p0, Point p1, Point p2, coord_t offset0, coord_t offset1);

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