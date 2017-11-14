//Copyright (c) 2013 Ultimaker
//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_H
#define INFILL_H

#include "utils/polygon.h"
#include "settings/settings.h"
#include "infill/ZigzagConnectorProcessor.h"
#include "infill/NoZigZagConnectorProcessor.h"
#include "infill/SubDivCube.h"
#include "infill/SpaceFillingTreeFill.h"
#include "utils/intpoint.h"
#include "utils/AABB.h"

namespace cura
{

class Infill 
{
    static constexpr int perimeter_gaps_extra_offset = 15; // extra offset so that the perimeter gaps aren't created everywhere due to rounding errors

    EFillMethod pattern; //!< the space filling pattern of the infill to generate
    bool zig_zaggify; //!< Whether to connect the end pieces of the support lines via the wall
    const Polygons& in_outline; //!< a reference polygon for getting the actual area within which to generate infill (see outline_offset)
    coord_t outline_offset; //!< Offset from Infill::in_outline to get the actual area within which to generate infill
    coord_t infill_line_width; //!< The line width of the infill lines to generate
    coord_t line_distance; //!< The distance between two infill lines / polygons
    coord_t infill_overlap; //!< the distance by which to overlap with the actual area within which to generate infill
    double fill_angle; //!< for linear infill types: the angle of the infill lines (or the angle of the grid)
    coord_t z; //!< height of the layer for which we generate infill
    coord_t shift; //!< shift of the scanlines in the direction perpendicular to the fill_angle
    const Point infill_origin; //!< origin of the infill pattern
    Polygons* perimeter_gaps; //!< (optional output) The areas in between consecutive insets when Concentric infill is used.
    bool connected_zigzags; //!< (ZigZag) Whether endpieces of zigzag infill should be connected to the nearest infill line on both sides of the zigzag connector
    bool use_endpieces; //!< (ZigZag) Whether to include endpieces: zigzag connector segments from one infill line to itself
    bool skip_some_zags;  //!< (ZigZag) Whether to skip some zags
    int zag_skip_count;  //!< (ZigZag) To skip one zag in every N if skip some zags is enabled
    bool apply_pockets_alternatingly; //!< Whether to add pockets to the cross 3d pattern only at half the intersections of the fractal
    coord_t pocket_size; //!< The size of the pockets at the intersections of the fractal in the cross 3d pattern

    static constexpr double one_over_sqrt_2 = 0.7071067811865475244008443621048490392848359376884740; //!< 1.0 / sqrt(2.0)
public:
    /*!
     * \warning If \p perimeter_gaps is given, then the difference between the \p in_outline
     * and the polygons which result from offsetting it by the \p outline_offset
     * and then expanding it again by half the \p infill_line_width
     * is added to the \p perimeter_gaps
     * 
     * \param[out] perimeter_gaps (optional output) The areas in between consecutive insets when Concentric infill is used.
     */
    Infill(EFillMethod pattern
        , bool zig_zaggify
        , const Polygons& in_outline
        , int outline_offset
        , int infill_line_width
        , int line_distance
        , int infill_overlap
        , double fill_angle
        , int64_t z
        , int64_t shift
        , const Point& infill_origin = Point()
        , Polygons* perimeter_gaps = nullptr
        , bool connected_zigzags = false
        , bool use_endpieces = false
        , bool skip_some_zags = false
        , int zag_skip_count = 0
        , bool apply_pockets_alternatingly = false
        , coord_t pocket_size = 0
    )
    : pattern(pattern)
    , zig_zaggify(zig_zaggify)
    , in_outline(in_outline)
    , outline_offset(outline_offset)
    , infill_line_width(infill_line_width)
    , line_distance(line_distance)
    , infill_overlap(infill_overlap)
    , fill_angle(fill_angle)
    , z(z)
    , shift(shift)
    , infill_origin(infill_origin)
    , perimeter_gaps(perimeter_gaps)
    , connected_zigzags(connected_zigzags)
    , use_endpieces(use_endpieces)
    , skip_some_zags(skip_some_zags)
    , zag_skip_count(zag_skip_count)
    , apply_pockets_alternatingly(apply_pockets_alternatingly)
    , pocket_size(pocket_size)
    {
    }

    /*!
     * Generate the infill.
     * 
     * \param result_polygons (output) The resulting polygons (from concentric infill)
     * \param result_lines (output) The resulting line segments (from linear infill types)
     * \param mesh The mesh for which to generate infill (should only be used for non-helper objects)
     * \param[in] cross_fill_pattern Where the cross fractal precomputation is stored
     */
    void generate(Polygons& result_polygons, Polygons& result_lines, const SpaceFillingTreeFill* cross_fill_pattern = nullptr, const SliceMeshStorage* mesh = nullptr);

private:
    /*!
     * Generate sparse concentric infill
     * 
     * Also adds \ref Inifll::perimeter_gaps between \ref Infill::in_outline and the first wall
     * 
     * \param result (output) The resulting polygons
     * \param inset_value The offset between each consecutive two polygons
     */
    void generateConcentricInfill(Polygons& result, int inset_value);

    /*!
     * Generate sparse concentric infill starting from a specific outer wall
     * \param first_wall The outer wall from which to start
     * \param result (output) The resulting polygons
     * \param inset_value The offset between each consecutive two polygons
     */
    void generateConcentricInfill(Polygons& first_wall, Polygons& result, int inset_value);

    /*!
     * Generate sparse concentric infill 
     * \param[out] result (output) The resulting lines
     */
    void generateConcentric3DInfill(Polygons& result);

    /*!
     * Generate a rectangular grid of infill lines
     * \param[out] result (output) The resulting lines
     */
    void generateGridInfill(Polygons& result);

    /*!
     * Generate a shifting triangular grid of infill lines, which combine with consecutive layers into a cubic pattern
     * \param[out] result (output) The resulting lines
     */
    void generateCubicInfill(Polygons& result);

    /*!
     * Generate a double shifting square grid of infill lines, which combine with consecutive layers into a tetrahedral pattern
     * \param[out] result (output) The resulting lines
     */
    void generateTetrahedralInfill(Polygons& result);

    /*!
     * Generate a double shifting square grid of infill lines, which combine with consecutive layers into a quarter cubic pattern
     * \param[out] result (output) The resulting lines
     */
    void generateQuarterCubicInfill(Polygons& result);

    /*!
     * Generate a single shifting square grid of infill lines.
     * This is used in tetrahedral infill (Octet infill) and in Quarter Cubic infill.
     * 
     * \param pattern_z_shift The amount by which to shift the whole pattern down
     * \param angle_shift The angle to add to the infill_angle
     * \param[out] result (output) The resulting lines
     */
    void generateHalfTetrahedralInfill(float pattern_z_shift, int angle_shift, Polygons& result);

    /*!
     * Generate a triangular grid of infill lines
     * \param[out] result (output) The resulting lines
     */
    void generateTriangleInfill(Polygons& result);

    /*!
     * Generate a triangular grid of infill lines
     * \param[out] result (output) The resulting lines
     */
    void generateTrihexagonInfill(Polygons& result);

    /*!
     * Generate a 3d pattern of subdivided cubes on their points
     * \param[out] result The resulting lines
     * \param[in] mesh Where the Cubic Subdivision Infill precomputation is stored
     */
    void generateCubicSubDivInfill(Polygons& result, const SliceMeshStorage& mesh);

    /*!
     * Generate a 3d pattern of subdivided cubes on their points
     * \param[in] cross_fill_pattern Where the cross fractal precomputation is stored
     * \param[out] result_polygons The resulting polygons
     * \param[out] result_lines The resulting lines
     */
    void generateCrossInfill(const SpaceFillingTreeFill& cross_fill_pattern, Polygons& result_polygons, Polygons& result_lines);

    /*!
     * Convert a mapping from scanline to line_segment-scanline-intersections (\p cut_list) into line segments, using the even-odd rule
     * \param[out] result (output) The resulting lines
     * \param rotation_matrix The rotation matrix (un)applied to enforce the angle of the infill 
     * \param scanline_min_idx The lowest index of all scanlines crossing the polygon
     * \param line_distance The distance between two lines which are in the same direction
     * \param boundary The axis aligned boundary box within which the polygon is
     * \param cut_list A mapping of each scanline to all y-coordinates (in the space transformed by rotation_matrix) where the polygons are crossing the scanline
     * \param total_shift total shift of the scanlines in the direction perpendicular to the fill_angle.
     */
    void addLineInfill(Polygons& result, const PointMatrix& rotation_matrix, const int scanline_min_idx, const int line_distance, const AABB boundary, std::vector<std::vector<int64_t>>& cut_list, int64_t total_shift);

    /*!
     * Crop line segments by the infill polygon using Clipper
     * \param[out] result (output) The resulting lines
     * \param input The line segments to be cropped
     */
    void addLineSegmentsInfill(Polygons& result, Polygons& input);

    /*!
     * generate lines within the area of \p in_outline, at regular intervals of \p line_distance
     * 
     * idea:
     * intersect a regular grid of 'scanlines' with the area inside \p in_outline
     * 
     * \param[out] result (output) The resulting lines
     * \param line_distance The distance between two lines which are in the same direction
     * \param infill_rotation The angle of the generated lines
     * \param extra_shift extra shift of the scanlines in the direction perpendicular to the infill_rotation
     */
    void generateLineInfill(Polygons& result, int line_distance, const double& infill_rotation, int64_t extra_shift);
    
    /*!
     * Function for creating linear based infill types (Lines, ZigZag).
     * 
     * This function implements the basic functionality of Infill::generateLineInfill (see doc of that function),
     * but makes calls to a ZigzagConnectorProcessor which handles what to do with each line segment - scanline intersection.
     * 
     * It is called only from Infill::generateLineinfill and Infill::generateZigZagInfill.
     * 
     * \param outline_offset An offset from the reference polygon (Infill::in_outline) to get the actual outline within which to generate infill
     * \param[out] result (output) The resulting lines
     * \param line_distance The distance between two lines which are in the same direction
     * \param rotation_matrix The rotation matrix (un)applied to enforce the angle of the infill 
     * \param zigzag_connector_processor The processor used to generate zigzag connectors
     * \param connected_zigzags Whether to connect the endpiece zigzag segments on both sides to the same infill line
     * \param extra_shift extra shift of the scanlines in the direction perpendicular to the fill_angle
     */
    void generateLinearBasedInfill(const int outline_offset, Polygons& result, const int line_distance, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags, int64_t extra_shift);

    /*!
     * 
     * generate lines within the area of [in_outline], at regular intervals of [line_distance]
     * idea:
     * intersect a regular grid of 'scanlines' with the area inside [in_outline] (see generateLineInfill)
     * zigzag:
     * include pieces of boundary, connecting the lines, forming an accordion like zigzag instead of separate lines    |_|^|_|
     * 
     * Note that ZigZag consists of 3 types:
     * - without endpieces
     * - with disconnected endpieces
     * - with connected endpieces
     * 
     *     <--
     *     ___
     *    |   |   |
     *    |   |   |
     *    |   |___|
     *         -->
     * 
     *        ^ = even scanline
     *  ^            ^ no endpieces
     * 
     * start boundary from even scanline! :D
     * 
     * 
     *                 v  disconnected end piece: leave out last line segment
     *          _____
     *   |     |     |  \                     .
     *   |     |     |  |
     *   |_____|     |__/
     * 
     *   ^     ^     ^    scanlines
     * 
     * 
     *                 v  connected end piece
     *          ________
     *   |     |     |  \                      .
     *   |     |     |  |
     *   |_____|     |__/                       .
     * 
     *   ^     ^     ^    scanlines
     * 
     * \param[out] result (output) The resulting lines
     * \param line_distance The distance between two lines which are in the same direction
     * \param infill_rotation The angle of the generated lines
     */
    void generateZigZagInfill(Polygons& result, const int line_distance, const double& infill_rotation);

    /*!
     * determine how far the infill pattern should be shifted based on the values of infill_origin and \p infill_rotation
     *
     * \param[in] infill_rotation the angle the infill pattern is rotated through
     *
     * \return the distance the infill pattern should be shifted
     */
    int64_t getShiftOffsetFromInfillOriginAndRotation(const double& infill_rotation);
};

}//namespace cura

#endif//INFILL_H
