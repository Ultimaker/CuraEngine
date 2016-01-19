/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INFILL_H
#define INFILL_H

#include "utils/polygon.h"
#include "settings.h"
#include "ZigzagConnectorProcessor.h"
#include "utils/intpoint.h"
#include "utils/AABB.h"

namespace cura
{

class Infill 
{
    EFillMethod pattern; //!< the space filling pattern of the infill to generate
    const Polygons& in_outline; //!< a reference polygon for getting the actual area within which to generate infill (see outline_offset)
    int outlineOffset; //!< Offset from Infill::in_outline to get the actual area within which to generate infill
    bool avoidOverlappingPerimeters; //!< Whether to remove overlapping perimeter parts
    int infill_line_width; //!< The line width of the infill lines to generate
    int line_distance; //!< The distance between two infill lines / polygons
    double infill_overlap; //!< the percentage (of infill_line_width) to overlap with the actual area within which to generate infill
    double fill_angle; //!< for linear infill types: the angle of the infill lines (or the angle of the grid)
    bool connected_zigzags; //!< (ZigZag) Whether endpieces of zigzag infill should be connected to the nearest infill line on both sides of the zigzag connector
    bool use_endPieces; //!< (ZigZag) Whether to include endpieces: zigzag connector segments from one infill line to itself

public:
    Infill(EFillMethod pattern, const Polygons& in_outline, int outlineOffset, bool avoidOverlappingPerimeters, int infill_line_width, int line_distance, double infill_overlap, double fill_angle, bool connected_zigzags = false, bool use_endPieces = false)
    : pattern(pattern)
    , in_outline(in_outline)
    , outlineOffset(outlineOffset)
    , avoidOverlappingPerimeters(avoidOverlappingPerimeters)
    , infill_line_width(infill_line_width)
    , line_distance(line_distance)
    , infill_overlap(infill_overlap)
    , fill_angle(fill_angle)
    , connected_zigzags(connected_zigzags)
    , use_endPieces(use_endPieces)
    {
    }
    /*!
     * Generate the infill.
     * 
     * \param result_polygons (output) The resulting polygons (from concentric infill)
     * \param result_lines (output) The resulting line segments (from linear infill types)
     * \param in_between (optional output) The areas in between two concecutive concentric infill polygons
     */
    void generate(Polygons& result_polygons, Polygons& result_lines, Polygons* in_between);

private:

    /*!
     * Generate sparse concentric infill 
     * \param outline The actual outline of the area within which to generate infill
     * \param result (output) The resulting polygons
     * \param inset_value The offset between each consecutive two polygons
     */
    void generateConcentricInfill(Polygons outline, Polygons& result, int inset_value);

    /*!
     * Generate dense concentric infill (100%)
     * 
     * \param outline The actual outline of the area within which to generate infill
     * \param result (output) The resulting polygons
     * \param in_between (output) The areas in between each two consecutive polygons
     * \param avoidOverlappingPerimeters Whether to remove overlapping perimeter parts
     */
    void generateConcentricInfillDense(Polygons outline, Polygons& result, Polygons* in_between, bool avoidOverlappingPerimeters);

    /*!
     * Generate a rectangular grid of infill lines
     * \param in_outline A reference polygon for creating the actual outline in which to generate infill
     * \param outlineOffset An offset from the reference polygon (@p in_outline) to get the actual outline within which to generate infill
     * \param result (output) The resulting lines
     */
    void generateGridInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int lineSpacing, double rotation);

    /*!
     * Generate a triangular grid of infill lines
     * \param in_outline A reference polygon for creating the actual outline in which to generate infill
     * \param outlineOffset An offset from the reference polygon (@p in_outline) to get the actual outline within which to generate infill
     * \param result (output) The resulting lines
     */
    void generateTriangleInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int lineSpacing, double rotation);
    
    /*!
     * Convert a mapping from scanline to line_segment-scanline-intersections (\p cutList) into line segments, using the even-odd rule
     * \param result (output) The resulting lines
     * \param rotation_matrix The rotation matrix (un)applied to enforce the angle of the infill 
     * \param scanline_min_idx The lowest index of all scanlines crossing the polygon
     * \param lineSpacing The distance between two lines which are in the same direction
     * \param boundary The axis aligned boundary box within which the polygon is
     * \param cutList A mapping of each scanline to all y-coordinates (in the space transformed by rotation_matrix) where the polygons are crossing the scanline
     */
    void addLineInfill(Polygons& result, const PointMatrix& rotation_matrix, const int scanline_min_idx, const int lineSpacing, const AABB boundary, std::vector<std::vector<int64_t>>& cutList);

    /*!
     * generate lines within the area of \p in_outline, at regular intervals of \p lineSpacing
     * 
     * idea:
     * intersect a regular grid of 'scanlines' with the area inside \p in_outline
     * 
     * we call the areas between two consecutive scanlines a 'scansegment'.
     * Scansegment x is the area between scanline x and scanline x+1
     * 
     * algorithm:
     * 1) for each line segment of each polygon:
     *      for each scanline crossing that line segment:
     *          store the intersections of that line segment with the scanlines in a mapping (vector of vectors) from scanline to intersections
     * 2) for each scanline:
     *      sort the associated intersections 
     *      connect them using the even-odd rule and generate a line for each
     * 
     * \param in_outline A reference polygon for creating the actual outline in which to generate infill
     * \param outlineOffset An offset from the reference polygon (@p in_outline) to get the actual outline within which to generate infill
     * \param result (output) The resulting lines
     * \param lineSpacing The distance between two lines which are in the same direction
     * \param fill_angle The angle of the generated lines
     */
    void generateLineInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int lineSpacing, const double& fill_angle);
    
    /*!
     * Function for creating linear based infill types (Lines, ZigZag).
     * 
     * This function implements the basic functionality of Infill::generateLineInfill (see doc of that function),
     * but makes calls to a ZigzagConnectorProcessor which handles what to do with each line segment - scanline intersection.
     * 
     * It is called only from Infill::generateLineinfill and Infill::generateZigZagInfill.
     * 
     * \param in_outline A reference polygon for creating the actual outline in which to generate infill
     * \param outlineOffset An offset from the reference polygon (@p in_outline) to get the actual outline within which to generate infill
     * \param result (output) The resulting lines
     * \param lineSpacing The distance between two lines which are in the same direction
     * \param rotation_matrix The rotation matrix (un)applied to enforce the angle of the infill 
     * \param zigzag_connector_processor The processor used to generate zigzag connectors
     * \param connected_zigzags Whether to connect the endpiece zigzag segments on both sides to the same infill line
     */
    void generateLinearBasedInfill(const Polygons& in_outline, const int outlineOffset, Polygons& result, const int lineSpacing, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags);

    /*!
     * 
     * generate lines within the area of [in_outline], at regular intervals of [lineSpacing]
     * idea:
     * intersect a regular grid of 'scanlines' with the area inside [in_outline] (see generateLineInfill)
     * zigzag:
     * include pieces of boundary, connecting the lines, forming an accordion like zigzag instead of separate lines    |_|^|_|
     * 
     * we call the areas between two consecutive scanlines a 'scansegment'
     * 
     * algorithm:
     * 1. for each line segment of each polygon:
     *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
     *      (zigzag): add boundary segments to result
     * 2. for each scanline:
     *      sort the associated intersections 
     *      and connect them using the even-odd rule
     * 
     * zigzag algorithm:
     * while walking around (each) polygon (1.)
     *  if polygon intersects with even scanline
     *      start boundary segment (add each following segment to the [result])
     *  when polygon intersects with a scanline again
     *      stop boundary segment (stop adding segments to the [result])
     *      if polygon intersects with even scanline again (instead of odd)
     *          dont add the last line segment to the boundary (unless [connected_zigzags])
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
     *          _____
     *   |     |     |  \                     .
     *   |     |     |  |
     *   |_____|     |__/
     * 
     *   ^     ^     ^    scanlines
     *                 ^  disconnected end piece: leave out last line segment
     *          ________
     *   |     |     |  \                      .
     *   |     |     |  |
     *   |_____|     |__/                       .
     * 
     *   ^     ^     ^    scanlines
     *                 ^  connected end piece
     * 
     * \param in_outline The actual outline in which to generate infill
     * \param result (output) The resulting lines
     * \param lineSpacing The distance between two lines which are in the same direction
     * \param fill_angle The angle of the generated lines
     * \param connected_zigzags Whether to connect the endpiece zigzag segments on both sides to the same infill line
     * \param use_endPieces Whether to include zigzag segments connecting a scanline to itself
     */
    void generateZigZagInfill(const Polygons& in_outline, Polygons& result, const int lineSpacing, const double& fill_angle, const bool connected_zigzags, const bool use_endPieces);
};

}//namespace cura

#endif//INFILL_H
