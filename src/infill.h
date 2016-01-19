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
    EFillMethod pattern;
    const Polygons& in_outline;
    int outlineOffset;
    bool avoidOverlappingPerimeters;
    int infill_line_width;
    int line_distance;
    double infill_overlap;
    double fill_angle;
    PointMatrix rotation_matrix;
    bool connected_zigzags;
    bool use_endPieces;

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
    , rotation_matrix(fill_angle)
    , connected_zigzags(connected_zigzags)
    , use_endPieces(use_endPieces)
    {
    }
    void generate(Polygons& result_polygons, Polygons& result_lines, Polygons* in_between);

private:

    void generateConcentricInfill(Polygons outline, Polygons& result, int inset_value);

    void generateConcentricInfillDense(Polygons outline, Polygons& result, Polygons* in_between, int infill_line_width, bool avoidOverlappingPerimeters);

    void generateGridInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int infill_line_width, int lineSpacing, double infillOverlap, double rotation);

    void generateTriangleInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int infill_line_width, int lineSpacing, double infillOverlap, double rotation);
    
    /*!
     * Convert a mapping from scanline to line_segment-scanline-intersections (\p cutList) into line segments, using the even-odd rule
     */
    static void addLineInfill(Polygons& result, const PointMatrix& matrix, const int scanline_min_idx, const int lineSpacing, const AABB boundary, std::vector<std::vector<int64_t>>& cutList, const int infill_line_width);

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
     */
    void generateLineInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int infill_line_width, int lineSpacing, double infillOverlap, const PointMatrix& rotation_matrix);
    
    /*!
     * Function for creating linear based infill types (Lines, ZigZag).
     * 
     * This function implements the basic functionality of Infill::generateLineInfill (see doc of that function),
     * but makes calls to a ZigzagConnectorProcessor which handles what to do with each line segment - scanline intersection.
     * 
     * It is called only from Infill::generateLineinfill and Infill::generateZigZagInfill.
     */
    void generateLinearBasedInfill(const Polygons& in_outline, const int outlineOffset, Polygons& result, const int infill_line_width, const int lineSpacing, const double infillOverlap, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags);

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
     */
    void generateZigZagInfill(const Polygons& in_outline, Polygons& result, const int infill_line_width, const int lineSpacing, const double infillOverlap, const PointMatrix& rotation_matrix, const bool connected_zigzags, const bool use_endPieces);
};

}//namespace cura

#endif//INFILL_H
