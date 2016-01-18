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
    int extrusion_width;
    int line_distance;
    double infill_overlap;
    double fill_angle;
    PointMatrix rotation_matrix;
    bool connected_zigzags;
    bool use_endPieces;

public:
    Infill(EFillMethod pattern, const Polygons& in_outline, int outlineOffset, bool avoidOverlappingPerimeters, int extrusion_width, int line_distance, double infill_overlap, double fill_angle, bool connected_zigzags = false, bool use_endPieces = false)
    : pattern(pattern)
    , in_outline(in_outline)
    , outlineOffset(outlineOffset)
    , avoidOverlappingPerimeters(avoidOverlappingPerimeters)
    , extrusion_width(extrusion_width)
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
        
        

    static void generateInfill(EFillMethod pattern, const Polygons& in_outline, const int outlineOffset, Polygons& result_polygons, Polygons& result_lines, Polygons* in_between, const bool avoidOverlappingPerimeters, const int extrusion_width, const int line_distance, const double infill_overlap, const double fill_angle, const bool connected_zigzags, const bool use_endPieces);

    static void generateConcentricInfill(Polygons outline, Polygons& result, int inset_value);

    static void generateConcentricInfillDense(Polygons outline, Polygons& result, Polygons* in_between, int extrusionWidth, bool avoidOverlappingPerimeters);

    static void generateGridInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);

    static void generateTriangleInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);
    
    static void addLineInfill(Polygons& result, const PointMatrix& matrix, const int scanline_min_idx, const int lineSpacing, const AABB boundary, std::vector<std::vector<int64_t>>& cutList, const int extrusionWidth);

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
     *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
     *      (zigzag): add boundary segments to result
     * 2) for each scanline:
     *      sort the associated intersections 
     *      and connect them using the even-odd rule
     * 
     */
    static void generateLineInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, const PointMatrix& rotation_matrix);
    
    static void generateLinearBasedInfill(const Polygons& in_outline, const int outlineOffset, Polygons& result, const int extrusionWidth, const int lineSpacing, const double infillOverlap, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags);

    static void generateZigZagInfill(const Polygons& in_outline, Polygons& result, const int extrusionWidth, const int lineSpacing, const double infillOverlap, const PointMatrix& rotation_matrix, const bool connected_zigzags, const bool use_endPieces);

    /*!
     * adapted from generateLineInfill(.)
     * 
     * generate lines within the area of [in_outline], at regular intervals of [lineSpacing]
     * idea:
     * intersect a regular grid of 'scanlines' with the area inside [in_outline]
     * sigzag:
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
     * 
     *     <--
     *     ___
     *    |   |   |
     *    |   |   |
     *    |   |___|
     *         -->
     * 
     *        ^ = even scanline
     * 
     * start boundary from even scanline! :D
     * 
     * 
     *          _____
     *   |     |     | ,
     *   |     |     |  |
     *   |_____|     |__/
     * 
     *   ^     ^     ^    scanlines
     *                 ^  disconnected end piece
     */
    static void generateZigZagIninfill_endPieces(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation, bool connected_zigzags);
    static void generateZigZagIninfill_noEndPieces(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);
    
};

}//namespace cura

#endif//INFILL_H
