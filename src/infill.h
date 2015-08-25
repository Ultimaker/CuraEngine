/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INFILL_H
#define INFILL_H

#include "utils/polygon.h"
#include "settings.h"

namespace cura {

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
    bool connect_zigzags;
    bool use_endPieces;

public:
    Infill(EFillMethod pattern, const Polygons& in_outline, int outlineOffset, bool avoidOverlappingPerimeters, int extrusion_width, int line_distance, double infill_overlap, double fill_angle, bool connect_zigzags, bool use_endPieces)
    : pattern(pattern)
    , in_outline(in_outline)
    , outlineOffset(outlineOffset)
    , avoidOverlappingPerimeters(avoidOverlappingPerimeters)
    , extrusion_width(extrusion_width)
    , line_distance(line_distance)
    , infill_overlap(infill_overlap)
    , fill_angle(fill_angle)
    , connect_zigzags(connect_zigzags)
    , use_endPieces(use_endPieces)
    {
    }
    void generate(Polygons& result_polygons, Polygons& result_lines, Polygons* in_between);
};
    
void generateInfill(EFillMethod pattern, const Polygons& in_outline, int outlineOffset, Polygons& result_polygons, Polygons& result_lines, Polygons* in_between, bool avoidOverlappingPerimeters, int extrusion_width, int line_distance, double infill_overlap, double fill_angle, bool connect_zigzags, bool use_endPieces);
void generateConcentricInfill(Polygons outline, Polygons& result, int inset_value);
void generateConcentricInfillDense(Polygons outline, Polygons& result, Polygons* in_between, int extrusionWidth, bool avoidOverlappingPerimeters);
void generateGridInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);
void generateTriangleInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);
void generateLineInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);
void generateZigZagInfill(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation, bool connect_zigzags, bool use_endPieces);
void generateZigZagIninfill_endPieces(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation, bool connect_zigzags);
void generateZigZagIninfill_noEndPieces(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, double rotation);
}//namespace cura

#endif//INFILL_H
