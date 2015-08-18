/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef INFILL_H
#define INFILL_H

#include "utils/polygon.h"

namespace cura {

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
