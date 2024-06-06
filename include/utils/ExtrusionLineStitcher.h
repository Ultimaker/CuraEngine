// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_EXTRUSION_LINE_STITCHER_H
#define UTILS_EXTRUSION_LINE_STITCHER_H

#include "PolylineStitcher.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

using ExtrusionLineStitcher = PolylineStitcher<VariableWidthLines, VariableWidthLines, ExtrusionLine, ExtrusionJunction>;

} // namespace cura
#endif // UTILS_EXTRUSION_LINE_STITCHER_H
