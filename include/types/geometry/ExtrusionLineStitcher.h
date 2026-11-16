// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/ExtrusionLine.h"
#include "types/geometry/PolylineStitcher.h"

namespace curaengine::types::geometry
{

using ExtrusionLineStitcher = PolylineStitcher<VariableWidthLines, VariableWidthLines, ExtrusionLine, ExtrusionJunction>;

} // namespace curaengine::types::geometry
