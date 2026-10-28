// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PolylineStitcher.h"
#include "utils/ExtrusionLine.h"

namespace curaengine
{

using ExtrusionLineStitcher = PolylineStitcher<VariableWidthLines, VariableWidthLines, ExtrusionLine, ExtrusionJunction>;

} // namespace curaengine
