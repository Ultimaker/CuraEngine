// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PolylineStitcher.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

using ExtrusionLineStitcher = PolylineStitcher<VariableWidthLines, VariableWidthLines, ExtrusionLine, ExtrusionJunction>;

} // namespace cura
