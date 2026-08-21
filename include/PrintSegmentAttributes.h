// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PRINT_SEGMENT_FLAG
#define PRINT_SEGMENT_FLAG

#include <cstdint>

#include "utils/Flags.h"

namespace cura
{

enum class PrintSegmentAttribute : uint8_t
{
    None = 0,
    Overhanging = 0x1,
    Bridging = 0x2,
};

using PrintSegmentAttributes = Flags<PrintSegmentAttribute>;

} // namespace cura

#endif
