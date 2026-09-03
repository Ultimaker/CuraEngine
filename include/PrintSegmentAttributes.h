// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PRINT_SEGMENT_FLAG
#define PRINT_SEGMENT_FLAG

#include <cstdint>

#include "utils/Flags.h"

namespace cura
{

/*!
 * Print attributes are flags that can be added to some print segments to indicate that they have been processed a specific way,
 * e.g. by using overhanging or bridging settings. They are used for display purposes.
 * This enumeration has an equivalent in Cura/cura/PrintSegmentAttributes.py
 */
enum class PrintSegmentAttribute : uint8_t
{
    // clang-format off
    None        = 0b00,
    Overhanging = 0b01,
    Bridging    = 0b10,
    // clang-format on
};

using PrintSegmentAttributes = Flags<PrintSegmentAttribute>;

} // namespace cura

#endif
