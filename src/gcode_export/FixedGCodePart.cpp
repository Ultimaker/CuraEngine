// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcode_export/FixedGCodePart.h"


namespace cura
{

FixedGCodePart::FixedGCodePart(const bool print_code)
    : GCodePart(print_code)
{
    stream_ << std::fixed;
}

std::string FixedGCodePart::str() const
{
    return stream_.str();
}

std::ostringstream& FixedGCodePart::stream()
{
    return stream_;
}

} // namespace cura
