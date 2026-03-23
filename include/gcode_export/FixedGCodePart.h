// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_FIXEDGCODEPART_H
#define GCODEEXPORT_FIXEDGCODEPART_H

#include <sstream>

#include "gcode_export/GCodePart.h"

namespace cura
{

class FixedGCodePart : public GCodePart
{
public:
    explicit FixedGCodePart();

    std::string str() const override;

    std::ostringstream& stream();

private:
    std::ostringstream stream_;
};

} // namespace cura

#endif
