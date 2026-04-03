// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_FIXEDGCODEPART_H
#define GCODEEXPORT_FIXEDGCODEPART_H

#include <sstream>

#include "gcode_export/GCodePart.h"

namespace cura
{

/*! \brief Contains pieces of GCode that are fixed string */
class FixedGCodePart : public GCodePart
{
public:
    explicit FixedGCodePart();

    /*! \brief Gets the full piece of GCode to be exported */
    std::string str() const override;

    /*! \brief Gets the actual stream in which the GCode parts can be stored */
    std::ostringstream& stream();

private:
    std::ostringstream stream_;
};

} // namespace cura

#endif
