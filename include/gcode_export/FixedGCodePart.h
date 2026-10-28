// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <sstream>

#include "gcode_export/GCodePart.h"

namespace curaengine::gcode_export
{

/*! \brief Contains pieces of GCode that are fixed string */
class FixedGCodePart : public GCodePart
{
public:
    explicit FixedGCodePart(const GCodePartType type);

    /*! \brief Gets the full piece of GCode to be exported */
    std::string str() const override;

    /*! \brief Gets the actual stream in which the GCode parts can be stored */
    std::ostringstream& stream();

private:
    std::ostringstream stream_;
};

} // namespace curaengine::gcode_export
