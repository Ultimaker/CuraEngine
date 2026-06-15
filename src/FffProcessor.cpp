// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffProcessor.h"

namespace cura
{

FffProcessor FffProcessor::instance; // definition must be in cpp

void FffProcessor::finalize()
{
    gcode_writer.finalize();
}

} // namespace cura
