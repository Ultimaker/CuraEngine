// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffProcessor.h"

namespace curaengine
{

FffProcessor FffProcessor::instance; // definition must be in cpp

void FffProcessor::finalize()
{
    gcode_writer.finalize();
}

} // namespace curaengine
