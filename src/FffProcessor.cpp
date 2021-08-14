//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffProcessor.h"

namespace cura 
{

FffProcessor FffProcessor::instance; // definition must be in cpp

bool FffProcessor::setTargetFile(const char* filename)
{
    return gcode_writer.setTargetFile(filename);
}

void FffProcessor::setTargetStream(std::ostream* stream)
{
    return gcode_writer.setTargetStream(stream);
}

double FffProcessor::getTotalFilamentUsed(int extruder_nr)
{
    return gcode_writer.getTotalFilamentUsed(extruder_nr);
}

std::vector<Duration> FffProcessor::getTotalPrintTimePerFeature()
{
    return gcode_writer.getTotalPrintTimePerFeature();
}

void FffProcessor::finalize()
{
    gcode_writer.finalize();
}

} // namespace cura 
