//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <numeric> //For std::accumulate.

#include "CommandLine.h"
#include "../Application.h" //To get the extruders for material estimates.
#include "../FffProcessor.h" //To get time/material estimates.

namespace cura
{

CommandLine::CommandLine(const std::vector<std::string>& arguments)
: arguments(arguments)
, last_shown_progress(0)
{
}

//These are not applicable to command line slicing.
void CommandLine::beginGCode() { }
void CommandLine::flushGCode() { }
void CommandLine::sendCurrentPosition(const Point& position) { }
void CommandLine::sendFinishedSlicing() const { }
void CommandLine::sendLayerComplete(const LayerIndex& layer_nr, const coord_t& z, const coord_t& thickness) { }
void CommandLine::sendLineTo(const PrintFeatureType& type, const Point& to, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) { }
void CommandLine::sendOptimizedLayerData() { }
void CommandLine::sendPolygon(const PrintFeatureType& type, const ConstPolygonRef& polygon, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) { }
void CommandLine::sendPolygons(const PrintFeatureType& type, const Polygons& polygons, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) { }
void CommandLine::setExtruderForSend(const ExtruderTrain& extruder) { }

const bool CommandLine::hasSlice() const
{
    return !arguments.empty();
}

const bool CommandLine::isSequential() const
{
    return true; //We have to receive the g-code in sequential order. Start g-code before the rest and so on.
}

void CommandLine::sendGCodePrefix(const std::string& prefix) const
{
    std::cout << prefix << std::endl;
}

void CommandLine::sendPrintTimeMaterialEstimates() const
{
    std::vector<double> time_estimates = FffProcessor::getInstance()->getTotalPrintTimePerFeature();
    double sum = std::accumulate(time_estimates.begin(), time_estimates.end(), 0.0);
    log("Total print time: %5.3fs\n", sum);

    sum = 0.0;
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice.scene.settings.get<size_t>("machine_extruder_count"); extruder_nr++)
    {
        sum += FffProcessor::getInstance()->getTotalFilamentUsed(extruder_nr);
    }
}

void CommandLine::sendProgress(const float& progress) const
{
    const unsigned int rounded_amount = 100 * progress;
    if (last_shown_progress == rounded_amount) //No need to send another tiny update step.
    {
        return;
    }
}

void CommandLine::sliceNext()
{
    //TODO.
}

} //namespace cura