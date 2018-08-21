//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffProcessor.h"
#include "Application.h" //To send the layer data through the communication channel.
#include "communication/Communication.h" //To send the layer data through the communication channel.

namespace cura 
{

FffProcessor FffProcessor::instance; // definition must be in cpp

FffProcessor::FffProcessor()
: polygon_generator(this)
{
}

std::string FffProcessor::getAllSettingsString(MeshGroup& meshgroup, bool first_meshgroup)
{
    std::stringstream sstream;
    if (first_meshgroup)
    {
        sstream << Application::getInstance().current_slice->scene.settings.getAllSettingsString(); //Global settings.
        sstream << " -g";
    }
    else 
    {
        sstream << " --next";
    }
    sstream << meshgroup.settings.getAllSettingsString();
    const Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_nr = 0; extruder_nr < scene.extruders.size(); extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];
        sstream << " -e" << extruder_nr << train.settings.getAllSettingsString();
    }
    for (unsigned int mesh_idx = 0; mesh_idx < meshgroup.meshes.size(); mesh_idx++)
    {
        Mesh& mesh = meshgroup.meshes[mesh_idx];
        sstream << " -e" << mesh.settings.get<size_t>("extruder_nr") << " -l \"" << mesh_idx << "\"" << mesh.settings.getAllSettingsString();
    }
    sstream << "\n";
    return sstream.str();
}

} // namespace cura 