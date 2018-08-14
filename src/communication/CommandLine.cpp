//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <numeric> //For std::accumulate.
#include <omp.h> //To change the number of threads to slice with.

#include "CommandLine.h"
#include "../Application.h" //To get the extruders for material estimates.
#include "../FffProcessor.h" //To start a slice and get time estimates.
#include "../settings/SettingRegistry.h" //To load JSON files with setting definitions.

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
    //TODO: Right now this is done directly in the g-code writer. For consistency it should be moved here?
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
    FffProcessor::getInstance()->time_keeper.restart();

    Application::getInstance().current_slice.reset(); //Create a new Slice.
    Slice& slice = Application::getInstance().current_slice;

    MeshGroup* mesh_group = new MeshGroup(FffProcessor::getInstance());
    slice.scene.extruders.emplace_back(0, &slice.scene.settings); //Always have one extruder.
    Settings* last_settings = &mesh_group->settings;
    ExtruderTrain& last_extruder = slice.scene.extruders[0];

    for (size_t argument_index = 2; argument_index < arguments.size(); argument_index++)
    {
        std::string argument = arguments[argument_index];
        if (argument[0] == '-') //Starts with "-".
        {
            if (argument[1] == '-') //Starts with "--".
            {
                if (argument.find("--next") == 0) //Starts with "--next".
                {
                    try
                    {
                        log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());

                        const size_t extruder_count = slice.scene.settings.get<size_t>("machine_extruder_count");
                        while (slice.scene.extruders.size() < extruder_count)
                        {
                            slice.scene.extruders.emplace_back(slice.scene.extruders.size(), &slice.scene.settings);
                        }

                        mesh_group->finalize();

                        //Start slicing.
                        FffProcessor::getInstance()->processMeshGroup(mesh_group);

                        //Initialize loading of the next meshes.
                        FffProcessor::getInstance()->time_keeper.restart();
                        delete mesh_group;
                        mesh_group = new MeshGroup(FffProcessor::getInstance());
                        last_settings = &mesh_group->settings;
                    }
                    catch(...)
                    {
                        //Catch all exceptions.
                        //This prevents the "something went wrong" dialogue on Windows to pop up on a thrown exception.
                        //Only ClipperLib currently throws exceptions. And only in the case that it makes an internal error.
                        logError("Unknown exception!\n");
                                delete mesh_group;
                        exit(1);
                    }
                }
                else
                {
                    logError("Unknown option: %s\n", argument);
                }
            }
            else //Starts with "-" but not with "--".
            {
                for (argument_index++; argument_index < arguments.size(); argument_index++)
                {
                    argument = arguments[argument_index];
                    switch(argument[1])
                    {
                        case 'v':
                            increaseVerboseLevel();
                            break;
#ifdef _OPENMP
                        case 'm':
                        {
                            int threads = stoi(argument.substr(1));
                            threads = std::max(1, threads);
                            omp_set_num_threads(threads);
                            break;
                        }
#endif //_OPENMP
                        case 'p':
                            enableProgressLogging();
                            break;
                        case 'j':
                            argument_index++;
                            if (argument_index >= arguments.size())
                            {
                                logError("Missing JSON file with -j argument.");
                                delete mesh_group;
                                exit(1);
                            }
                            if (SettingRegistry::getInstance()->loadJSONsettings(argument, last_settings))
                            {
                                logError("Failed to load JSON file: %s\n", argument);
                                delete mesh_group;
                                exit(1);
                            }
                            break;
                        case 'e':
                        {
                            size_t extruder_nr = stoul(argument.substr(1));
                            while (slice.scene.extruders.size() <= extruder_nr) //Make sure we have enough extruders up to the extruder_nr that the user wanted.
                            {
                                slice.scene.extruders.emplace_back(extruder_nr, &slice.scene.settings);
                            }
                            last_settings = &slice.scene.extruders[extruder_nr].settings;
                            break;
                        }
                        case 'l':
                        {
                            argument_index++;
                            if (argument_index >= arguments.size())
                            {
                                logError("Missing model file with -l argument.");
                                delete mesh_group;
                                exit(1);
                            }

                            const FMatrix3x3 transformation = last_settings->get<FMatrix3x3>("mesh_rotation_matrix"); //The transformation applied to the model when loaded.

                            argument = arguments[argument_index];
                            if (!loadMeshIntoMeshGroup(mesh_group, argument.c_str(), transformation, last_extruder))
                            {
                                logError("Failed to load model: %s\n", argument);
                                delete mesh_group;
                                exit(1);
                            }
                            else
                            {
                                last_settings = &mesh_group->meshes.back().settings;
                            }
                            break;
                        }
                        case 'o':
                            argument_index++;
                            if (argument_index >= arguments.size())
                            {
                                logError("Missing output file with -o argument.");
                                delete mesh_group;
                                exit(1);
                            }
                            argument = arguments[argument_index];
                            if (!FffProcessor::getInstance()->setTargetFile(argument.c_str()))
                            {
                                logError("Failed to open %s for output.\n", argument);
                                delete mesh_group;
                                exit(1);
                            }
                            break;
                        case 'g':
                            last_settings = &mesh_group->settings;
                            //Fall-through is intended here. No break!
                        case 's':
                        {
                            //Parse the given setting and store it.
                            argument_index++;
                            if (argument_index >= arguments.size())
                            {
                                logError("Missing setting name and value with -s argument.");
                                delete mesh_group;
                                exit(1);
                            }
                            argument = arguments[argument_index];
                            const size_t value_position = argument.find("=");
                            std::string key = argument.substr(0, value_position);
                            if (value_position == std::string::npos)
                            {
                                logError("Missing value in setting argument: -s %s", argument);
                                delete mesh_group;
                                exit(1);
                            }
                            std::string value = argument.substr(value_position + 1);
                            last_settings->add(key, value);
                            break;
                        }
                        default:
                            logError("Unknown option: -%c\n", argument[1]);
                            Application::getInstance().printCall();
                            Application::getInstance().printHelp();
                            delete mesh_group;
                            exit(1);
                            break;
                    }
                }
            }
        }
        else
        {
            logError("Unknown option: %s\n", argument);
            Application::getInstance().printCall();
            Application::getInstance().printHelp();
            delete mesh_group;
            exit(1);
        }
    }

    arguments.clear(); //We've processed all arguments now.

#ifndef DEBUG
    try
    {
#endif //DEBUG
        mesh_group->finalize();
        log("Loaded from disk in %5.3fs\n", FffProcessor::getInstance()->time_keeper.restart());

        //Start slicing.
        FffProcessor::getInstance()->processMeshGroup(mesh_group);
#ifndef DEBUG
    }
    catch(...)
    {
        //Catch all exceptions.
        //This prevents the "something went wrong" dialogue on Windows to pop up on a thrown exception.
        //Only ClipperLib currently throws exceptions. And only in the case that it makes an internal error.
        logError("Unknown exception.\n");
        delete mesh_group;
        exit(1);
    }
#endif //DEBUG

    //Finalize the processor. This adds the end g-code and reports statistics.
    FffProcessor::getInstance()->finalize();

    delete mesh_group;
}

} //namespace cura