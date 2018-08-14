//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cstring> //For strtok and strcopy.
#include <fstream> //To check if files exist.
#include <libgen.h> //To get the parent directory of a file path.
#include <numeric> //For std::accumulate.
#include <omp.h> //To change the number of threads to slice with.
#include <rapidjson/rapidjson.h>
#include <rapidjson/error/en.h> //Loading JSON documents to get settings from them.
#include <rapidjson/filereadstream.h>
#include <unordered_set>

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
void CommandLine::setLayerForSend(const LayerIndex& layer_nr) { }

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
                            if (loadJSON(argument, *last_settings))
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

int CommandLine::loadJSON(const std::string& json_filename, Settings& settings)
{
    FILE* file = fopen(json_filename.c_str(), "rb");
    if (!file)
    {
        logError("Couldn't open JSON file.\n");
        return 1;
    }

    rapidjson::Document json_document;
    char read_buffer[4096];
    rapidjson::FileReadStream reader_stream(file, read_buffer, sizeof(read_buffer));
    json_document.ParseStream(reader_stream);
    fclose(file);
    if (json_document.HasParseError())
    {
        logError("Error parsing JSON (offset %u): %s\n", static_cast<unsigned int>(json_document.GetErrorOffset()), GetParseError_En(json_document.GetParseError()));
        return 2;
    }

    std::unordered_set<std::string> search_directories; //For finding the inheriting JSON files.
    char filename_copy[json_filename.size()];
    std::strcpy(filename_copy, json_filename.c_str());
    std::string directory = std::string(dirname(filename_copy));
    search_directories.emplace(directory);

    return loadJSON(json_document, search_directories, settings);
}

std::unordered_set<std::string> CommandLine::defaultSearchDirectories()
{
    std::unordered_set<std::string> result;

    char* paths = getenv("CURA_ENGINE_SEARCH_PATH");
    if (paths)
    {
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
        char delims[] = ":"; //Colon for Unix.
#else
        char delims[] = ";"; //Semicolon for Windows.
#endif
        char* path = strtok(paths, delims);
        while (path != nullptr)
        {
            result.emplace(path);
            path = strtok(nullptr, ";:,"); //Continue searching in last call to strtok.
        }
    }

    return result;
}

int CommandLine::loadJSON(const rapidjson::Document& document, const std::unordered_set<std::string>& search_directories, Settings& settings)
{
    //Inheritance from other JSON documents.
    if (document.HasMember("inherits") && document["inherits"].IsString())
    {
        std::string parent_file = findDefinitionFile(document["inherits"].GetString(), search_directories);
        if (parent_file == "")
        {
            logError("Inherited JSON file \"%s\" not found.\n", document["inherits"].GetString());
            return 1;
        }
        int error_code = loadJSON(parent_file, settings); //Head-recursively load the settings file that we inherit from.
        if (error_code)
        {
            return error_code;
        }
    }

    //Extruders defined from here, if any.
    //Note that this always puts the extruder settings in the slice of the current extruder. It doesn't keep the nested structure of the JSON files, if extruders would have their own sub-extruders.
    Slice& slice = Application::getInstance().current_slice;
    if (document.HasMember("metadata") && document["metadata"].IsObject())
    {
        const rapidjson::Value& metadata = document["metadata"];
        if (metadata.HasMember("machine_extruder_trains") && metadata["machine_extruder_trains"].IsObject())
        {
            const rapidjson::Value& extruder_trains = metadata["machine_extruder_trains"];
            for (rapidjson::Value::ConstMemberIterator extruder_train = extruder_trains.MemberBegin(); extruder_train != extruder_trains.MemberEnd(); extruder_train++)
            {
                const int extruder_nr = atoi(extruder_train->name.GetString());
                if (extruder_nr < 0)
                {
                    continue;
                }
                while (slice.scene.extruders.size() <= static_cast<size_t>(extruder_nr))
                {
                    slice.scene.extruders.emplace_back(slice.scene.extruders.size(), &slice.scene.settings);
                }
                const rapidjson::Value& extruder_id = extruder_train->value;
                if (!extruder_id.IsString())
                {
                    continue;
                }
                const std::string extruder_definition_id(extruder_id.GetString());
                const std::string extruder_file = findDefinitionFile(extruder_definition_id, search_directories);
                loadJSON(extruder_file, slice.scene.extruders[extruder_nr].settings);
            }
        }
    }

    if (document.HasMember("settings") && document["settings"].IsObject())
    {
        loadJSONSettings(document["settings"], settings);
    }
    if (document.HasMember("overrides") && document["overrides"].IsObject())
    {
        loadJSONSettings(document["overrides"], settings);
    }
    return 0;
}

void CommandLine::loadJSONSettings(const rapidjson::Value& element, Settings& settings)
{
    for (rapidjson::Value::ConstMemberIterator setting = element.MemberBegin(); setting != element.MemberEnd(); setting++)
    {
        const std::string name = setting->name.GetString();

        const rapidjson::Value& setting_object = setting->value;
        if (!setting_object.IsObject())
        {
            logError("JSON setting %s is not an object!\n", name.c_str());
            continue;
        }

        if (setting_object.HasMember("children"))
        {
            loadJSONSettings(setting_object["children"], settings);
        }
        else //Only process leaf settings. We don't process categories or settings that have sub-settings.
        {
            if (!setting_object.HasMember("default_value"))
            {
                logWarning("JSON setting %s has no default_value!\n", name.c_str());
            }
            const rapidjson::Value& default_value = setting_object["default_value"];
            std::string value_string;
            if (default_value.IsString())
            {
                value_string = default_value.GetString();
            }
            else if (default_value.IsTrue())
            {
                value_string = "true";
            }
            else if (default_value.IsFalse())
            {
                value_string = "false";
            }
            else if (default_value.IsNumber())
            {
                std::ostringstream ss;
                ss << default_value.GetDouble();
                value_string = ss.str();
            }
            else
            {
                logWarning("Unrecognized data type in JSON setting %s\n", name.c_str());
                continue;
            }
            settings.add(name, value_string);
        }
    }
}

const std::string CommandLine::findDefinitionFile(const std::string& definition_id, const std::unordered_set<std::string>& search_directories)
{
    for (const std::string& search_directory : search_directories)
    {
        const std::string candidate = search_directory + std::string("/") + definition_id + std::string(".def.json");
        const std::ifstream ifile(candidate.c_str()); //Check whether the file exists and is readable by opening it.
        if (ifile)
        {
            return candidate;
        }
    }
    return std::string("");
}

} //namespace cura