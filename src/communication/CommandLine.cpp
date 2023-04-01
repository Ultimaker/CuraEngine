// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <cstring> //For strtok and strcopy.
#include <errno.h> // error number when trying to read file
#include <filesystem>
#include <fstream> //To check if files exist.
#include <numeric> //For std::accumulate.
#include <unordered_set>

#include <rapidjson/error/en.h> //Loading JSON documents to get settings from them.
#include <rapidjson/filereadstream.h>
#include <rapidjson/rapidjson.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To get the extruders for material estimates.
#include "ExtruderTrain.h"
#include "FffProcessor.h" //To start a slice and get time estimates.
#include "Slice.h"
#include "communication/CommandLine.h"
#include "utils/FMatrix4x3.h" //For the mesh_rotation_matrix setting.

namespace cura
{

CommandLine::CommandLine(const std::vector<std::string>& arguments) : arguments(arguments), last_shown_progress(0)
{
}

// These are not applicable to command line slicing.
void CommandLine::beginGCode()
{
}
void CommandLine::flushGCode()
{
}
void CommandLine::sendCurrentPosition(const Point&)
{
}
void CommandLine::sendFinishedSlicing() const
{
}
void CommandLine::sendLayerComplete(const LayerIndex&, const coord_t&, const coord_t&)
{
}
void CommandLine::sendLineTo(const PrintFeatureType&, const Point&, const coord_t&, const coord_t&, const Velocity&)
{
}
void CommandLine::sendOptimizedLayerData()
{
}
void CommandLine::sendPolygon(const PrintFeatureType&, const ConstPolygonRef&, const coord_t&, const coord_t&, const Velocity&)
{
}
void CommandLine::sendPolygons(const PrintFeatureType&, const Polygons&, const coord_t&, const coord_t&, const Velocity&)
{
}
void CommandLine::setExtruderForSend(const ExtruderTrain&)
{
}
void CommandLine::setLayerForSend(const LayerIndex&)
{
}

bool CommandLine::hasSlice() const
{
    return ! arguments.empty();
}

bool CommandLine::isSequential() const
{
    return true; // We have to receive the g-code in sequential order. Start g-code before the rest and so on.
}

void CommandLine::sendGCodePrefix(const std::string&) const
{
    // TODO: Right now this is done directly in the g-code writer. For consistency it should be moved here?
}

void CommandLine::sendSliceUUID(const std::string& slice_uuid) const
{
    // pass
}

void CommandLine::sendPrintTimeMaterialEstimates() const
{
    std::vector<Duration> time_estimates = FffProcessor::getInstance()->getTotalPrintTimePerFeature();
    double sum = std::accumulate(time_estimates.begin(), time_estimates.end(), 0.0);
    spdlog::info("Total print time: {:3}", sum);

    sum = 0.0;
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
    {
        sum += FffProcessor::getInstance()->getTotalFilamentUsed(extruder_nr);
    }
}

void CommandLine::sendProgress(const float& progress) const
{
    const unsigned int rounded_amount = 100 * progress;
    if (last_shown_progress == rounded_amount) // No need to send another tiny update step.
    {
        return;
    }
    // TODO: Do we want to print a progress bar? We'd need a better solution to not have that progress bar be ruined by any logging.
}

void CommandLine::sliceNext()
{
    FffProcessor::getInstance()->time_keeper.restart();

    // Count the number of mesh groups to slice for.
    size_t num_mesh_groups = 1;
    for (size_t argument_index = 2; argument_index < arguments.size(); argument_index++)
    {
        if (arguments[argument_index].find("--next") == 0) // Starts with "--next".
        {
            num_mesh_groups++;
        }
    }
    Slice slice(num_mesh_groups);

    Application::getInstance().current_slice = &slice;

    size_t mesh_group_index = 0;
    Settings* last_settings = &slice.scene.settings;

    slice.scene.extruders.reserve(arguments.size() >> 1); // Allocate enough memory to prevent moves.
    slice.scene.extruders.emplace_back(0, &slice.scene.settings); // Always have one extruder.
    ExtruderTrain* last_extruder = &slice.scene.extruders[0];

    bool force_read_parent = false;
    bool force_read_nondefault = false;

    for (size_t argument_index = 2; argument_index < arguments.size(); argument_index++)
    {
        std::string argument = arguments[argument_index];
        if (argument[0] == '-') // Starts with "-".
        {
            if (argument[1] == '-') // Starts with "--".
            {
                if (argument.find("--next") == 0) // Starts with "--next".
                {
                    try
                    {
                        spdlog::info("Loaded from disk in {}", FffProcessor::getInstance()->time_keeper.restart());

                        mesh_group_index++;
                        FffProcessor::getInstance()->time_keeper.restart();
                        last_settings = &slice.scene.mesh_groups[mesh_group_index].settings;
                    }
                    catch (...)
                    {
                        // Catch all exceptions.
                        // This prevents the "something went wrong" dialogue on Windows to pop up on a thrown exception.
                        // Only ClipperLib currently throws exceptions. And only in the case that it makes an internal error.
                        spdlog::error("Unknown exception!");
                        exit(1);
                    }
                }
                else if (argument.find("--force-read-parent") == 0 || argument.find("--force_read_parent") == 0)
                {
                    spdlog::info("From this point on, force the parser to read values of non-leaf settings, instead of skipping over them as is proper.");
                    force_read_parent = true;
                }
                else if (argument.find("--force-read-nondefault") == 0 || argument.find("--force_read_nondefault") == 0)
                {
                    spdlog::info("From this point on, if 'default_value' is not available, force the parser to read 'value' (instead of dropping it) to fill the used setting-values.");
                    force_read_nondefault = true;
                }
                else if (argument.find("--end-force-read") == 0 || argument.find("--end_force_read") == 0)
                {
                    spdlog::info("From this point on, reset all force-XXX values to false (don't 'force read ___' anymore).");
                    force_read_parent = false;
                    force_read_nondefault = false;
                }
                else
                {
                    spdlog::error("Unknown option: {}", argument);
                }
            }
            else // Starts with "-" but not with "--".
            {
                argument = arguments[argument_index];
                switch (argument[1])
                {
                case 'v':
                {
                    spdlog::set_level(spdlog::level::debug);
                    break;
                }
                case 'm':
                {
                    int threads = stoi(argument.substr(2));
                    Application::getInstance().startThreadPool(threads);
                    break;
                }
                case 'p':
                {
                    // enableProgressLogging(); FIXME: how to handle progress logging? Is this still relevant?
                    break;
                }
                case 'j':
                {
                    argument_index++;
                    if (argument_index >= arguments.size())
                    {
                        spdlog::error("Missing JSON file with -j argument.");
                        exit(1);
                    }
                    argument = arguments[argument_index];
                    if (loadJSON(argument, *last_settings, force_read_parent, force_read_nondefault))
                    {
                        spdlog::error("Failed to load JSON file: {}", argument);
                        exit(1);
                    }

                    // If this was the global stack, create extruders for the machine_extruder_count setting.
                    if (last_settings == &slice.scene.settings)
                    {
                        const size_t extruder_count = slice.scene.settings.get<size_t>("machine_extruder_count");
                        while (slice.scene.extruders.size() < extruder_count)
                        {
                            slice.scene.extruders.emplace_back(slice.scene.extruders.size(), &slice.scene.settings);
                        }
                    }
                    // If this was an extruder stack, make sure that the extruder_nr setting is correct.
                    if (last_settings == &last_extruder->settings)
                    {
                        last_extruder->settings.add("extruder_nr", std::to_string(last_extruder->extruder_nr));
                    }
                    break;
                }
                case 'e':
                {
                    size_t extruder_nr = stoul(argument.substr(2));
                    while (slice.scene.extruders.size() <= extruder_nr) // Make sure we have enough extruders up to the extruder_nr that the user wanted.
                    {
                        slice.scene.extruders.emplace_back(extruder_nr, &slice.scene.settings);
                    }
                    last_settings = &slice.scene.extruders[extruder_nr].settings;
                    last_settings->add("extruder_nr", argument.substr(2));
                    last_extruder = &slice.scene.extruders[extruder_nr];
                    break;
                }
                case 'l':
                {
                    argument_index++;
                    if (argument_index >= arguments.size())
                    {
                        spdlog::error("Missing model file with -l argument.");
                        exit(1);
                    }
                    argument = arguments[argument_index];

                    const FMatrix4x3 transformation = last_settings->get<FMatrix4x3>("mesh_rotation_matrix"); // The transformation applied to the model when loaded.

                    if (! loadMeshIntoMeshGroup(&slice.scene.mesh_groups[mesh_group_index], argument.c_str(), transformation, last_extruder->settings))
                    {
                        spdlog::error("Failed to load model: {}. (error number {})", argument, errno);
                        exit(1);
                    }
                    else
                    {
                        last_settings = &slice.scene.mesh_groups[mesh_group_index].meshes.back().settings;
                    }
                    break;
                }
                case 'o':
                {
                    argument_index++;
                    if (argument_index >= arguments.size())
                    {
                        spdlog::error("Missing output file with -o argument.");
                        exit(1);
                    }
                    argument = arguments[argument_index];
                    if (! FffProcessor::getInstance()->setTargetFile(argument.c_str()))
                    {
                        spdlog::error("Failed to open {} for output.", argument.c_str());
                        exit(1);
                    }
                    break;
                }
                case 'g':
                {
                    last_settings = &slice.scene.mesh_groups[mesh_group_index].settings;
                    break;
                }
                /* ... falls through ... */
                case 's':
                {
                    // Parse the given setting and store it.
                    argument_index++;
                    if (argument_index >= arguments.size())
                    {
                        spdlog::error("Missing setting name and value with -s argument.");
                        exit(1);
                    }
                    argument = arguments[argument_index];
                    const size_t value_position = argument.find("=");
                    std::string key = argument.substr(0, value_position);
                    if (value_position == std::string::npos)
                    {
                        spdlog::error("Missing value in setting argument: -s {}", argument);
                        exit(1);
                    }
                    std::string value = argument.substr(value_position + 1);
                    last_settings->add(key, value);
                    break;
                }
                default:
                {
                    spdlog::error("Unknown option: -{}", argument[1]);
                    Application::getInstance().printCall();
                    Application::getInstance().printHelp();
                    exit(1);
                    break;
                }
                }
            }
        }
        else
        {
            spdlog::error("Unknown option: {}", argument);
            Application::getInstance().printCall();
            Application::getInstance().printHelp();
            exit(1);
        }
    }

    arguments.clear(); // We've processed all arguments now.

#ifndef DEBUG
    try
    {
#endif // DEBUG
        slice.scene.mesh_groups[mesh_group_index].finalize();
        spdlog::info("Loaded from disk in {:3}s\n", FffProcessor::getInstance()->time_keeper.restart());

        // Start slicing.
        slice.compute();
#ifndef DEBUG
    }
    catch (...)
    {
        // Catch all exceptions.
        // This prevents the "something went wrong" dialogue on Windows to pop up on a thrown exception.
        // Only ClipperLib currently throws exceptions. And only in the case that it makes an internal error.
        spdlog::error("Unknown exception.");
        exit(1);
    }
#endif // DEBUG

    // Finalize the processor. This adds the end g-code and reports statistics.
    FffProcessor::getInstance()->finalize();
}

int CommandLine::loadJSON(const std::string& json_filename, Settings& settings, bool force_read_parent, bool force_read_nondefault)
{
    FILE* file = fopen(json_filename.c_str(), "rb");
    if (! file)
    {
        spdlog::error("Couldn't open JSON file: {}", json_filename);
        return 1;
    }

    rapidjson::Document json_document;
    char read_buffer[4096];
    rapidjson::FileReadStream reader_stream(file, read_buffer, sizeof(read_buffer));
    json_document.ParseStream(reader_stream);
    fclose(file);
    if (json_document.HasParseError())
    {
        spdlog::error("Error parsing JSON (offset {}): {}", json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
        return 2;
    }

    std::unordered_set<std::string> search_directories = defaultSearchDirectories(); // For finding the inheriting JSON files.
    std::string directory = std::filesystem::path(json_filename).parent_path().string();
    search_directories.insert(directory);

    return loadJSON(json_document, search_directories, settings, force_read_parent, force_read_nondefault);
}

std::unordered_set<std::string> CommandLine::defaultSearchDirectories()
{
    std::unordered_set<std::string> result;

    char* search_path_env = getenv("CURA_ENGINE_SEARCH_PATH");
    if (search_path_env)
    {
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
        char delims[] = ":"; // Colon for Unix.
#else
        char delims[] = ";"; // Semicolon for Windows.
#endif
        char paths[128 * 1024]; // Maximum length of environment variable.
        strcpy(paths, search_path_env); // Necessary because strtok actually modifies the original string, and we don't want to modify the environment variable itself.
        char* path = strtok(paths, delims);
        while (path != nullptr)
        {
            result.insert(path);
            path = strtok(nullptr, delims); // Continue searching in last call to strtok.
        }
    }
    return result;
}

int CommandLine::loadJSON
(
    const rapidjson::Document& document,
    const std::unordered_set<std::string>& search_directories,
    Settings& settings,
    bool force_read_parent,
    bool force_read_nondefault
)
{
    // Inheritance from other JSON documents.
    if (document.HasMember("inherits") && document["inherits"].IsString())
    {
        std::string parent_file = findDefinitionFile(document["inherits"].GetString(), search_directories);
        if (parent_file == "")
        {
            spdlog::error("Inherited JSON file: {} not found.", document["inherits"].GetString());
            return 1;
        }
        int error_code = loadJSON(parent_file, settings, force_read_parent, force_read_nondefault); // Head-recursively load the settings file that we inherit from.
        if (error_code)
        {
            return error_code;
        }
    }

    // Extruders defined from here, if any.
    // Note that this always puts the extruder settings in the slice of the current extruder. It doesn't keep the nested structure of the JSON files, if extruders would have their own sub-extruders.
    Scene& scene = Application::getInstance().current_slice->scene;
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
                while (scene.extruders.size() <= static_cast<size_t>(extruder_nr))
                {
                    scene.extruders.emplace_back(scene.extruders.size(), &scene.settings);
                }
                const rapidjson::Value& extruder_id = extruder_train->value;
                if (! extruder_id.IsString())
                {
                    continue;
                }
                const std::string extruder_definition_id(extruder_id.GetString());
                const std::string extruder_file = findDefinitionFile(extruder_definition_id, search_directories);
                loadJSON(extruder_file, scene.extruders[extruder_nr].settings, force_read_parent, force_read_nondefault);
            }
        }
    }

    if (document.HasMember("settings") && document["settings"].IsObject())
    {
        loadJSONSettings(document["settings"], settings, force_read_parent, force_read_nondefault);
    }
    if (document.HasMember("overrides") && document["overrides"].IsObject())
    {
        loadJSONSettings(document["overrides"], settings, force_read_parent, force_read_nondefault);
    }
    return 0;
}

bool jsonValue2Str(const rapidjson::Value& value, std::string& value_string)
{
    if (value.IsString())
    {
        value_string = value.GetString();
    }
    else if (value.IsTrue())
    {
        value_string = "true";
    }
    else if (value.IsFalse())
    {
        value_string = "false";
    }
    else if (value.IsNumber())
    {
        value_string = std::to_string(value.GetDouble());
    }
    else if (value.IsArray())
    {
        if (value.Empty())
        {
            value_string = "[]";
            return true;
        }
        std::string temp;
        jsonValue2Str(value[0], temp);
        value_string =
            std::string("[") +
            std::accumulate
            (
                std::next(value.Begin()),
                value.End(),
                temp,
                [&temp](std::string converted, const rapidjson::Value& next)
                {
                    jsonValue2Str(next, temp);
                    return std::move(converted) + "," + temp;
                }
            ) +
            std::string("]");
    }
    else
    {
        return false;
    }
    return true;
}

void CommandLine::loadJSONSettings(const rapidjson::Value& element, Settings& settings, bool force_read_parent, bool force_read_nondefault)
{
    for (rapidjson::Value::ConstMemberIterator setting = element.MemberBegin(); setting != element.MemberEnd(); setting++)
    {
        const std::string name = setting->name.GetString();

        const rapidjson::Value& setting_object = setting->value;
        if (! setting_object.IsObject())
        {
            spdlog::error("JSON setting {} is not an object!", name);
            continue;
        }

        if (setting_object.HasMember("children"))
        {
            loadJSONSettings(setting_object["children"], settings, force_read_parent, force_read_nondefault);
            if (! force_read_parent)
            {
                continue;
            }
        }

        if (! (setting_object.HasMember("default_value") || (force_read_nondefault && setting_object.HasMember("value") && ! settings.has(name))))
        {
            if (! setting_object.HasMember("children"))
            {
                // Setting has no child-settings, so must be leaf, but also holds no (default) value?!
                spdlog::warn("JSON setting {} has no [default_]value!", name);
            }
            continue;
        }

        // At this point in the code, it's known that the setting either has a default value _or_ force_read_nondefault _and_ has-member 'value' is true.
        const rapidjson::Value& json_value = setting_object.HasMember("default_value") ? setting_object["default_value"] : setting_object["value"];

        std::string value_string;
        if (! jsonValue2Str(json_value, value_string))
        {
            spdlog::warn("Unrecognized data type in JSON setting {}", name);
            continue;
        }
        settings.add(name, value_string);
    }
}

const std::string CommandLine::findDefinitionFile(const std::string& definition_id, const std::unordered_set<std::string>& search_directories)
{
    for (const std::string& search_directory : search_directories)
    {
        const std::string candidate = search_directory + std::string("/") + definition_id + std::string(".def.json");
        const std::ifstream ifile(candidate.c_str()); // Check whether the file exists and is readable by opening it.
        if (ifile)
        {
            return candidate;
        }
    }
    spdlog::error("Couldn't find definition file with ID: {}", definition_id);
    return std::string("");
}

} // namespace cura