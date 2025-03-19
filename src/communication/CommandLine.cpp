// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/CommandLine.h"

#include <cerrno> // error number when trying to read file
#include <cstring> //For strtok and strcopy.
#include <filesystem>
#include <fstream> //To check if files exist.
#include <numeric> //For std::accumulate.
#include <optional>
#include <rapidjson/error/en.h> //Loading JSON documents to get settings from them.
#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <range/v3/all.hpp>
#include <spdlog/details/os.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To get the extruders for material estimates.
#include "ExtruderTrain.h"
#include "FffProcessor.h" //To start a slice and get time estimates.
#include "MeshGroup.h"
#include "Slice.h"
#include "utils/Matrix4x3D.h" //For the mesh_rotation_matrix setting.
#include "utils/format/filesystem_path.h"
#include "utils/views/split_paths.h"

namespace cura
{

CommandLine::CommandLine(const std::vector<std::string>& arguments)
    : arguments_{ arguments }
    , last_shown_progress_{ 0 }
{
    if (auto search_paths = spdlog::details::os::getenv("CURA_ENGINE_SEARCH_PATH"); ! search_paths.empty())
    {
        search_directories_ = search_paths | views::split_paths | ranges::to<std::vector<std::filesystem::path>>();
    };
}

// These are not applicable to command line slicing.
void CommandLine::beginGCode()
{
}
void CommandLine::flushGCode()
{
}
void CommandLine::sendCurrentPosition(const Point3LL&)
{
}
void CommandLine::sendFinishedSlicing() const
{
}
void CommandLine::sendLayerComplete(const LayerIndex::value_type&, const coord_t&, const coord_t&)
{
}
void CommandLine::sendLineTo(const PrintFeatureType&, const Point3LL&, const coord_t&, const coord_t&, const Velocity&)
{
}
void CommandLine::sendOptimizedLayerData()
{
}
void CommandLine::setExtruderForSend(const ExtruderTrain&)
{
}
void CommandLine::setLayerForSend(const LayerIndex::value_type&)
{
}

bool CommandLine::hasSlice() const
{
    return ! arguments_.empty();
}

bool CommandLine::isSequential() const
{
    return true; // We have to receive the g-code in sequential order. Start g-code before the rest and so on.
}

void CommandLine::sendGCodePrefix(const std::string&) const
{
    // TODO: Right now this is done directly in the g-code writer. For consistency it should be moved here?
}

void CommandLine::sendSliceUUID([[maybe_unused]] const std::string& slice_uuid) const
{
    // pass
}

void CommandLine::sendPrintTimeMaterialEstimates() const
{
    std::vector<Duration> time_estimates = FffProcessor::getInstance()->getTotalPrintTimePerFeature();
    double sum = std::accumulate(time_estimates.begin(), time_estimates.end(), 0.0);
    spdlog::info("Total print time: {:3}", sum);

    sum = 0.0;
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice_->scene.extruders.size(); extruder_nr++)
    {
        sum += FffProcessor::getInstance()->getTotalFilamentUsed(static_cast<int>(extruder_nr));
    }
}

void CommandLine::sendProgress(double progress) const
{
    const unsigned int rounded_amount = 100 * progress;
    if (last_shown_progress_ == rounded_amount) // No need to send another tiny update step.
    {
        return;
    }
}

void CommandLine::sliceNext()
{
    FffProcessor::getInstance()->time_keeper.restart();

    // Count the number of mesh groups to slice for.
    size_t num_mesh_groups = 1;
    for (size_t argument_index = 2; argument_index < arguments_.size(); argument_index++)
    {
        if (arguments_[argument_index].starts_with("--next")) // Starts with "--next".
        {
            num_mesh_groups++;
        }
    }

    Application::getInstance().current_slice_ = std::make_shared<Slice>(num_mesh_groups);
    auto slice = Application::getInstance().current_slice_;

    size_t mesh_group_index = 0;
    Settings* last_settings = &slice->scene.settings;

    slice->scene.extruders.reserve(arguments_.size() >> 1); // Allocate enough memory to prevent moves.
    slice->scene.extruders.emplace_back(0, &slice->scene.settings); // Always have one extruder.
    ExtruderTrain* last_extruder = slice->scene.extruders.data();

    bool force_read_parent = false;
    bool force_read_nondefault = false;

    for (size_t argument_index = 2; argument_index < arguments_.size(); argument_index++)
    {
        std::string argument = arguments_[argument_index];
        if (argument[0] == '-') // Starts with "-".
        {
            if (argument[1] == '-') // Starts with "--".
            {
                if (argument.starts_with("--next")) // Starts with "--next".
                {
                    try
                    {
                        spdlog::info("Loaded from disk in {}", FffProcessor::getInstance()->time_keeper.restart());

                        mesh_group_index++;
                        FffProcessor::getInstance()->time_keeper.restart();
                        last_settings = &slice->scene.mesh_groups[mesh_group_index].settings;
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
                else if (argument.starts_with("--force-read-parent") || argument.starts_with("--force_read_parent"))
                {
                    spdlog::info("From this point on, force the parser to read values of non-leaf settings, instead of skipping over them as is proper.");
                    force_read_parent = true;
                }
                else if (argument.starts_with("--force-read-nondefault") || argument.starts_with("--force_read_nondefault"))
                {
                    spdlog::info(
                        "From this point on, if 'default_value' is not available, force the parser to read 'value' (instead of dropping it) to fill the used setting-values.");
                    force_read_nondefault = true;
                }
                else if (argument.starts_with("--end-force-read") || argument.starts_with("--end_force_read"))
                {
                    spdlog::info("From this point on, reset all force-XXX values to false (don't 'force read ___' anymore).");
                    force_read_parent = false;
                    force_read_nondefault = false;
                }
                else if (
                    argument.starts_with("--progress_cb") || argument.starts_with("--slice_info_cb") || argument.starts_with("--gcode_header_cb")
                    || argument.starts_with("--engine_info_cb"))
                {
                    // Unused in command line slicing, but used in EmscriptenCommunication.
                    argument_index++;
                    argument = arguments_[argument_index];
                }
                else
                {
                    spdlog::error("Unknown option: {}", argument);
                }
            }
            else // Starts with "-" but not with "--".
            {
                argument = arguments_[argument_index];
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
                case 'd':
                {
                    argument_index++;
                    if (argument_index >= arguments_.size())
                    {
                        spdlog::error("Missing definition search paths");
                        exit(1);
                    }
                    argument = arguments_[argument_index];
                    search_directories_ = argument | views::split_paths | ranges::to<std::vector<std::filesystem::path>>();
                    break;
                }
                case 'j':
                {
                    argument_index++;
                    if (argument_index >= arguments_.size())
                    {
                        spdlog::error("Missing JSON file with -j argument.");
                        exit(1);
                    }
                    argument = arguments_[argument_index];
                    if (loadJSON(std::filesystem::path{ argument }, *last_settings, force_read_parent, force_read_nondefault) != 0)
                    {
                        spdlog::error("Failed to load JSON file: {}", argument);
                        exit(1);
                    }

                    // If this was the global stack, create extruders for the machine_extruder_count setting.
                    if (last_settings == &slice->scene.settings)
                    {
                        const auto extruder_count = slice->scene.settings.get<size_t>("machine_extruder_count");
                        while (slice->scene.extruders.size() < extruder_count)
                        {
                            slice->scene.extruders.emplace_back(slice->scene.extruders.size(), &slice->scene.settings);
                        }
                    }
                    // If this was an extruder stack, make sure that the extruder_nr setting is correct.
                    if (last_settings == &last_extruder->settings_)
                    {
                        last_extruder->settings_.add("extruder_nr", std::to_string(last_extruder->extruder_nr_));
                    }
                    break;
                }
                case 'e':
                {
                    size_t extruder_nr = stoul(argument.substr(2));
                    while (slice->scene.extruders.size() <= extruder_nr) // Make sure we have enough extruders up to the extruder_nr that the user wanted.
                    {
                        slice->scene.extruders.emplace_back(extruder_nr, &slice->scene.settings);
                    }
                    last_settings = &slice->scene.extruders[extruder_nr].settings_;
                    last_settings->add("extruder_nr", argument.substr(2));
                    last_extruder = &slice->scene.extruders[extruder_nr];
                    break;
                }
                case 'l':
                {
                    argument_index++;
                    if (argument_index >= arguments_.size())
                    {
                        spdlog::error("Missing model file with -l argument.");
                        exit(1);
                    }
                    argument = arguments_[argument_index];

                    const auto transformation = last_settings->get<Matrix4x3D>("mesh_rotation_matrix"); // The transformation applied to the model when loaded.

                    if (! loadMeshIntoMeshGroup(&slice->scene.mesh_groups[mesh_group_index], argument.c_str(), transformation, last_extruder->settings_))
                    {
                        spdlog::error("Failed to load model: {}. (error number {})", argument, errno);
                        exit(1);
                    }
                    else
                    {
                        last_settings = &slice->scene.mesh_groups[mesh_group_index].meshes.back().settings_;
                    }
                    break;
                }
                case 'o':
                {
                    argument_index++;
                    if (argument_index >= arguments_.size())
                    {
                        spdlog::error("Missing output file with -o argument.");
                        exit(1);
                    }
                    argument = arguments_[argument_index];
                    if (! FffProcessor::getInstance()->setTargetFile(argument.c_str()))
                    {
                        spdlog::error("Failed to open {} for output.", argument.c_str());
                        exit(1);
                    }
                    break;
                }
                case 'g':
                {
                    last_settings = &slice->scene.mesh_groups[mesh_group_index].settings;
                    break;
                }
                /* ... falls through ... */
                case 's':
                {
                    // Parse the given setting and store it.
                    argument_index++;
                    if (argument_index >= arguments_.size())
                    {
                        spdlog::error("Missing setting name and value with -s argument.");
                        exit(1);
                    }
                    argument = arguments_[argument_index];
                    const size_t value_position = argument.find('=');
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
                case 'r':
                {
                    /*
                     * read in resolved values from a json file. The json format of the file resolved settings is the following:
                     *
                     * ```
                     * {
                     *     "global": [SETTINGS],
                     *     "extruder.0": [SETTINGS],
                     *     "extruder.1": [SETTINGS],
                     *     "model.stl": [SETTINGS]
                     * }
                     * ```
                     * where `[SETTINGS]` follow the schema
                     * ```
                     * {
                     *     [key: string]: bool | string | number | number[] | number[][]
                     * }
                     * ```
                     * There can be any number of extruders (denoted with `extruder.n`) and any number of models (denoted with `[modelname].stl`).
                     * The key of the model values will also be the filename of the relevant model, when running CuraEngine with this option the
                     * model file with that same name _must_ be in the same folder as the resolved settings json.
                     */

                    argument_index++;
                    if (argument_index >= arguments_.size())
                    {
                        spdlog::error("Missing setting name and value with -r argument.");
                        exit(1);
                    }
                    argument = arguments_[argument_index];
                    const auto settings = readResolvedJsonValues(std::filesystem::path{ argument });

                    if (! settings.has_value())
                    {
                        spdlog::error("Failed to load JSON file: {}", argument);
                        exit(1);
                    }

                    constexpr std::string_view global_identifier = "global";
                    constexpr std::string_view extruder_identifier = "extruder.";
                    constexpr std::string_view model_identifier = "model.";
                    constexpr std::string_view limit_to_extruder_identifier = "limit_to_extruder";

                    // Split the settings into global, extruder and model settings. This is needed since the order in which the settings are applied is important.
                    // first global settings, then extruder settings, then model settings. The order of these stacks is not enforced in the JSON files.
                    std::unordered_map<std::string, std::string> global_settings;
                    container_setting_map extruder_settings;
                    container_setting_map model_settings;
                    std::unordered_map<std::string, std::string> limit_to_extruder;

                    for (const auto& [key, values] : settings.value())
                    {
                        if (key == global_identifier)
                        {
                            global_settings = values;
                        }
                        else if (key.starts_with(extruder_identifier))
                        {
                            extruder_settings[key] = values;
                        }
                        else if (key == limit_to_extruder_identifier)
                        {
                            limit_to_extruder = values;
                        }
                        else
                        {
                            model_settings[key] = values;
                        }
                    }

                    for (const auto& [setting_key, setting_value] : global_settings)
                    {
                        slice->scene.settings.add(setting_key, setting_value);
                    }

                    for (const auto& [key, values] : extruder_settings)
                    {
                        const auto extruder_nr = std::stoi(key.substr(extruder_identifier.size()));
                        while (slice->scene.extruders.size() <= static_cast<size_t>(extruder_nr))
                        {
                            slice->scene.extruders.emplace_back(extruder_nr, &slice->scene.settings);
                        }
                        for (const auto& [setting_key, setting_value] : values)
                        {
                            slice->scene.extruders[extruder_nr].settings_.add(setting_key, setting_value);
                        }
                    }

                    for (const auto& [key, values] : model_settings)
                    {
                        const auto& model_name = key;
                        for (const auto& [setting_key, setting_value] : values)
                        {
                            slice->scene.mesh_groups[mesh_group_index].settings.add(setting_key, setting_value);
                        }

                        const auto transformation = slice->scene.mesh_groups[mesh_group_index].settings.get<Matrix4x3D>("mesh_rotation_matrix");
                        const auto extruder_nr = slice->scene.mesh_groups[mesh_group_index].settings.get<size_t>("extruder_nr");

                        if (! loadMeshIntoMeshGroup(&slice->scene.mesh_groups[mesh_group_index], model_name.c_str(), transformation, slice->scene.extruders[extruder_nr].settings_))
                        {
                            spdlog::error("Failed to load model: {}. (error number {})", model_name, errno);
                            exit(1);
                        }
                    }
                    for (const auto& [key, value] : limit_to_extruder)
                    {
                        const auto extruder_nr = std::stoi(value.substr(extruder_identifier.size()));
                        if (extruder_nr >= 0)
                        {
                            slice->scene.limit_to_extruder[key] = &slice->scene.extruders[extruder_nr];
                        }
                    }

                    break;
                }
                default:
                {
                    spdlog::error("Unknown option: -{}", argument[1]);
                    Application::getInstance().printCall();
                    Application::getInstance().printHelp();
                    exit(1);
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

    arguments_.clear(); // We've processed all arguments now.

#ifndef DEBUG
    try
    {
#endif // DEBUG
        slice->scene.mesh_groups[mesh_group_index].finalize();
        spdlog::info("Loaded from disk in {:3}s\n", FffProcessor::getInstance()->time_keeper.restart());

        // Start slicing.
        slice->compute();
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

int CommandLine::loadJSON(const std::filesystem::path& json_filename, Settings& settings, bool force_read_parent, bool force_read_nondefault)
{
    std::ifstream file(json_filename, std::ios::binary);
    if (! file)
    {
        spdlog::error("Couldn't open JSON file: {}", json_filename.generic_string());
        return 1;
    }

    std::vector<char> read_buffer(std::istreambuf_iterator<char>(file), {});
    rapidjson::MemoryStream memory_stream(read_buffer.data(), read_buffer.size());

    rapidjson::Document json_document;
    json_document.ParseStream(memory_stream);
    if (json_document.HasParseError())
    {
        spdlog::error("Error parsing JSON (offset {}): {}", json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
        return 2;
    }

    search_directories_.push_back(std::filesystem::path(json_filename).parent_path());
    return loadJSON(json_document, search_directories_, settings, force_read_parent, force_read_nondefault);
}

int CommandLine::loadJSON(
    const rapidjson::Document& document,
    const std::vector<std::filesystem::path>& search_directories,
    Settings& settings,
    bool force_read_parent,
    bool force_read_nondefault)
{
    // Inheritance from other JSON documents.
    if (document.HasMember("inherits") && document["inherits"].IsString())
    {
        std::string parent_file = findDefinitionFile(document["inherits"].GetString(), search_directories);
        if (parent_file.empty())
        {
            spdlog::error("Inherited JSON file: {} not found.", document["inherits"].GetString());
            return 1;
        }
        // Head-recursively load the settings file that we inherit from.
        if (const auto error_code = loadJSON(parent_file, settings, force_read_parent, force_read_nondefault); error_code != 0)
        {
            return error_code;
        }
    }

    // Extruders defined from here, if any.
    // Note that this always puts the extruder settings in the slice of the current extruder. It doesn't keep the nested structure of the JSON files, if extruders would have their
    // own sub-extruders.
    Scene& scene = Application::getInstance().current_slice_->scene;
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
                loadJSON(extruder_file, scene.extruders[extruder_nr].settings_, force_read_parent, force_read_nondefault);
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
        value_string = std::string("[")
                     + std::accumulate(
                           std::next(value.Begin()),
                           value.End(),
                           temp,
                           [&temp](std::string converted, const rapidjson::Value& next)
                           {
                               jsonValue2Str(next, temp);
                               return std::move(converted) + "," + temp;
                           })
                     + std::string("]");
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

        if (! setting_object.HasMember("default_value") && (! force_read_nondefault || ! setting_object.HasMember("value") || settings.has(name)))
        {
            if (! setting_object.HasMember("children"))
            {
                // Setting has no child-settings, so must be leaf, but also holds no (default) value?!
                spdlog::warn("JSON setting '{}' has no [default_]value!", name);
                rapidjson::StringBuffer buffer;
                rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
                setting_object.Accept(writer);
                spdlog::debug("JSON setting '{}': '{}'", name, buffer.GetString());
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

std::optional<container_setting_map> CommandLine::readResolvedJsonValues(const std::filesystem::path& json_filename)
{
    std::ifstream file(json_filename, std::ios::binary);
    if (! file)
    {
        spdlog::error("Couldn't open JSON file: {}", json_filename.generic_string());
        return std::nullopt;
    }

    std::vector<char> read_buffer(std::istreambuf_iterator<char>(file), {});
    rapidjson::MemoryStream memory_stream(read_buffer.data(), read_buffer.size());

    rapidjson::Document json_document;
    json_document.ParseStream(memory_stream);
    if (json_document.HasParseError())
    {
        spdlog::error("Error parsing JSON (offset {}): {}", json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
        return std::nullopt;
    }

    return readResolvedJsonValues(json_document);
}

std::optional<container_setting_map> CommandLine::readResolvedJsonValues(const rapidjson::Document& document)
{
    if (! document.IsObject())
    {
        return std::nullopt;
    }

    container_setting_map result;
    for (rapidjson::Value::ConstMemberIterator resolved_key = document.MemberBegin(); resolved_key != document.MemberEnd(); resolved_key++)
    {
        std::unordered_map<std::string, std::string> values;
        for (rapidjson::Value::ConstMemberIterator resolved_value = resolved_key->value.MemberBegin(); resolved_value != resolved_key->value.MemberEnd(); resolved_value++)
        {
            std::string value_string;
            if (! jsonValue2Str(resolved_value->value, value_string))
            {
                spdlog::warn("Unrecognized data type in JSON setting {}", resolved_value->name.GetString());
                continue;
            }
            values.emplace(resolved_value->name.GetString(), value_string);
        }
        result.emplace(resolved_key->name.GetString(), std::move(values));
    }
    return result;
}

std::string CommandLine::findDefinitionFile(const std::string& definition_id, const std::vector<std::filesystem::path>& search_directories)
{
    for (const auto& search_directory : search_directories)
    {
        if (auto candidate = search_directory / (definition_id + ".def.json"); std::filesystem::exists(candidate))
        {
            return candidate.string();
        }
    }
    spdlog::error("Couldn't find definition file with ID: {}", definition_id);
    return {};
}

} // namespace cura
