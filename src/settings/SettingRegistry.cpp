/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "SettingRegistry.h"

#include <sstream>
#include <iostream> // debug IO
#include <libgen.h> // dirname
#include <string>
#include <cstring> // strtok (split string using delimiters)
#include <fstream> // ifstream (to see if file exists)

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "../utils/logoutput.h"

namespace cura
{
    
SettingRegistry SettingRegistry::instance; // define settingRegistry

std::string SettingRegistry::toString(rapidjson::Type type)
{
    switch (type)
    {
        case rapidjson::Type::kNullType: return "null";
        case rapidjson::Type::kFalseType: return "false";
        case rapidjson::Type::kTrueType: return "true";
        case rapidjson::Type::kObjectType: return "object";
        case rapidjson::Type::kArrayType: return "array";
        case rapidjson::Type::kStringType: return "string";
        case rapidjson::Type::kNumberType: return "number";
        default: return "Unknown";
    }
}


SettingConfig::SettingConfig(std::string key, std::string label)
: SettingContainer(key, label)
{
//     std::cerr << key << std::endl; // debug output to show all frontend registered settings...
}

bool SettingRegistry::settingExists(std::string key) const
{
    return setting_key_to_config.find(key) != setting_key_to_config.end();
}

SettingConfig* SettingRegistry::getSettingConfig(std::string key) const
{
    auto it = setting_key_to_config.find(key);
    if (it == setting_key_to_config.end())
        return nullptr;
    return it->second;
}

SettingContainer* SettingRegistry::getExtruderTrain(unsigned int extruder_nr)
{
    if (extruder_nr < 0 || extruder_nr >= extruder_trains.size())
    {
        return nullptr;
    }
    return &extruder_trains[extruder_nr];
}


SettingRegistry::SettingRegistry()
: setting_definitions("settings", "Settings")
{
}

int SettingRegistry::loadJSON(std::string filename, rapidjson::Document& json_document)
{
    FILE* f = fopen(filename.c_str(), "rb");
    if (!f)
    {
        cura::logError("Couldn't open JSON file.\n");
        return 1;
    }
    char read_buffer[4096];
    rapidjson::FileReadStream reader_stream(f, read_buffer, sizeof(read_buffer));
    json_document.ParseStream(reader_stream);
    fclose(f);
    if (json_document.HasParseError())
    {
        cura::logError("Error parsing JSON(offset %u): %s\n", (unsigned)json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
        return 2;
    }

    return 0;
}

/*!
 * Check whether a file exists.
 * from https://techoverflow.net/blog/2013/01/11/cpp-check-if-file-exists/
 * 
 * \param filename The path to a filename to check if it exists
 * \return Whether the file exists.
 */
bool fexists(const char *filename)
{
  std::ifstream ifile(filename);
  return (bool)ifile;
}

bool SettingRegistry::getDefinitionFile(const std::string machine_id, const std::string parent_file, std::string& result)
{
    // check for file in same directory as the file provided
    std::string parent_filename_copy = std::string(parent_file.c_str()); // copy the string because dirname(.) changes the input string!!!
    char* parent_filename_cstr = (char*)parent_filename_copy.c_str();
    result = std::string(dirname(parent_filename_cstr)) + std::string("/") + machine_id + std::string(".def.json");
    if (fexists(result.c_str()))
    {
        return true;
    }

    // check for file in the directories supplied in the environment variable CURA_ENGINE_SEARCH_PATH
    char* paths = getenv("CURA_ENGINE_SEARCH_PATH");
    if (paths)
    {
        char* path = strtok(paths,";:,"); // search for path delimited by ';', ':', ','
        while (path != NULL)
        {
            result = std::string(path) + std::string("/") + machine_id + std::string(".def.json");
            if (fexists(result.c_str()))
            {
                return true;
            }
            path = strtok(NULL, ";:,"); // continue searching in last call to strtok
        }
    }
    return false;
}

int SettingRegistry::loadJSONsettings(std::string filename, SettingsBase* settings_base)
{
    rapidjson::Document json_document;
    
    int err = loadJSON(filename, json_document);
    if (err) { return err; }
    bool overload_defaults_only;

    log("Loading %s...\n", filename.c_str());
    if (json_document.HasMember("inherits") && json_document["inherits"].IsString())
    {
        std::string child_filename;
        bool found = getDefinitionFile(json_document["inherits"].GetString(), filename, child_filename);
        if (!found)
        {
            return -1;
        }
        int err = loadJSONsettings(child_filename, settings_base); // load child first
        if (err)
        {
            return err;
        }
        overload_defaults_only = true;
        return loadJSONsettingsFromDoc(json_document, settings_base, false, overload_defaults_only); // overload settings of this definition file
    }
    else 
    {
        overload_defaults_only = false;
        return loadJSONsettingsFromDoc(json_document, settings_base, true, overload_defaults_only);
    }
}

int SettingRegistry::loadJSONsettingsFromDoc(rapidjson::Document& json_document, SettingsBase* settings_base, bool warn_duplicates, bool overload_defaults_only)
{
    
    if (!json_document.IsObject())
    {
        cura::logError("JSON file is not an object.\n");
        return 3;
    }

    { // handle machine name
        std::string machine_name = "Unknown";
        if (json_document.HasMember("name"))
        {
            const rapidjson::Value& machine_name_field = json_document["name"];
            if (machine_name_field.IsString())
            {
                machine_name = machine_name_field.GetString();
            }
        }
        SettingConfig& machine_name_setting = addSetting("machine_name", "Machine Name");
        machine_name_setting.setDefault(machine_name);
        machine_name_setting.setType("string");
    }

    if (json_document.HasMember("settings"))
    {
        std::list<std::string> path;
        handleChildren(json_document["settings"], path, settings_base, warn_duplicates, overload_defaults_only);
    }
    
    if (json_document.HasMember("overrides"))
    {
        const rapidjson::Value& json_object_container = json_document["overrides"];
        for (rapidjson::Value::ConstMemberIterator override_iterator = json_object_container.MemberBegin(); override_iterator != json_object_container.MemberEnd(); ++override_iterator)
        {
            std::string setting = override_iterator->name.GetString();
            SettingConfig* conf = getSettingConfig(setting);
            if (!conf) //Setting could not be found.
            {
//                 logWarning("Trying to override unknown setting %s.\n", setting.c_str());
                continue;
            }
            _loadSettingValues(conf, override_iterator);
        }
    }
    
    return 0;
}

void SettingRegistry::handleChildren(const rapidjson::Value& settings_list, std::list<std::string>& path, SettingsBase* settings_base, bool warn_duplicates, bool overload_defaults_only)
{
    if (!settings_list.IsObject())
    {
        logError("ERROR: json settings list is not an object!\n");
        return;
    }
    for (rapidjson::Value::ConstMemberIterator setting_iterator = settings_list.MemberBegin(); setting_iterator != settings_list.MemberEnd(); ++setting_iterator)
    {
        handleSetting(setting_iterator, path, settings_base, warn_duplicates, overload_defaults_only);
        if (setting_iterator->value.HasMember("children"))
        {
            std::list<std::string> path_here = path;
            path_here.push_back(setting_iterator->name.GetString());
            handleChildren(setting_iterator->value["children"], path_here, settings_base, warn_duplicates, overload_defaults_only);
        }
    }
}

bool SettingRegistry::settingIsUsedByEngine(const rapidjson::Value& setting)
{
    if (setting.HasMember("children"))
    {
        return false;
    }
    else
    {
        return true;
    }
}


void SettingRegistry::handleSetting(const rapidjson::Value::ConstMemberIterator& json_setting_it, std::list<std::string>& path, SettingsBase* settings_base, bool warn_duplicates, bool overload_defaults_only)
{
    const rapidjson::Value& json_setting = json_setting_it->value;
    if (!json_setting.IsObject())
    {
        logError("ERROR: json setting is not an object!\n");
        return;
    }
    std::string name = json_setting_it->name.GetString();
    if (json_setting.HasMember("type") && json_setting["type"].IsString() && json_setting["type"].GetString() == std::string("category"))
    { // skip category objects
        return;
    }
    if (settingIsUsedByEngine(json_setting))
    {
        if (!json_setting.HasMember("label") || !json_setting["label"].IsString())
        {
            logError("ERROR: json setting \"%s\" has no label!\n", name.c_str());
            return;
        }
        std::string label = json_setting["label"].GetString();
        
        settings_base->_setSetting(name, getDefault(json_setting_it));
        if (!overload_defaults_only)
        {
            SettingConfig* setting = getSettingConfig(name);
            if (warn_duplicates && setting)
            {
                cura::logError("Duplicate definition of setting: %s a.k.a. \"%s\" was already claimed by \"%s\"\n", name.c_str(), label.c_str(), getSettingConfig(name)->getLabel().c_str());
            }
            if (!setting)
            {
                setting = &addSetting(name, label);
            }
            _loadSettingValues(setting, json_setting_it);
        }
    }
}

SettingConfig& SettingRegistry::addSetting(std::string name, std::string label)
{
    SettingConfig* config = setting_definitions.addChild(name, label);

    setting_key_to_config[name] = config;

    return *config;
}

std::string SettingRegistry::getDefault(const rapidjson::GenericValue< rapidjson::UTF8< char > >::ConstMemberIterator& json_object_it)
{
    const rapidjson::Value& setting_content = json_object_it->value;
    if (setting_content.HasMember("default_value"))
    {
        const rapidjson::Value& dflt = setting_content["default_value"];
        if (dflt.IsString())
        {
            return dflt.GetString();
        }
        else if (dflt.IsTrue())
        {
            return "true";
        }
        else if (dflt.IsFalse())
        {
            return "false";
        }
        else if (dflt.IsNumber())
        {
            std::ostringstream ss;
            ss << dflt.GetDouble();
            return ss.str();
        } // arrays are ignored because machine_extruder_trains needs to be handled separately
        else 
        {
            if (setting_content.HasMember("type") && setting_content["type"].IsString() && 
                (setting_content["type"].GetString() == std::string("polygon") || setting_content["type"].GetString() == std::string("polygons")))
            {
//                 logWarning("WARNING: Loading polygon setting %s not implemented...\n", json_object_it->name.GetString());
            }
            else
            {
//                 logWarning("WARNING: Unrecognized data type in JSON: %s has type %s\n", json_object_it->name.GetString(), toString(dflt.GetType()).c_str());
            }
        }
    }
    return "";
}


void SettingRegistry::_loadSettingValues(SettingConfig* config, const rapidjson::GenericValue< rapidjson::UTF8< char > >::ConstMemberIterator& json_object_it)
{
    const rapidjson::Value& data = json_object_it->value;
    /// Fill the setting config object with data we have in the json file.
    if (data.HasMember("type") && data["type"].IsString())
    {
        config->setType(data["type"].GetString());
    }
    config->setDefault(getDefault(json_object_it));

    if (data.HasMember("unit") && data["unit"].IsString())
    {
        config->setUnit(data["unit"].GetString());
    }
}

}//namespace cura
