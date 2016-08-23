/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "SettingRegistry.h"

#include <sstream>
#include <iostream> // debug IO
#ifndef WIN32
#include <libgen.h> // dirname
#else
extern char *dirname(char *path);
#endif
#include <string>
#include <cstring> // strtok (split string using delimiters) strcpy
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

SettingRegistry::SettingRegistry()
: setting_definitions("settings", "Settings")
{
    // load search paths from environment variable CURA_ENGINE_SEARCH_PATH
    char* paths = getenv("CURA_ENGINE_SEARCH_PATH");
    if (paths)
    {
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
        char delims[] = ":"; // colon
#else
        char delims[] = ";"; // semicolon
#endif
        char* path = strtok(paths, delims); // search for next path delimited by any of the characters in delims
        while (path != NULL)
        {
            search_paths.emplace(path);
            path = strtok(NULL, ";:,"); // continue searching in last call to strtok
        }
    }
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

bool SettingRegistry::getDefinitionFile(const std::string machine_id, std::string& result)
{
    for (const std::string& search_path : search_paths)
    {
        result = search_path + std::string("/") + machine_id + std::string(".def.json");
        if (fexists(result.c_str()))
        {
            return true;
        }
    }
    return false;
}


int SettingRegistry::loadExtruderJSONsettings(unsigned int extruder_nr, SettingsBase* settings_base)
{
    if (extruder_nr >= extruder_train_ids.size())
    {
        return -1;
    }
    
    std::string definition_file;
    bool found = getDefinitionFile(extruder_train_ids[extruder_nr], definition_file);
    if (!found)
    {
        return -1;
    }
    bool warn_base_file_duplicates = false;
    return loadJSONsettings(definition_file, settings_base, warn_base_file_duplicates);
}

int SettingRegistry::loadJSONsettings(std::string filename, SettingsBase* settings_base, bool warn_base_file_duplicates)
{
    rapidjson::Document json_document;
    
    log("Loading %s...\n", filename.c_str());

    int err = loadJSON(filename, json_document);
    if (err) { return err; }

    { // add parent folder to search paths
        char *filename_cstr = (char*)alloca(filename.size());
        std::strcpy(filename_cstr, filename.c_str()); // copy the string because dirname(.) changes the input string!!!
        std::string folder_name = std::string(dirname(filename_cstr));
        search_paths.emplace(folder_name);
    }

    if (json_document.HasMember("inherits") && json_document["inherits"].IsString())
    {
        std::string child_filename;
        bool found = getDefinitionFile(json_document["inherits"].GetString(), child_filename);
        if (!found)
        {
            cura::logError("Inherited JSON file \"%s\" not found\n", json_document["inherits"].GetString());
            return -1;
        }
        err = loadJSONsettings(child_filename, settings_base, warn_base_file_duplicates); // load child first
        if (err)
        {
            return err;
        }
        err = loadJSONsettingsFromDoc(json_document, settings_base, false);
    }
    else 
    {
        err = loadJSONsettingsFromDoc(json_document, settings_base, warn_base_file_duplicates);
    }

    if (json_document.HasMember("metadata") && json_document["metadata"].IsObject())
    {
        const rapidjson::Value& json_metadata = json_document["metadata"];
        if (json_metadata.HasMember("machine_extruder_trains") && json_metadata["machine_extruder_trains"].IsObject())
        {
            const rapidjson::Value& json_machine_extruder_trains = json_metadata["machine_extruder_trains"];
            for (rapidjson::Value::ConstMemberIterator extr_train_iterator = json_machine_extruder_trains.MemberBegin(); extr_train_iterator != json_machine_extruder_trains.MemberEnd(); ++extr_train_iterator)
            {
                int extruder_train_nr = atoi(extr_train_iterator->name.GetString());
                if (extruder_train_nr < 0)
                {
                    continue;
                }
                const rapidjson::Value& json_id = extr_train_iterator->value;
                if (!json_id.IsString())
                {
                    continue;
                }
                const char* id = json_id.GetString();
                if (extruder_train_nr >= (int) extruder_train_ids.size())
                {
                    extruder_train_ids.resize(extruder_train_nr + 1);
                }
                extruder_train_ids[extruder_train_nr] = std::string(id);
            }
        }
    }

    return err;
}

int SettingRegistry::loadJSONsettingsFromDoc(rapidjson::Document& json_document, SettingsBase* settings_base, bool warn_duplicates)
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
        settings_base->_setSetting(machine_name_setting.getKey(), machine_name_setting.getDefaultValue());
    }

    if (json_document.HasMember("settings"))
    {
        std::list<std::string> path;
        handleChildren(json_document["settings"], path, settings_base, warn_duplicates);
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
                logWarning("Trying to override unknown setting %s.\n", setting.c_str());
                continue;
            }
            _loadSettingValues(conf, override_iterator, settings_base);
        }
    }
    
    return 0;
}

void SettingRegistry::handleChildren(const rapidjson::Value& settings_list, std::list<std::string>& path, SettingsBase* settings_base, bool warn_duplicates)
{
    if (!settings_list.IsObject())
    {
        logError("ERROR: json settings list is not an object!\n");
        return;
    }
    for (rapidjson::Value::ConstMemberIterator setting_iterator = settings_list.MemberBegin(); setting_iterator != settings_list.MemberEnd(); ++setting_iterator)
    {
        handleSetting(setting_iterator, path, settings_base, warn_duplicates);
        if (setting_iterator->value.HasMember("children"))
        {
            std::list<std::string> path_here = path;
            path_here.push_back(setting_iterator->name.GetString());
            handleChildren(setting_iterator->value["children"], path_here, settings_base, warn_duplicates);
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


void SettingRegistry::handleSetting(const rapidjson::Value::ConstMemberIterator& json_setting_it, std::list<std::string>& path, SettingsBase* settings_base, bool warn_duplicates)
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
        setting_key_to_config[name] = nullptr; // add the category name to the mapping, but don't instantiate a setting config for it.
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
        
        SettingConfig* setting = getSettingConfig(name);
        if (warn_duplicates && setting)
        {
            cura::logError("Duplicate definition of setting: %s a.k.a. \"%s\" was already claimed by \"%s\"\n", name.c_str(), label.c_str(), getSettingConfig(name)->getLabel().c_str());
        }
        if (!setting)
        {
            setting = &addSetting(name, label);
        }
        _loadSettingValues(setting, json_setting_it, settings_base);
    }
    else
    {
        setting_key_to_config[name] = nullptr; // add the setting name to the mapping, but don't instantiate a setting config for it.
    }
}

SettingConfig& SettingRegistry::addSetting(std::string name, std::string label)
{
    SettingConfig* config = setting_definitions.addChild(name, label);

    setting_key_to_config[name] = config;

    return *config;
}

void SettingRegistry::loadDefault(const rapidjson::GenericValue< rapidjson::UTF8< char > >::ConstMemberIterator& json_object_it, SettingConfig* config)
{
    const rapidjson::Value& setting_content = json_object_it->value;
    if (setting_content.HasMember("default_value"))
    {
        const rapidjson::Value& dflt = setting_content["default_value"];
        if (dflt.IsString())
        {
            config->setDefault(dflt.GetString());
        }
        else if (dflt.IsTrue())
        {
            config->setDefault("true");
        }
        else if (dflt.IsFalse())
        {
            config->setDefault("false");
        }
        else if (dflt.IsNumber())
        {
            std::ostringstream ss;
            ss << dflt.GetDouble();
            config->setDefault(ss.str());
        } // arrays are ignored because machine_extruder_trains needs to be handled separately
        else 
        {
            logWarning("WARNING: Unrecognized data type in JSON: %s has type %s\n", json_object_it->name.GetString(), toString(dflt.GetType()).c_str());
        }
    }
}


void SettingRegistry::_loadSettingValues(SettingConfig* config, const rapidjson::GenericValue< rapidjson::UTF8< char > >::ConstMemberIterator& json_object_it, SettingsBase* settings_base)
{
    const rapidjson::Value& data = json_object_it->value;
    /// Fill the setting config object with data we have in the json file.
    if (data.HasMember("type") && data["type"].IsString())
    {
        config->setType(data["type"].GetString());
    }
    if (config->getType() == std::string("polygon") || config->getType() == std::string("polygons"))
    { // skip polygon settings : not implemented yet and not used yet (TODO)
//         logWarning("WARNING: Loading polygon setting %s not implemented...\n", json_object_it->name.GetString());
        return;
    }

    loadDefault(json_object_it, config);

    if (data.HasMember("unit") && data["unit"].IsString())
    {
        config->setUnit(data["unit"].GetString());
    }

    settings_base->_setSetting(config->getKey(), config->getDefaultValue());
}

}//namespace cura

#ifdef WIN32
#include <windows.h>

char *dirname(char *path)
{
	static char folder_name[MAX_PATH + 1], *p;
	GetFullPathNameA(path, _countof(folder_name), folder_name, &p);
	p[-1] = 0;
	return folder_name;
}
#endif
