#include "settingRegistry.h"

#include <sstream>
#include <iostream> // debug IO
#include <libgen.h> // dirname
#include <string>
#include <algorithm> // find_if

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "utils/logoutput.h"

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


SettingContainer::SettingContainer(std::string key, std::string label)
: key(key)
, label(label)
{
}

SettingConfig* SettingContainer::addChild(std::string key, std::string label)
{
    children.emplace_back(key, label);
    return &children.back();
}

SettingConfig& SettingContainer::getOrCreateChild(std::string key, std::string label)
{
    auto child_it = std::find_if(children.begin(), children.end(), [&key](SettingConfig& child) { return child.key == key; } );
    if (child_it == children.end())
    {
        children.emplace_back(key, label);
        return children.back();
    }
    else
    {
        return *child_it;
    }
}


SettingConfig::SettingConfig(std::string key, std::string label)
: SettingContainer(key, label)
{
//     std::cerr << key << std::endl; // debug output to show all frontend registered settings...
}

void SettingContainer::debugOutputAllSettings() const
{
    std::cerr << "\nSETTINGS BASE: " << key << std::endl;
    for (const SettingConfig& child : children)
    {
        child.debugOutputAllSettings();
    }
}


bool SettingRegistry::settingExists(std::string key) const
{
    return settings.find(key) != settings.end();
}

SettingConfig* SettingRegistry::getSettingConfig(std::string key) const
{
    auto it = settings.find(key);
    if (it == settings.end())
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

bool SettingRegistry::settingsLoaded() const
{
    return settings.size() > 0;
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

int SettingRegistry::loadJSONsettings(std::string filename)
{
    rapidjson::Document json_document;
    
    int err = loadJSON(filename, json_document);
    if (err) { return err; }

    log("Loading %s...\n", filename.c_str());

    err = loadJSONsettingsFromDoc(json_document, true);

    if (!err)
    {
        warn_duplicates = false; // Don't warn duplicates for further JSON files loaded
    }

    return err;
}

int SettingRegistry::loadJSONsettingsFromDoc(rapidjson::Document& json_document, bool warn_duplicates)
{
    
    if (!json_document.IsObject())
    {
        cura::logError("JSON file is not an object.\n");
        return 3;
    }

    /*
    if (json_document.HasMember("machine_extruder_trains"))
    {
        const rapidjson::Value& trains = json_document["machine_extruder_trains"];
        SettingContainer& category_trains = getOrCreateCategory("machine_extruder_trains", &trains);

        if (trains.IsObject())
        {
            for (rapidjson::Value::ConstMemberIterator train_iterator = trains.MemberBegin(); train_iterator != trains.MemberEnd(); ++train_iterator)
            {
                SettingConfig& child = category_trains.getOrCreateChild(train_iterator->name.GetString(), std::string("Extruder ") + train_iterator->name.GetString());
                const rapidjson::Value& train = train_iterator->value;
                for (rapidjson::Value::ConstMemberIterator setting_iterator = train.MemberBegin(); setting_iterator != train.MemberEnd(); ++setting_iterator)
                {
                    _addSettingToContainer(&child, setting_iterator, warn_duplicates, false);
                }
            }
        }
    }
    */

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
        handleChildren(json_document["settings"], path);
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
            _loadSettingValues(conf, override_iterator);
        }
    }
    
    return 0;
}

void SettingRegistry::handleChildren(const rapidjson::Value& settings_list, std::list<std::string>& path)
{
    if (!settings_list.IsObject())
    {
        logError("ERROR: json settings list is not an object!\n");
        return;
    }
    for (rapidjson::Value::ConstMemberIterator setting_iterator = settings_list.MemberBegin(); setting_iterator != settings_list.MemberEnd(); ++setting_iterator)
    {
        handleSetting(setting_iterator, path);
        if (setting_iterator->value.HasMember("children"))
        {
            std::list<std::string> path_here = path;
            path_here.push_back(setting_iterator->name.GetString());
            handleChildren(setting_iterator->value["children"], path_here);
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


void SettingRegistry::handleSetting(const rapidjson::Value::ConstMemberIterator& json_setting_it, std::list<std::string>& path)
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
        
        SettingConfig& setting = addSetting(name, label);
        
        _loadSettingValues(&setting, json_setting_it);
    }
}

SettingConfig& SettingRegistry::addSetting(std::__cxx11::string name, std::__cxx11::string label)
{
    SettingConfig* config = setting_definitions.addChild(name, label);

    if (warn_duplicates && settingExists(config->getKey()))
    {
        cura::logError("Duplicate definition of setting: %s a.k.a. \"%s\" was already claimed by \"%s\"\n", config->getKey().c_str(), config->getLabel().c_str(), getSettingConfig(config->getKey())->getLabel().c_str());
    }

    settings[name] = config;

    return *config;
}

void SettingRegistry::_loadSettingValues(SettingConfig* config, const rapidjson::GenericValue< rapidjson::UTF8< char > >::ConstMemberIterator& json_object_it)
{
    const rapidjson::Value& data = json_object_it->value;
    /// Fill the setting config object with data we have in the json file.
    if (data.HasMember("type") && data["type"].IsString())
    {
        config->setType(data["type"].GetString());
    }
    if (data.HasMember("default_value"))
    {
        const rapidjson::Value& dflt = data["default_value"];
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
            if (data.HasMember("type") && data["type"].IsString() && 
                (data["type"].GetString() == std::string("polygon") || data["type"].GetString() == std::string("polygons")))
            {
                logWarning("WARNING: Loading polygon setting %s not implemented...\n", json_object_it->name.GetString());
            }
            else
            {
                logWarning("WARNING: Unrecognized data type in JSON: %s has type %s\n", json_object_it->name.GetString(), toString(dflt.GetType()).c_str());
            }
        }
    }
    if (data.HasMember("unit") && data["unit"].IsString())
    {
        config->setUnit(data["unit"].GetString());
    }
}

}//namespace cura
