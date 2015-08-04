#include "settingRegistry.h"

#include <sstream>
#include <iostream> // debug IO
#include "utils/logoutput.h"

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "utils/logoutput.h"

namespace cura
{
    
SettingRegistry SettingRegistry::instance; // define settingRegistry

bool SettingRegistry::settingExists(std::string key) const
{
    return settings.find(key) != settings.end();
}

const SettingConfig* SettingRegistry::getSettingConfig(std::string key)
{
    if (settings.find(key) == settings.end())
        return nullptr;
    return settings[key];
}

SettingRegistry::SettingRegistry()
{
}

bool SettingRegistry::settingsLoaded()
{
    return settings.size() > 0;
}

int SettingRegistry::loadJSON(std::string filename)
{
    rapidjson::Document json_document;
    
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
            cura::logError("Error(offset %u): %s\n", (unsigned)json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
            return 2;
        }
    }

    if (!json_document.IsObject())
    {
        cura::logError("JSON file is not an object.\n");
        return 3;
    }

    if (json_document.HasMember("machine_settings"))
    {
        categories.emplace_back("machine_settings", "Machine Settings");
        SettingCategory* category_machine_settings = &categories.back();
        _addSettingsToCategory(category_machine_settings, json_document["machine_settings"], NULL);
    }
    
    if (json_document.HasMember("categories"))
    {
        for (rapidjson::Value::ConstMemberIterator category_iterator = json_document["categories"].MemberBegin(); category_iterator != json_document["categories"].MemberEnd(); ++category_iterator)
        {
            if (!category_iterator->value.IsObject())
            {
                continue;
            }
            if (!category_iterator->value.HasMember("label") || !category_iterator->value["label"].IsString())
            {
                continue;
            }
            if (!category_iterator->value.HasMember("settings") || !category_iterator->value["settings"].IsObject())
            {
                continue;
            }
            
            categories.emplace_back(category_iterator->name.GetString(), category_iterator->value["label"].GetString());
            SettingCategory* category = &categories.back();
            
            _addSettingsToCategory(category, category_iterator->value["settings"], NULL);
        }
    }
    
    return 0;
}
#include <string>
void SettingRegistry::_addSettingsToCategory(SettingCategory* category, const rapidjson::Value& json_object, SettingConfig* parent)
{
    for (rapidjson::Value::ConstMemberIterator setting_iterator = json_object.MemberBegin(); setting_iterator != json_object.MemberEnd(); ++setting_iterator)
    {
        const rapidjson::Value& data = setting_iterator->value;
        
        std::string label;
        if (!setting_iterator->value.HasMember("label") || !data["label"].IsString())
        {
            label = "N/A";
        }
        else
        {
            label = data["label"].GetString();
        }

        std::string ff = std::string("machine_extruder_count");
        if (std::string(setting_iterator->name.GetString()).compare(ff) == 0)
            logError("");
        
        /// Create the new setting config object.
        SettingConfig* config;
        if (parent)
            config = parent->addChild(setting_iterator->name.GetString(), label);
        else
            config = category->addChild(setting_iterator->name.GetString(), label);
        
        
        /// Fill the setting config object with data we have in the json file.
        if (data.HasMember("type") && data["type"].IsString())
        {
            config->setType(data["type"].GetString());
        }
        if (data.HasMember("default"))
        {
            const rapidjson::Value& dflt = data["default"];
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
            }
            else if (dflt.IsArray()) 
            {
                if (dflt.Size() > 0 && dflt[0].IsObject())
                {
                    unsigned int idx = 0;
                    for (auto it = dflt.Begin(); it != dflt.End(); ++it)
                    {
                        SettingConfig* child = config->addChild(std::to_string(idx), std::to_string(idx));
                        _addSettingsToCategory(category, *it, child);
                        
                        idx++;
                    }
                }
            }
            else 
            {
                logError("Unrecognized data type in JSON: %i\n", int(dflt.GetType()));
            }
        }
        if (data.HasMember("unit") && data["unit"].IsString())
        {
            config->setUnit(data["unit"].GetString());
        }
        
        /// Register the setting in the settings map lookup.
        if (settingExists(config->getKey()))
        {
            cura::logError("Duplicate definition of setting: %s\n", config->getKey().c_str());
        }
        settings[config->getKey()] = config;

        /// When this setting has children, add those children to this setting.
        if (data.HasMember("children") && data["children"].IsObject())
        {
            _addSettingsToCategory(category, data["children"], config);
        }
    }
}

SettingCategory::SettingCategory(std::string key, std::string label)
: label(label), key(key)
{
}

SettingConfig* SettingCategory::addChild(std::string key, std::string label)
{
    children.emplace_back(key, label, nullptr);
    return &children.back();
}

SettingConfig::SettingConfig(std::string key, std::string label, SettingConfig* parent)
: label(label), key(key), parent(parent)
{
//     std::cerr << key << std::endl; // debug output to show all frontend registered settings...
}

SettingConfig* SettingConfig::addChild(std::string key, std::string label)
{
    children.emplace_back(key, label, this);
    return &children.back();
}

}//namespace cura