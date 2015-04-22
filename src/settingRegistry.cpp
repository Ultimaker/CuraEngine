#include <sstream>
#include "utils/logOutput.h"

#include "settingRegistry.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

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

bool SettingRegistry::loadJSON(std::string filename)
{
    rapidjson::Document json_document;
    
    {
        FILE* f = fopen(filename.c_str(), "rb");
        char read_buffer[4096];
        rapidjson::FileReadStream reader_stream(f, read_buffer, sizeof(read_buffer));
        json_document.ParseStream(reader_stream);
        fclose(f);
    }

    if (!json_document.IsObject())
    {
        return false;
    }
    if (!json_document.HasMember("categories"))
    {
        return false;
    }

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
    
    return true;
}

void SettingRegistry::_addSettingsToCategory(SettingCategory* category, const rapidjson::Value& json_object, SettingConfig* parent)
{
    for (rapidjson::Value::ConstMemberIterator setting_iterator = json_object.MemberBegin(); setting_iterator != json_object.MemberEnd(); ++setting_iterator)
    {
        const rapidjson::Value& data = setting_iterator->value;
        if (!setting_iterator->value.HasMember("label") || !data["label"].IsString())
        {
            continue;
        }

        /// Create the new setting config object.
        SettingConfig* config;
        if (parent)
            config = parent->addChild(setting_iterator->name.GetString(), data["label"].GetString());
        else
            config = category->addChild(setting_iterator->name.GetString(), data["label"].GetString());
        
        
        /// Fill the setting config object with data we have in the json file.
        if (data.HasMember("type") && data["type"].IsString())
        {
            config->setType(data["type"].GetString());
        }
        if (data.HasMember("default"))
        {
            if (data["default"].IsString())
            {
                config->setDefault(data["default"].GetString());
            }
            else if (data["default"].IsTrue())
            {
                config->setDefault("true");
            }
            else if (data["default"].IsNumber())
            {
                std::ostringstream ss;
                ss << data["default"].GetDouble();
                config->setDefault(ss.str());
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
}

SettingConfig* SettingConfig::addChild(std::string key, std::string label)
{
    children.emplace_back(key, label, this);
    return &children.back();
}
