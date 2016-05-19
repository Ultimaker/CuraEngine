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
    std::cerr << "\nCATEGORY: " << key << std::endl;
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

SettingContainer* SettingRegistry::getCategory(std::string key)
{
    for (SettingContainer& cat : categories)
        if (cat.getKey().compare(key) == 0)
            return &cat;
    return nullptr;
}

const SettingContainer* SettingRegistry::getCategory(std::string key) const
{
    for (const SettingContainer& cat : categories)
        if (cat.getKey().compare(key) == 0)
            return &cat;
    return nullptr;
}

SettingContainer& SettingRegistry::getOrCreateCategory(std::string cat_name, const rapidjson::Value* category)
{
    std::list<SettingContainer>::iterator category_found = std::find_if(categories.begin(), categories.end(), [&cat_name](SettingContainer& cat) { return cat.getKey().compare(cat_name) == 0; });
    if (category_found != categories.end())
    { // category is already present; add settings to category
        return *category_found;
    }
    else 
    {
        std::string label = cat_name;
        if (category && category->IsObject() && category->HasMember("label") && (*category)["label"].IsString())
        {
            label = (*category)["label"].GetString();
        }
        categories.emplace_back(cat_name, label);
        return categories.back();
    }
}


SettingRegistry::SettingRegistry()
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

    return loadJSONsettingsFromDoc(json_document, true);
}

int SettingRegistry::loadJSONsettingsFromDoc(rapidjson::Document& json_document, bool warn_duplicates)
{
    
    if (!json_document.IsObject())
    {
        cura::logError("JSON file is not an object.\n");
        return 3;
    }

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
        SettingConfig& machine_name_setting = getOrCreateCategory("machine_settings").getOrCreateChild("machine_name", "Machine Name");
        machine_name_setting.setDefault(machine_name);
        machine_name_setting.setType("string");
        settings["machine_name"] = &machine_name_setting;
    }

    if (json_document.HasMember("categories"))
    {
        for (rapidjson::Value::ConstMemberIterator category_iterator = json_document["categories"].MemberBegin(); category_iterator != json_document["categories"].MemberEnd(); ++category_iterator)
        {
            _addCategory(category_iterator->name.GetString(), category_iterator->value, warn_duplicates);
        }
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
            _loadSettingValues(conf, override_iterator, false);
        }
    }
    
    return 0;
}

void SettingRegistry::_addCategory(std::string cat_name, const rapidjson::Value& fields, bool warn_duplicates)
{
    if (!fields.IsObject())
    {
        return;
    }
    if (!fields.HasMember("settings") || !fields["settings"].IsObject())
    {
        return;
    }
    SettingContainer& category = getOrCreateCategory(cat_name, &fields);
    const rapidjson::Value& json_object_container = fields["settings"];
    for (rapidjson::Value::ConstMemberIterator setting_iterator = json_object_container.MemberBegin(); setting_iterator != json_object_container.MemberEnd(); ++setting_iterator)
    {
        _addSettingToContainer(&category, setting_iterator, warn_duplicates);
    }
}


void SettingRegistry::_addSettingToContainer(SettingContainer* parent, const rapidjson::Value::ConstMemberIterator& json_object_it, bool warn_duplicates, bool add_to_settings)
{
    const rapidjson::Value& data = json_object_it->value;
    
    std::string label;
    if (!json_object_it->value.HasMember("label") || !data["label"].IsString())
    {
        label = "N/A";
    }
    else
    {
        label = data["label"].GetString();
    }

    /// Create the new setting config object.
    SettingConfig& config = parent->getOrCreateChild(json_object_it->name.GetString(), label);

    _loadSettingValues(&config, json_object_it, warn_duplicates, add_to_settings);
}

void SettingRegistry::_loadSettingValues(SettingConfig* config, const rapidjson::GenericValue< rapidjson::UTF8< char > >::ConstMemberIterator& json_object_it, bool warn_duplicates, bool add_to_settings)
{
    const rapidjson::Value& data = json_object_it->value;
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

    /// Register the setting in the settings map lookup.
    if (warn_duplicates && settingExists(config->getKey()))
    {
        cura::logError("Duplicate definition of setting: %s a.k.a. \"%s\" was already claimed by \"%s\"\n", config->getKey().c_str(), config->getLabel().c_str(), getSettingConfig(config->getKey())->getLabel().c_str());
    }

    if (add_to_settings)
    {
        settings[config->getKey()] = config;
    }

    /// When this setting has children, add those children to this setting.
    if (data.HasMember("children") && data["children"].IsObject())
    {
        const rapidjson::Value& json_object_container = data["children"];
        for (rapidjson::Value::ConstMemberIterator setting_iterator = json_object_container.MemberBegin(); setting_iterator != json_object_container.MemberEnd(); ++setting_iterator)
        {
            _addSettingToContainer(config, setting_iterator, warn_duplicates, add_to_settings);
        }
    }
}

}//namespace cura
