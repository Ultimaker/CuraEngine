#ifndef SETTING_REGISTRY_H
#define SETTING_REGISTRY_H

#include <vector>
#include <map>

#include "rapidjson/document.h"

// Forward declaration
class SettingConfig;

/*!
 * Setting category.
 * Filled from the fdmprinter.json file. Contains one or more children settings.
 */
class SettingCategory
{
private:
    std::string label;
    std::string key;
    std::vector<SettingConfig> children;
public:
    SettingCategory(std::string key, std::string label);
    
    SettingConfig* addChild(std::string key, std::string label);
};

/*!
 * Single setting data.
 * Filled from the fdmprinter.json file. Can contain child settings, and is registered in the
 * setting registry with it's key.
 */
class SettingConfig
{
private:
    std::string label;
    std::string key;
    std::string type;
    std::string default_value;
    std::string unit;
    SettingConfig* parent;
    std::vector<SettingConfig> children;
public:
    SettingConfig(std::string key, std::string label, SettingConfig* parent);

    SettingConfig* addChild(std::string key, std::string label);
    
    std::string getKey() const
    {
        return key;
    }
    
    void setType(std::string type)
    {
        this->type = type;
    }
    
    std::string getType() const
    {
        return type;
    }

    void setDefault(std::string default_value)
    {
        this->default_value = default_value;
    }
    
    std::string getDefaultValue() const
    {
        return default_value;
    }

    void setUnit(std::string unit)
    {
        this->unit = unit;
    }
    
    std::string getUnit() const
    {
        return unit;
    }
};

/*!
 * Setting registry.
 * There is a single global setting registry.
 * This registry contains all known setting keys.
 * The registry also contains the settings categories to build up the setting hiarcy from the json file.
 */
class SettingRegistry
{
private:
    static SettingRegistry instance;

    std::map<std::string, SettingConfig*> settings;
    std::vector<SettingCategory> categories;
public:
    static SettingRegistry* getInstance() { return &instance; }
    
    bool settingExists(std::string key) const;
    const SettingConfig* getSettingConfig(std::string key);
    
    bool loadJSON(std::string filename);
private:
    SettingRegistry();
    
    void _addSettingsToCategory(SettingCategory* category, const rapidjson::Value& json_object, SettingConfig* parent);
};

#endif//SETTING_REGISTRY_H
