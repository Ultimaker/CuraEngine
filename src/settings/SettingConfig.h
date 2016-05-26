/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_SETTING_CONFIG_H
#define SETTINGS_SETTING_CONFIG_H

#include <string>
#include <iostream> // debug out

#include "SettingContainer.h"

#include "../utils/NoCopy.h"
#include "rapidjson/document.h"

namespace cura
{

/*!
 * Single setting data.
 * Filled from the fdmprinter.json file. Can contain child settings, and is registered in the
 * setting registry with it's key.
 */
class SettingConfig : public SettingContainer
{
private:
    std::string type; //!< The type of the default_value, e.g. str, int, bool
    std::string default_value; //!< The default value for this setting
    std::string unit; //!< The unit of the physical quantity in which this setting is measured, e.g. "mm", "mm/s", ""
public:
    SettingConfig(std::string key, std::string label);

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
    
    void debugOutputAllSettings() const
    {
        std::cerr << key <<  "(" << default_value << ")" << std::endl;
        for (const SettingConfig& child : children)
        {
            child.debugOutputAllSettings();
        }
    }
};

}//namespace cura
#endif//SETTINGS_SETTING_CONFIG_H
