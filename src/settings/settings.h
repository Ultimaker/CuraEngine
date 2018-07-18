//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_SETTINGS_H
#define SETTINGS_SETTINGS_H

#include <vector>
#include <map>
#include <unordered_map>
#include <sstream>

#include "SettingsBaseVirtual.h"
#include "../utils/string.h"

namespace cura
{

#ifndef VERSION
#define VERSION "DEV"
#endif

/*!
 * Converts a gcode flavor type to string so that it can be included in the gcode.
 */
std::string toString(EGCodeFlavor flavor);

#define MAX_EXTRUDERS 16

//Maximum number of infill layers that can be combined into a single infill extrusion area.
#define MAX_INFILL_COMBINE 8
    
class SettingsBase;

class SettingRegistry;
/*!
 * Base class for every object that can hold settings.
 * The SettingBase object can hold multiple key-value pairs that define settings.
 * The settings that are set on a SettingBase are checked against the SettingRegistry to ensure keys are valid.
 * Different conversion functions are available for settings to increase code clarity and in the future make
 * unit conversions possible.
 */
class SettingsBase : public SettingsBaseVirtual
{
    friend class SettingRegistry;
private:
    std::unordered_map<std::string, std::string> setting_values;

    /*!
     * Mapping for each setting which must inherit from a different setting base than \ref SettingsBaseVirtual::parent
     */
    std::unordered_map<std::string, const SettingsBaseVirtual*> setting_inherit_base;
public:
    SettingsBase(); //!< SettingsBase without a parent settings object
    SettingsBase(SettingsBaseVirtual* parent); //!< construct a SettingsBase with a parent settings object

    /*!
     * Set a setting to a value.
     * \param key the setting
     * \param value the value
     */
    void setSetting(std::string key, std::string value);
    void setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent); //!< See \ref SettingsBaseVirtual::setSettingInheritBase
    const std::string& getSettingString(const std::string& key) const; //!< Get a setting from this SettingsBase (or any ancestral SettingsBase)

    /*!
     * Format a string that contains all settings and their values similar to a
     * command to call CuraEngine with via CLI.
     *
     * \return A string containing all local settings and their values.
     */
    std::string getAllLocalSettingsString() const;

    void debugOutputAllLocalSettings()  const
    {
        for (auto pair : setting_values)
            std::cerr << pair.first << " : " << pair.second << std::endl;
    }
protected:
    /*!
     * Set a setting without checking if it's registered.
     * 
     * Used in SettingsRegistry
     */
    void _setSetting(std::string key, std::string value);
};

/*!
 * Base class for an object which passes on settings from another object.
 * An object which is a subclass of SettingsMessenger can be handled as a SettingsBase;
 * the difference is that such an object cannot hold any settings, but can only pass on the settings from its parent.
 */
class SettingsMessenger : public SettingsBaseVirtual
{
public:
    SettingsMessenger(SettingsBaseVirtual* parent); //!< construct a SettingsMessenger with a parent settings object
    
    void setSetting(std::string key, std::string value); //!< Set a setting of the parent SettingsBase to a given value
    void setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent); //!< See \ref SettingsBaseVirtual::setSettingInheritBase
    const std::string& getSettingString(const std::string& key) const; //!< Get a setting from the parent SettingsBase (or any further ancestral SettingsBase)
};


}//namespace cura
#endif//SETTINGS_SETTINGS_H

