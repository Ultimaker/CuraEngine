#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include "utils/logoutput.h"

#include "settings.h"
#include "settingRegistry.h"

//c++11 no longer defines M_PI, so add our own constant.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SettingsBase::SettingsBase()
: parent(NULL)
{
}

SettingsBase::SettingsBase(SettingsBase* parent)
: parent(parent)
{
}

void SettingsBase::setSetting(std::string key, std::string value)
{
    if (SettingRegistry::getInstance()->settingExists(key))
    {
        setting_values[key] = value;
    }
    else
    {
        cura::logError("Ignoring unknown setting %s\n", key.c_str() );
    }
}

std::string SettingsBase::getSettingString(std::string key)
{
    if (setting_values.find(key) != setting_values.end())
    {
        return setting_values[key];
    }
    if (parent)
    {
        return parent->getSettingString(key);
    }
    
    if (SettingRegistry::getInstance()->settingExists(key))
    {
        setting_values[key] = SettingRegistry::getInstance()->getSettingConfig(key)->getDefaultValue();
        cura::logError("Using default for: %s = %s\n", key.c_str(), setting_values[key].c_str());
    }
    else
    {
        setting_values[key] = "";
        cura::logError("Unregistered setting %s\n", key.c_str());
    }
    return setting_values[key];
}

bool SettingsBase::hasSetting(std::string key)
{
    if (setting_values.find(key) != setting_values.end())
    {
        return true;
    }
    if (parent)
    {
        return parent->hasSetting(key);
    }
    
    return false;
}

int SettingsBase::getSettingAsIndex(std::string key)
{
    std::string value = getSettingString(key);
    return atoi(value.c_str());
}

int SettingsBase::getSettingAsCount(std::string key)
{
    std::string value = getSettingString(key);
    return atoi(value.c_str());
}

int SettingsBase::getSettingInMicrons(std::string key)
{
    std::string value = getSettingString(key);
    return atof(value.c_str()) * 1000.0;
}

double SettingsBase::getSettingInAngleRadians(std::string key)
{
    std::string value = getSettingString(key);
    return atof(value.c_str()) / 180.0 * M_PI;
}

bool SettingsBase::getSettingBoolean(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "on")
        return true;
    if (value == "yes")
        return true;
    if (value == "true")
        return true;
    return atoi(value.c_str()) != 0;
}

double SettingsBase::getSettingInDegreeCelsius(std::string key)
{
    std::string value = getSettingString(key);
    return atof(value.c_str());
}

double SettingsBase::getSettingInMillimetersPerSecond(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(1.0, atof(value.c_str()));
}

double SettingsBase::getSettingInPercentage(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBase::getSettingInSeconds(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

EGCodeFlavor SettingsBase::getSettingAsGCodeFlavor(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "RepRap")
        return GCODE_FLAVOR_REPRAP;
    else if (value == "UltiGCode")
        return GCODE_FLAVOR_ULTIGCODE;
    else if (value == "Makerbot")
        return GCODE_FLAVOR_MAKERBOT;
    else if (value == "BFB")
        return GCODE_FLAVOR_BFB;
    else if (value == "MACH3")
        return GCODE_FLAVOR_MACH3;
    else if (value == "RepRap (Volumatric)")
        return GCODE_FLAVOR_REPRAP_VOLUMATRIC;
    return GCODE_FLAVOR_REPRAP;
}

EFillMethod SettingsBase::getSettingAsFillMethod(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Lines")
        return Fill_Lines;
    if (value == "Grid")
        return Fill_Grid;
    if (value == "Triangles")
        return Fill_Triangles;
    if (value == "Concentric")
        return Fill_Concentric;
    if (value == "ZigZag")
        return Fill_ZigZag;
    return Fill_None;
}

EPlatformAdhesion SettingsBase::getSettingAsPlatformAdhesion(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Brim")
        return Adhesion_Brim;
    if (value == "Raft")
        return Adhesion_Raft;
    return Adhesion_None;
}

ESupportType SettingsBase::getSettingAsSupportType(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Everywhere")
        return Support_Everywhere;
    if (value == "Touching Buildplate")
        return Support_PlatformOnly;
    return Support_None;
}
