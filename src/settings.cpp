#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include "utils/logoutput.h"

#include "settings.h"
#include "settingRegistry.h"

namespace cura
{
//c++11 no longer defines M_PI, so add our own constant.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SettingsBaseVirtual::SettingsBaseVirtual()
: parent(NULL)
{
}

SettingsBaseVirtual::SettingsBaseVirtual(SettingsBaseVirtual* parent)
: parent(parent)
{
}

SettingsBase::SettingsBase()
: SettingsBaseVirtual(NULL)
{
}

SettingsBase::SettingsBase(SettingsBaseVirtual* parent)
: SettingsBaseVirtual(parent)
{
}

SettingsMessenger::SettingsMessenger(SettingsBaseVirtual* parent)
: SettingsBaseVirtual(parent)
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
        cura::logError("Warning: setting an unregistered setting %s\n", key.c_str() );
        setting_values[key] = value; // Handy when programmers are in the process of introducing a new setting
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
    }
    else
    {
        setting_values[key] = "";
        cura::logError("Unregistered setting %s\n", key.c_str());
    }
    return setting_values[key];
}

void SettingsMessenger::setSetting(std::string key, std::string value)
{
    parent->setSetting(key, value);
}

std::string SettingsMessenger::getSettingString(std::string key)
{
    return parent->getSettingString(key);
}


void SettingsBase::setExtruderTrainDefaults(unsigned int extruder_nr)
{
    const SettingContainer* machine_extruder_trains = SettingRegistry::getInstance()->getCategory(std::string("machine_extruder_trains"));
    
    if (!machine_extruder_trains) 
    {
        logWarning("Error: no machine_extruder_trains category found in JSON!\n");
        return;
    }
    
    const SettingConfig* train = machine_extruder_trains->getChild(extruder_nr);
    
    if (!train)
    {
        logError("Not enough extruder trains specified in JSON: %i\n", extruder_nr);
        return;
    }
    
    for (const SettingConfig& setting : train->getChildren())
    {
        if (setting_values.find(setting.getKey()) == setting_values.end())
        {
            setSetting(setting.getKey(), setting.getDefaultValue());
        }
    }
}

int SettingsBaseVirtual::getSettingAsIndex(std::string key)
{
    std::string value = getSettingString(key);
    return atoi(value.c_str());
}

int SettingsBaseVirtual::getSettingAsCount(std::string key)
{
    std::string value = getSettingString(key);
    return atoi(value.c_str());
}

int SettingsBaseVirtual::getSettingInMicrons(std::string key)
{
    std::string value = getSettingString(key);
    return atof(value.c_str()) * 1000.0;
}

double SettingsBaseVirtual::getSettingInAngleRadians(std::string key)
{
    std::string value = getSettingString(key);
    return atof(value.c_str()) / 180.0 * M_PI;
}

bool SettingsBaseVirtual::getSettingBoolean(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "on")
        return true;
    if (value == "yes")
        return true;
    if (value == "true" or value == "True") //Python uses "True"
        return true;
    return atoi(value.c_str()) != 0;
}

double SettingsBaseVirtual::getSettingInDegreeCelsius(std::string key)
{
    std::string value = getSettingString(key);
    return atof(value.c_str());
}

double SettingsBaseVirtual::getSettingInMillimetersPerSecond(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(1.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInCubicMillimeters(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInPercentage(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInSeconds(std::string key)
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

EGCodeFlavor SettingsBaseVirtual::getSettingAsGCodeFlavor(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "RepRap")
        return EGCodeFlavor::REPRAP;
    else if (value == "UltiGCode")
        return EGCodeFlavor::ULTIGCODE;
    else if (value == "Makerbot")
        return EGCodeFlavor::MAKERBOT;
    else if (value == "BFB")
        return EGCodeFlavor::BFB;
    else if (value == "MACH3")
        return EGCodeFlavor::MACH3;
    else if (value == "RepRap (Volumatric)")
        return EGCodeFlavor::REPRAP_VOLUMATRIC;
    return EGCodeFlavor::REPRAP;
}

EFillMethod SettingsBaseVirtual::getSettingAsFillMethod(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Lines")
        return EFillMethod::LINES;
    if (value == "Grid")
        return EFillMethod::GRID;
    if (value == "Triangles")
        return EFillMethod::TRIANGLES;
    if (value == "Concentric")
        return EFillMethod::CONCENTRIC;
    if (value == "ZigZag")
        return EFillMethod::ZIG_ZAG;
    return EFillMethod::NONE;
}

EPlatformAdhesion SettingsBaseVirtual::getSettingAsPlatformAdhesion(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Brim")
        return EPlatformAdhesion::BRIM;
    if (value == "Raft")
        return EPlatformAdhesion::RAFT;
    return EPlatformAdhesion::SKIRT;
}

ESupportType SettingsBaseVirtual::getSettingAsSupportType(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Everywhere")
        return ESupportType::EVERYWHERE;
    if (value == "Touching Buildplate")
        return ESupportType::PLATFORM_ONLY;
    return ESupportType::NONE;
}

EZSeamType SettingsBaseVirtual::getSettingAsZSeamType(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Random")
        return EZSeamType::RANDOM;
    if (value == "Shortest")
        return EZSeamType::SHORTEST;
    if (value == "Back")
        return EZSeamType::BACK;
    return EZSeamType::SHORTEST;
}

ESurfaceMode SettingsBaseVirtual::getSettingAsSurfaceMode(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "Normal")
        return ESurfaceMode::NORMAL;
    if (value == "Surface")
        return ESurfaceMode::SURFACE;
    if (value == "Both")
        return ESurfaceMode::BOTH;
    return ESurfaceMode::NORMAL;
}


}//namespace cura