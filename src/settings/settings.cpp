/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include "../utils/logoutput.h"

#include "settings.h"
#include "SettingRegistry.h"

namespace cura
{
//c++11 no longer defines M_PI, so add our own constant.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::string toString(EGCodeFlavor flavor)
{
    switch (flavor)
    {
        case EGCodeFlavor::BFB:
            return "BFB";
        case EGCodeFlavor::MACH3:
            return "Mach3";
        case EGCodeFlavor::MAKERBOT:
            return "Makerbot";
        case EGCodeFlavor::ULTIGCODE:
            return "UltiGCode";
        case EGCodeFlavor::REPRAP_VOLUMATRIC:
            return "RepRap(Volumetric)";
        case EGCodeFlavor::GRIFFIN:
            return "Griffin";
        case EGCodeFlavor::REPRAP:
        default:
            return "RepRap";
    }
}

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

void SettingsBase::_setSetting(std::string key, std::string value)
{
    setting_values[key] = value;
}


void SettingsBase::setSetting(std::string key, std::string value)
{
    if (SettingRegistry::getInstance()->settingExists(key))
    {
        _setSetting(key, value);
    }
    else
    {
        cura::logError("Warning: setting an unregistered setting %s\n", key.c_str() );
        _setSetting(key, value); // Handy when programmers are in the process of introducing a new setting
    }
}

std::string SettingsBase::getSettingString(std::string key) const
{
    if (setting_values.find(key) != setting_values.end())
    {
        return setting_values.at(key);
    }
    if (parent)
    {
        return parent->getSettingString(key);
    }

    SettingsBase& nonConstThis = const_cast<SettingsBase&>(*this);
    if (SettingRegistry::getInstance()->settingExists(key))
    {
        nonConstThis.setting_values[key] = SettingRegistry::getInstance()->getSettingConfig(key)->getDefaultValue();
    }
    else
    {
        nonConstThis.setting_values[key] = "";
        cura::logError("Unregistered setting %s\n", key.c_str());
    }
    return setting_values.at(key);
}

void SettingsMessenger::setSetting(std::string key, std::string value)
{
    parent->setSetting(key, value);
}

std::string SettingsMessenger::getSettingString(std::string key) const
{
    return parent->getSettingString(key);
}


void SettingsBase::setExtruderTrainDefaults(unsigned int extruder_nr)
{
    const SettingContainer* train = SettingRegistry::getInstance()->getExtruderTrain(extruder_nr);

    if (!train)
    {
        // not enough machine_extruder_trains settings present; just use defaults for this train..
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

int SettingsBaseVirtual::getSettingAsIndex(std::string key) const
{
    std::string value = getSettingString(key);
    return atoi(value.c_str());
}

int SettingsBaseVirtual::getSettingAsCount(std::string key) const
{
    std::string value = getSettingString(key);
    return atoi(value.c_str());
}

double SettingsBaseVirtual::getSettingInMillimeters(std::string key) const
{
    std::string value = getSettingString(key);
    return atof(value.c_str());
}

int SettingsBaseVirtual::getSettingInMicrons(std::string key) const
{
    return getSettingInMillimeters(key) * 1000.0;
}

double SettingsBaseVirtual::getSettingInAngleRadians(std::string key) const
{
    std::string value = getSettingString(key);
    return atof(value.c_str()) / 180.0 * M_PI;
}

bool SettingsBaseVirtual::getSettingBoolean(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "on")
        return true;
    if (value == "yes")
        return true;
    if (value == "true" or value == "True") //Python uses "True"
        return true;
    int num = atoi(value.c_str());
    return num != 0;
}

double SettingsBaseVirtual::getSettingInDegreeCelsius(std::string key) const
{
    std::string value = getSettingString(key);
    return atof(value.c_str());
}

double SettingsBaseVirtual::getSettingInMillimetersPerSecond(std::string key) const
{
    std::string value = getSettingString(key);
    return std::max(1.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInCubicMillimeters(std::string key) const
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInPercentage(std::string key) const
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInSeconds(std::string key) const
{
    std::string value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

FlowTempGraph SettingsBaseVirtual::getSettingAsFlowTempGraph(std::string key) const
{
    FlowTempGraph ret;
    const char* c_str = getSettingString(key).c_str();
    char const* char_p = c_str;
    while (*char_p != '[')
    {
        if (*char_p == '\0') //We've reached the end of string without encountering the first opening bracket.
        {
            return ret; //Empty at this point.
        }
        char_p++;
    }
    char_p++; // skip the '['
    for (; *char_p != '\0'; char_p++)
    {
        while (*char_p != '[')
        {
            if (*char_p == '\0') //We've reached the end of string without finding the next opening bracket.
            {
                return ret; //Don't continue parsing this item then. Just stop and return.
            }
            char_p++;
        }
        char_p++; // skip the '['
        char* end;
        double first = strtod(char_p, &end); //If not a valid number, this becomes zero.
        char_p = end;
        while (*char_p != ',')
        {
            if (*char_p == '\0') //We've reached the end of string without finding the comma.
            {
                return ret; //This entry is incomplete.
            }
            char_p++;
        }
        char_p++; // skip the ','
        double second = strtod(char_p, &end); //If not a valid number, this becomes zero.
        ret.data.emplace_back(first, second);
        char_p = end;
        while (*char_p != ']')
        {
            if (*char_p == '\0') //We've reached the end of string without finding the closing bracket.
            {
                return ret; //This entry is probably complete and has been added, but stop searching.
            }
            char_p++;
        }
        char_p++; // skip the ']'
        if (*char_p == ']' || *char_p == '\0')
        {
            break;
        }
    }
    return ret;
}


EGCodeFlavor SettingsBaseVirtual::getSettingAsGCodeFlavor(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "Griffin")
        return EGCodeFlavor::GRIFFIN;
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

EFillMethod SettingsBaseVirtual::getSettingAsFillMethod(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "lines")
        return EFillMethod::LINES;
    if (value == "grid")
        return EFillMethod::GRID;
    if (value == "triangles")
        return EFillMethod::TRIANGLES;
    if (value == "concentric")
        return EFillMethod::CONCENTRIC;
    if (value == "zigzag")
        return EFillMethod::ZIG_ZAG;
    return EFillMethod::NONE;
}

EPlatformAdhesion SettingsBaseVirtual::getSettingAsPlatformAdhesion(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "brim")
        return EPlatformAdhesion::BRIM;
    if (value == "raft")
        return EPlatformAdhesion::RAFT;
    return EPlatformAdhesion::SKIRT;
}

ESupportType SettingsBaseVirtual::getSettingAsSupportType(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "everywhere")
        return ESupportType::EVERYWHERE;
    if (value == "buildplate")
        return ESupportType::PLATFORM_ONLY;
    return ESupportType::NONE;
}

EZSeamType SettingsBaseVirtual::getSettingAsZSeamType(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "random")
        return EZSeamType::RANDOM;
    if (value == "shortest")
        return EZSeamType::SHORTEST;
    if (value == "back")
        return EZSeamType::BACK;
    return EZSeamType::SHORTEST;
}

ESurfaceMode SettingsBaseVirtual::getSettingAsSurfaceMode(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "normal")
        return ESurfaceMode::NORMAL;
    if (value == "surface")
        return ESurfaceMode::SURFACE;
    if (value == "both")
        return ESurfaceMode::BOTH;
    return ESurfaceMode::NORMAL;
}

CombingMode SettingsBaseVirtual::getSettingAsCombingMode(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "off")
    {
        return CombingMode::OFF;
    }
    if (value == "all")
    {
        return CombingMode::ALL;
    }
    if (value == "noskin")
    {
        return CombingMode::NO_SKIN;
    }
    return CombingMode::ALL;
}

SupportDistPriority SettingsBaseVirtual::getSettingAsSupportDistPriority(std::string key)
{
    std::string value = getSettingString(key);
    if (value == "xy_overrides_z")
    {
        return SupportDistPriority::XY_OVERRIDES_Z;
    }
    if (value == "z_overrides_xy")
    {
        return SupportDistPriority::Z_OVERRIDES_XY;
    }
    return SupportDistPriority::XY_OVERRIDES_Z;
}


}//namespace cura

