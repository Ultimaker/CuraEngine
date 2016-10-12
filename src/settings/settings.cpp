/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include <regex> // regex parsing for temp flow graph
#include <string> // stod (string to double)
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
        case EGCodeFlavor::REPETIER:
            return "Repetier";
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
        cura::logWarning("Setting an unregistered setting %s to %s\n", key.c_str(), value.c_str());
        _setSetting(key, value); // Handy when programmers are in the process of introducing a new setting
    }
}

void SettingsBase::setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent)
{
    setting_inherit_base.emplace(key, &parent);
}


std::string SettingsBase::getSettingString(std::string key) const
{
    if (setting_values.find(key) != setting_values.end())
    {
        return setting_values.at(key);
    }
    if (setting_inherit_base.find(key) != setting_inherit_base.end())
    {
        return setting_inherit_base.at(key)->getSettingString(key);
    }
    if (parent)
    {
        return parent->getSettingString(key);
    }

    const_cast<SettingsBase&>(*this).setting_values[key] = "";
    cura::logWarning("Unregistered setting %s\n", key.c_str());
    return "";
}

void SettingsMessenger::setSetting(std::string key, std::string value)
{
    parent->setSetting(key, value);
}

void SettingsMessenger::setSettingInheritBase(std::string key, const SettingsBaseVirtual& new_parent)
{
    parent->setSettingInheritBase(key, new_parent);
}


std::string SettingsMessenger::getSettingString(std::string key) const
{
    return parent->getSettingString(key);
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

unsigned int SettingsBaseVirtual::getSettingAsLayerNumber(std::string key) const
{
    const unsigned int indicated_layer_number = stoul(getSettingString(key));
    if (indicated_layer_number < 1) //Input checking: Layer 0 is not allowed.
    {
        cura::logWarning("Invalid layer number %i for setting %s.", indicated_layer_number, key.c_str());
        return 0; //Assume layer 1.
    }
    return indicated_layer_number - 1; //Input starts counting at layer 1, but engine code starts counting at layer 0.
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

double SettingsBaseVirtual::getSettingInAngleDegrees(std::string key) const
{
    std::string value = getSettingString(key);
    return atof(value.c_str());
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
    return std::max(0.0, atof(value.c_str()));
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

DraftShieldHeightLimitation SettingsBaseVirtual::getSettingAsDraftShieldHeightLimitation(const std::string key) const
{
    const std::string value = getSettingString(key);
    if (value == "full")
    {
        return DraftShieldHeightLimitation::FULL;
    }
    else if (value == "limited")
    {
        return DraftShieldHeightLimitation::LIMITED;
    }
    return DraftShieldHeightLimitation::FULL; //Default.
}

FlowTempGraph SettingsBaseVirtual::getSettingAsFlowTempGraph(std::string key) const
{
    FlowTempGraph ret;
    std::string value_string = getSettingString(key);
    if (value_string.empty())
    {
        return ret; //Empty at this point.
    }
    std::regex regex("(\\[([^,\\[]*),([^,\\]]*)\\])");
    // match with:
    // - the last opening bracket '['
    // - then a bunch of characters until the first comma
    // - a comma
    // - a bunch of cahracters until the first closing bracket ']'
    // matches with any substring which looks like "[  124.512 , 124.1 ]"

    // default constructor = end-of-sequence:
    std::regex_token_iterator<std::string::iterator> rend;

    int submatches[] = { 1, 2, 3 }; // match whole pair, first number and second number of a pair
    std::regex_token_iterator<std::string::iterator> match_iter(value_string.begin(), value_string.end(), regex, submatches);
    while (match_iter != rend)
    {
        match_iter++; // match the whole pair
        if (match_iter == rend)
        {
            break;
        }
        std::string first_substring = *match_iter++;
        std::string second_substring = *match_iter++;
        try
        {
            double first = std::stod(first_substring);
            double second = std::stod(second_substring);
            ret.data.emplace_back(first, second);
        }
        catch (const std::invalid_argument& e)
        {
            logError("Couldn't read 2D graph element [%s,%s] in setting '%s'. Ignored.\n", first_substring.c_str(), second_substring.c_str(), key.c_str());
        }
    }
    return ret;
}

FMatrix3x3 SettingsBaseVirtual::getSettingAsPointMatrix(std::string key) const
{
    FMatrix3x3 ret;

    std::string value_string = getSettingString(key);
    if (value_string.empty())
    {
        return ret; // standard matrix ([1,0,0],[0,1,0],[0,0,1])
    }

    std::string num("([^,\\] ]*)"); // match with anything but the next ',' ']' or space  and capture the match
    std::ostringstream row; // match with "[num,num,num]" and ignore whitespace
    row << "\\s*\\[\\s*" << num << "\\s*,\\s*" << num << "\\s*,\\s*" << num << "\\s*\\]\\s*";

    std::ostringstream matrix; // match with "[row,row,row]" and ignore whitespace
    matrix << "\\s*\\[" << row.str() << "\\s*,\\s*" << row.str() << "\\s*,\\s*" << row.str() << "\\]\\s*";

    std::regex point_matrix_regex(matrix.str());
    std::cmatch sub_matches;    // same as std::match_results<const char*> cm;
    std::regex_match(value_string.c_str(), sub_matches, point_matrix_regex);

    if (sub_matches.size() != 10) // one match for the whole string
    {
        logWarning("Mesh transformation matrix could not be parsed!\n\tFormat should be [[f,f,f],[f,f,f],[f,f,f]] allowing whitespace anywhere in between.\n\tWhile what was given was \"%s\".\n", value_string.c_str());
        return ret; // standard matrix ([1,0,0],[0,1,0],[0,0,1])
    }

    unsigned int sub_match_idx = 1; // skip the first because the first submatch is the whole string
    for (unsigned int x = 0; x < 3; x++)
    {
        for (unsigned int y = 0; y < 3; y++)
        {
            std::sub_match<const char*> sub_match = sub_matches[sub_match_idx];
            ret.m[y][x] = strtod(std::string(sub_match.str()).c_str(), nullptr);
            sub_match_idx++;
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
    else if (value == "Repetier")
        return EGCodeFlavor::REPETIER;
    return EGCodeFlavor::REPRAP;
}

EFillMethod SettingsBaseVirtual::getSettingAsFillMethod(std::string key) const
{
    std::string value = getSettingString(key);
    if (value == "lines")
        return EFillMethod::LINES;
    if (value == "grid")
        return EFillMethod::GRID;
    if (value == "cubic")
        return EFillMethod::CUBIC;
    if (value == "tetrahedral")
        return EFillMethod::TETRAHEDRAL;
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
    if (value == "skirt")
    	return EPlatformAdhesion::SKIRT;
    return EPlatformAdhesion::NONE;
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

