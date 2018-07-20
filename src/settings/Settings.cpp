//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include <regex> // regex parsing for temp flow graph
#include <string> //Parsing strings (stod, stoul).
#include "../utils/logoutput.h"

#include "SettingTypes.h" //To overload and return the correct types from settings.
#include "Settings.h"
#include "SettingRegistry.h"

namespace cura
{

void Settings::add(const std::string& key, const std::string value, ExtruderTrain* limit_to_extruder)
{
    settings.insert(std::pair<std::string, Setting>(key, Setting(value, limit_to_extruder)));
}

template<> std::string Settings::get<std::string>(const std::string& key) const
{
    if (settings.find(key) != settings.end())
    {
        Setting setting = settings.at(key);
        //TODO: If the setting has limit_to_extruder set, ask that extruder for the value instead.
        return setting.value;
    }
    else if(parent)
    {
        Setting setting = parent->get<std::string>(key);
        //TODO: If the setting has limit_to_extruder set, ask that extruder for the value instead.
        return setting.value;
    }
    else
    {
        logError("Trying to retrieve unregistered setting with no value given: '%s'\n", key.c_str());
        std::exit(2);
    }
}

template<> int Settings::get<int>(const std::string& key) const
{
    return atoi(get<std::string>(key).c_str());
}

template<> double Settings::get<double>(const std::string& key) const
{
    return atof(get<std::string>(key).c_str());
}

template<> size_t Settings::get<size_t>(const std::string& key) const
{
    return std::stoul(get<std::string>(key).c_str());
}

template<> unsigned int Settings::get<unsigned int>(const std::string& key) const
{
    return get<size_t>(key);
}

template<> bool Settings::get<bool>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "on" || value == "yes" || value == "true" || value == "True")
    {
        return true;
    }
    const int num = atoi(value.c_str());
    return num != 0;
}

template<> DraftShieldHeightLimitation Settings::get<DraftShieldHeightLimitation>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "limited")
    {
        return DraftShieldHeightLimitation::LIMITED;
    }
    else //if (value == "full") or default.
    {
        return DraftShieldHeightLimitation::FULL;
    }
}

template<> FlowTempGraph Settings::get<FlowTempGraph>(const std::string& key) const
{
    std::string value_string = get<std::string>(key);

    FlowTempGraph result;
    if (value_string.empty())
    {
        return result; //Empty at this point.
    }
    /* Match with:
     * - the last opening bracket '['
     * - then a bunch of characters until the first comma
     * - a comma
     * - a bunch of characters until the first closing bracket ']'.
     * This matches with any substring which looks like "[ 124.512 , 124.1 ]".
     */
    std::regex regex("(\\[([^,\\[]*),([^,\\]]*)\\])");

    //Default constructor = end-of-sequence:
    std::regex_token_iterator<std::string::iterator> rend;

    const int submatches[] = {1, 2, 3}; //Match whole pair, first number and second number of a pair.
    std::regex_token_iterator<std::string::iterator> match_iter(value_string.begin(), value_string.end(), regex, submatches);
    while (match_iter != rend)
    {
        match_iter++; //Match the whole pair.
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
            result.data.emplace_back(first, second);
        }
        catch (const std::invalid_argument& e)
        {
            logError("Couldn't read 2D graph element [%s,%s] in setting '%s'. Ignored.\n", first_substring.c_str(), second_substring.c_str(), key.c_str());
        }
    }

    return result;
}

////////////////////////////OLD IMPLEMENTATION BELOW////////////////////////////

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
        case EGCodeFlavor::MARLIN_VOLUMATRIC:
            return "Marlin(Volumetric)";
        case EGCodeFlavor::GRIFFIN:
            return "Griffin";
        case EGCodeFlavor::REPETIER:
            return "Repetier";
        case EGCodeFlavor::REPRAP:
            return "RepRap";
        case EGCodeFlavor::MARLIN:
        default:
            return "Marlin";
    }
}

SettingsBase::SettingsBase()
: SettingsBaseVirtual(nullptr)
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


const std::string& SettingsBase::getSettingString(const std::string& key) const
{
    auto value_it = setting_values.find(key);
    if (value_it != setting_values.end())
    {
        return value_it->second;
    }
    auto inherit_override_it = setting_inherit_base.find(key);
    if (inherit_override_it != setting_inherit_base.end())
    {
        return inherit_override_it->second->getSettingString(key);
    }
    if (parent)
    {
        return parent->getSettingString(key);
    }

    cura::logError("Trying to retrieve unregistered setting with no value given: '%s'\n", key.c_str());
    std::exit(-1);
    static std::string empty_string; // use static object rather than "" to avoid compilation warning
    return empty_string;
}

std::string SettingsBase::getAllLocalSettingsString() const
{
    std::stringstream sstream;
    for (auto pair : setting_values)
    {
        if (!pair.second.empty())
        {
            char buffer[4096];
            snprintf(buffer, 4096, " -s %s=\"%s\"", pair.first.c_str(), Escaped{pair.second.c_str()}.str);
            sstream << buffer;
        }
    }
    return sstream.str();
}

void SettingsMessenger::setSetting(std::string key, std::string value)
{
    parent->setSetting(key, value);
}

void SettingsMessenger::setSettingInheritBase(std::string key, const SettingsBaseVirtual& new_parent)
{
    parent->setSettingInheritBase(key, new_parent);
}


const std::string& SettingsMessenger::getSettingString(const std::string& key) const
{
    return parent->getSettingString(key);
}

}//namespace cura

