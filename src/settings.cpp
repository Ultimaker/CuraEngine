#include <cctype>
#include <fstream>
#include <stdio.h>
#include "utils/logoutput.h"

#include "settings.h"

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
    settings[key] = value;
}

int SettingsBase::getSettingInt(std::string key)
{
    std::string value = getSetting(key);
    return atoi(value.c_str());
}

std::string SettingsBase::getSetting(std::string key)
{
    if (settings.find(key) != settings.end())
    {
        return settings[key];
    }
    if (parent)
        return parent->getSetting(key);
    
    cura::logError("Failed to find settings %s\n", key.c_str());
    return "";
}

void SettingsBase::copySettings(SettingsBase& other)
{
    settings = other.settings;
}
