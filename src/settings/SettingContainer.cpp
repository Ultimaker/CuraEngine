/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "SettingContainer.h"
#include "SettingConfig.h"

#include <sstream>
#include <iostream> // debug IO
#include <libgen.h> // dirname
#include <string>
#include <algorithm> // find_if

namespace cura
{

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


void SettingContainer::debugOutputAllSettings() const
{
    std::cerr << "\nSETTINGS BASE: " << key << std::endl;
    for (const SettingConfig& child : children)
    {
        child.debugOutputAllSettings();
    }
}

}//namespace cura
