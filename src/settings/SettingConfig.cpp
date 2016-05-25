/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "SettingConfig.h"


namespace cura
{

SettingConfig::SettingConfig(std::string key, std::string label)
: SettingContainer(key, label)
{
//     std::cerr << key << std::endl; // debug output to show all frontend registered settings...
}

}//namespace cura
