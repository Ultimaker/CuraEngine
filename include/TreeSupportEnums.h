//CuraEngine is released under the terms of the AGPLv3 or higher.
#ifndef CURAENGINE_TREESUPPORTENUMS_H
#define CURAENGINE_TREESUPPORTENUMS_H

namespace cura
{

enum class InterfacePreference
{
    INTERFACE_AREA_OVERWRITES_SUPPORT,
    SUPPORT_AREA_OVERWRITES_INTERFACE,
    INTERFACE_LINES_OVERWRITE_SUPPORT,
    SUPPORT_LINES_OVERWRITE_INTERFACE,
    NOTHING
};

enum class RestPreference
{
    BUILDPLATE,
    GRACEFUL
};

enum class AvoidanceType
{
    SLOW,
    FAST_SAFE,
    FAST,
    COLLISION
};

}//end namespace

#endif // CURAENGINE_TREESUPPORTENUMS_H
