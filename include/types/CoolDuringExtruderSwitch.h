// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * How to enable/disable cooling fan(s) during extruder switch
 */
enum class CoolDuringExtruderSwitch
{
    Unchanged, // Let fans as they are during nozzle switch (historical behavior)
    OnlyLastExtruder, // Turn on fan of the previously used extruder to cool it down, turn others off
    AllFans, // Turn on all fans
};

} // namespace curaengine::types
