// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine
{
/*!
 * How to enable/disable cooling fan(s) during extruder switch
 */
enum class CoolDuringExtruderSwitch
{
    UNCHANGED, // Let fans as they are during nozzle switch (historical behavior)
    ONLY_LAST_EXTRUDER, // Turn on fan of the previously used extruder to cool it down, turn others off
    ALL_FANS, // Turn on all fans
};

} // namespace curaengine
