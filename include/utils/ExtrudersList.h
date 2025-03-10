// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "settings/Settings.h"

namespace cura
{

class ExtrudersList
{
public:
    bool contains(const size_t extruder_nr) const
    {
        return extruders_mask_ & (1 << extruder_nr);
    }

    void set(const size_t extruder_nr)
    {
        extruders_mask_ |= (1 << extruder_nr);
    }

    void unset(const size_t extruder_nr)
    {
        extruders_mask_ &= ~(1 << extruder_nr);
    }

private:
    EXTRUDERS_BITMASK_TYPE extruders_mask_{};
};

} // namespace cura
