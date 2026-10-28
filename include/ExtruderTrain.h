// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "settings/Settings.h"

namespace curaengine
{

class ExtruderTrain
{
public:
    /*
     * \brief Creates a new extruder. This extruder will have no settings but
     * inherits everything from the global settings.
     */
    ExtruderTrain(const size_t extruder_nr, Settings* parent_settings);

    /*
     * \brief The settings that this extruder overwrites.
     */
    Settings settings_;

    /*
     * \brief The position of this extruder.
     *
     * This may be used by g-code commands such as T to indicate to which
     * tool we must switch.
     */
    const size_t extruder_nr_;
};

} // namespace curaengine
