// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "FffGcodeWriter.h"
#include "FffPolygonGenerator.h"
#include "utils/NoCopy.h"
#include "utils/TimeKeeper.h"


namespace cura
{

// FusedFilamentFabrication processor. Singleton class
class FffProcessor : public NoCopy
{
private:
    /*!
     * The FffProcessor used for the (current) slicing (The instance of this singleton)
     */
    static FffProcessor instance;

public:
    /*!
     * Get the instance
     * \return The instance
     */
    static FffProcessor* getInstance()
    {
        return &instance;
    }

    /*!
     * The gcode writer, which generates paths in layer plans in a buffer, which converts these paths into gcode commands.
     */
    FffGcodeWriter gcode_writer;

    /*!
     * The polygon generator, which slices the models and generates all polygons to be printed and areas to be filled.
     */
    FffPolygonGenerator polygon_generator;

    /*!
     * The stop watch used to time how long the different stages take to compute.
     */
    TimeKeeper time_keeper; // TODO: use singleton time keeper

    /*!
     * Add the end gcode and set all temperatures to zero.
     */
    void finalize();
};

} // namespace cura
