//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include "FffGcodeWriter.h"
#include "FffPolygonGenerator.h"
#include "utils/gettime.h"
#include "utils/NoCopy.h"


namespace cura {

//FusedFilamentFabrication processor. Singleton class
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
     * Set the target to write gcode to: to a file.
     * 
     * Used when CuraEngine is used as command line tool.
     * 
     * \param filename The filename of the file to which to write the gcode.
     */
    bool setTargetFile(const char* filename);

    /*!
     * Set the target to write gcode to: an output stream.
     * 
     * Used when CuraEngine is NOT used as command line tool.
     * 
     * \param stream The stream to write gcode to.
     */
    void setTargetStream(std::ostream* stream);

    /*!
     * Get the total extruded volume for a specific extruder in mm^3
     * 
     * Retractions and unretractions don't contribute to this.
     * 
     * \param extruder_nr The extruder number for which to get the total netto extruded volume
     * \return total filament printed in mm^3
     */
    double getTotalFilamentUsed(int extruder_nr);

    /*!
     * Get the total estimated print time in seconds for each feature
     * 
     * \return total print time in seconds for each feature
     */
    std::vector<Duration> getTotalPrintTimePerFeature();

    /*!
     * Add the end gcode and set all temperatures to zero.
     */
    void finalize();
};

}//namespace cura

#endif//FFF_PROCESSOR_H
