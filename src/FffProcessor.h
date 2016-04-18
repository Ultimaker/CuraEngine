#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include "settings.h"
#include "FffGcodeWriter.h"
#include "FffPolygonGenerator.h"
#include "commandSocket.h"
#include "Weaver.h"
#include "Wireframe2gcode.h"
#include "progress/Progress.h"
#include "utils/gettime.h"
#include "utils/NoCopy.h"

#define SHOW_ALL_SETTINGS true

namespace cura {

//FusedFilamentFabrication processor. Singleton class
class FffProcessor : public SettingsBase , NoCopy
{
private:
    /*!
     * The FffProcessor used for the (current) slicing (The instance of this singleton)
     */
    static FffProcessor instance; 
    
    FffProcessor()
    : polygon_generator(this)
    , gcode_writer(this)
    , first_meshgroup(true)
    {
    }
public:
    /*!
     * Get the instance
     * \return The instance
     */
    static FffProcessor* getInstance() 
    { 
        return &instance; 
    }
    
private:
    /*!
     * The polygon generator, which slices the models and generates all polygons to be printed and areas to be filled.
     */
    FffPolygonGenerator polygon_generator;

    /*!
     * The gcode writer, which generates paths in layer plans in a buffer, which converts these paths into gcode commands.
     */
    FffGcodeWriter gcode_writer;

    /*!
     * Whether the firs meshgroup is being processed.
     */
    bool first_meshgroup;

    /*!
     * A string containing all setting values passed to the engine in the format by which CuraEngine is called via the command line.
     * 
     * Used in debugging.
     */
    std::string profile_string = "";

    /*!
     * Get all settings for the current meshgroup in the format by which CuraEngine is called via the command line.
     * 
     * Also includes all global settings if this is the first meshgroup.
     * 
     * Used in debugging.
     * 
     * \param meshgroup The meshgroup for which to stringify all settings
     * \param first_meshgroup Whether this is the first meshgroup and all global settigns should be included as well
     */
    std::string getAllSettingsString(MeshGroup& meshgroup, bool first_meshgroup);

public:
    /*!
     * Get a string containing all setting values passed to the engine in the format by which CuraEngine is called via the command line.
     * 
     * \return A string containing all setting values passed to the engine in the format by which CuraEngine is called via the command line.
     */
    std::string getProfileString() { return profile_string; }

    /*!
     * The stop watch used to time how long the different stages take to compute.
     */
    TimeKeeper time_keeper; // TODO: use singleton time keeper

    /*!
     * Reset the meshgroup number to the first meshgroup to start a new slicing.
     */
    void resetMeshGroupNumber()
    {
        gcode_writer.resetMeshGroupNumber();
    }

    /*!
     * Set the target to write gcode to: to a file.
     * 
     * Used when CuraEngine is used as command line tool.
     * 
     * \param filename The filename of the file to which to write the gcode.
     */
    bool setTargetFile(const char* filename)
    {
        return gcode_writer.setTargetFile(filename);
    }

    /*!
     * Set the target to write gcode to: an output stream.
     * 
     * Used when CuraEngine is NOT used as command line tool.
     * 
     * \param stream The stream to write gcode to.
     */
    void setTargetStream(std::ostream* stream)
    {
        return gcode_writer.setTargetStream(stream);
    }

    /*!
     * Get the total extruded volume for a specific extruder in mm^3
     * 
     * Retractions and unretractions don't contribute to this.
     * 
     * \param extruder_nr The extruder number for which to get the total netto extruded volume
     * \return total filament printed in mm^3
     */
    double getTotalFilamentUsed(int extruder_nr)
    {
        return gcode_writer.getTotalFilamentUsed(extruder_nr);
    }

    /*!
     * Get the total estimated print time in seconds
     * 
     * \return total print time in seconds
     */
    double getTotalPrintTime()
    {
        return gcode_writer.getTotalPrintTime();
    }

    /*!
     * Add the end gcode and set all temperatures to zero.
     */
    void finalize()
    {
        gcode_writer.finalize();
    }

    /*!
     * Process all files into one meshgroup
     * 
     * \warning Unused!
     */
    bool processFiles(const std::vector<std::string> &files);

    /*!
     * Generate gcode for a given \p meshgroup
     * The primary function of this class.
     * 
     * \param meshgroup The meshgroup for which to generate gcode
     * \return Whether this function succeeded
     */
    bool processMeshGroup(MeshGroup* meshgroup);
};

}//namespace cura

#endif//FFF_PROCESSOR_H
