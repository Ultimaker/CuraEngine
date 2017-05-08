/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#include <stdio.h>
#include <deque> // for extrusionAmountAtPreviousRetractions
#include <sstream> // for stream.str()

#include "settings/settings.h"
#include "utils/intpoint.h"
#include "utils/NoCopy.h"
#include "timeEstimate.h"
#include "MeshGroup.h"
#include "commandSocket.h"
#include "RetractionConfig.h"

namespace cura {

/*!
 * Coasting configuration used during printing.
 * Can differ per extruder.
 * 
 * Might be used in the future to have different coasting per feature, e.g. outer wall only.
 */
struct CoastingConfig
{
    bool coasting_enable; //!< Whether coasting is enabled on the extruder to which this config is attached 
    double coasting_volume; //!< The volume leeked when printing without feeding
    double coasting_speed; //!< A modifier (0-1) on the last used travel speed to move slower during coasting
    double coasting_min_volume;  //!< The minimal volume printed to build up enough pressure to leek the coasting_volume
};


//The GCodeExport class writes the actual GCode. This is the only class that knows how GCode looks and feels.
//  Any customizations on GCodes flavors are done in this class.
class GCodeExport : public NoCopy
{
private:
    struct ExtruderTrainAttributes
    {
        Point3 prime_pos; //!< The location this nozzle is primed before printing
        bool prime_pos_is_abs; //!< Whether the prime position is absolute, rather than relative to the last given position
        bool is_primed; //!< Whether this extruder has currently already been primed in this print
        bool use_temp; //!< Whether to insert temperature commands for this extruder

        bool is_used; //!< Whether this extruder train is actually used during the printing of all meshgroups
        int nozzle_size; //!< The nozzle size label of the nozzle (e.g. 0.4mm; irrespective of tolerances)
        Point nozzle_offset;
        char extruderCharacter;
        std::string material_guid; //!< The GUID for the material used by this extruder

        std::string start_code;
        std::string end_code;
        double filament_area; //!< in mm^2 for non-volumetric, cylindrical filament

        double totalFilament; //!< total filament used per extruder in mm^3
        int currentTemperature;
        int initial_temp; //!< Temperature this nozzle needs to be at the start of the print.

        double retraction_e_amount_current; //!< The current retracted amount (in mm or mm^3), or zero(i.e. false) if it is not currently retracted (positive values mean retracted amount, so negative impact on E values)
        double retraction_e_amount_at_e_start; //!< The ExtruderTrainAttributes::retraction_amount_current value at E0, i.e. the offset (in mm or mm^3) from E0 to the situation where the filament is at the tip of the nozzle.

        double prime_volume; //!< Amount of material (in mm^3) to be primed after an unretration (due to oozing and/or coasting)
        double last_retraction_prime_speed; //!< The last prime speed (in mm/s) of the to-be-primed amount

        std::deque<double> extruded_volume_at_previous_n_retractions; // in mm^3

        ExtruderTrainAttributes()
        : prime_pos(0, 0, 0)
        , prime_pos_is_abs(false)
        , is_primed(false)
        , is_used(false)
        , nozzle_offset(0,0)
        , extruderCharacter(0)
        , start_code("")
        , end_code("")
        , filament_area(0)
        , totalFilament(0)
        , currentTemperature(0)
        , initial_temp(0)
        , retraction_e_amount_current(0.0)
        , retraction_e_amount_at_e_start(0.0)
        , prime_volume(0.0)
        , last_retraction_prime_speed(0.0)
        { }
    };
    ExtruderTrainAttributes extruder_attr[MAX_EXTRUDERS];
    unsigned int extruder_count;
    bool use_extruder_offset_to_offset_coords;
    std::string machine_name;

    std::ostream* output_stream;
    std::string new_line;

    double current_e_value; //!< The last E value written to gcode (in mm or mm^3)
    Point3 currentPosition; //!< The last build plate coordinates written to gcode (which might be different from actually written gcode coordinates when the extruder offset is encoded in the gcode)
    double currentSpeed; //!< The current speed (F values / 60) in mm/s
    double current_acceleration; //!< The current acceleration in the XY direction (in mm/s^2)
    double current_travel_acceleration; //!< The current acceleration in the XY direction used for travel moves if different from current_acceleration (in mm/s^2) (Only used for Repetier flavor)
    double current_jerk; //!< The current jerk in the XY direction (in mm/s^3)
    double current_max_z_feedrate; //!< The current max z speed

    AABB3D total_bounding_box; //!< The bounding box of all g-code.

    /*!
     * The z position to be used on the next xy move, if the head wasn't in the correct z position yet.
     * 
     * \see GCodeExport::writeExtrusion(Point, double, double)
     * 
     * \note After GCodeExport::writeExtrusion(Point, double, double) has been called currentPosition.z coincides with this value
     */
    int current_layer_z;
    int isZHopped; //!< The amount by which the print head is currently z hopped, or zero if it is not z hopped. (A z hop is used during travel moves to avoid collision with other layer parts)

    int current_extruder;
    int currentFanSpeed;
    EGCodeFlavor flavor;

    std::vector<double> total_print_times; //!< The total estimated print time in seconds for each feature
    TimeEstimateCalculator estimateCalculator;
    
    bool is_volumatric;
    bool firmware_retract; //!< whether retractions are done in the firmware, or hardcoded in E values.

    unsigned int layer_nr; //!< for sending travel data

    int initial_bed_temp; //!< bed temperature at the beginning of the print.
protected:
    /*!
     * Convert an E value to a value in mm (if it wasn't already in mm) for the current extruder.
     * 
     * E values are either in mm or in mm^3
     * The current extruder is used to determine the filament area to make the conversion.
     * 
     * \param e the value to convert
     * \return the value converted to mm
     */
    double eToMm(double e);

    /*!
     * Convert a volume value to an E value (which might be volumetric as well) for the current extruder.
     * 
     * E values are either in mm or in mm^3
     * The current extruder is used to determine the filament area to make the conversion.
     * 
     * \param mm3 the value to convert
     * \return the value converted to mm or mm3 depending on whether the E axis is volumetric
     */
    double mm3ToE(double mm3);

    /*!
     * Convert a distance value to an E value (which might be linear/distance based as well) for the current extruder.
     * 
     * E values are either in mm or in mm^3
     * The current extruder is used to determine the filament area to make the conversion.
     * 
     * \param mm the value to convert
     * \return the value converted to mm or mm3 depending on whether the E axis is volumetric
     */
    double mmToE(double mm);

public:
    
    GCodeExport();
    ~GCodeExport();

    /*!
     * Get the gcode file header (e.g. ";FLAVOR:UltiGCode\n")
     * 
     * \param print_time The total print time in seconds of the whole gcode (if known)
     * \param filament_used The total mm^3 filament used for each extruder or a vector of the wrong size of unknown
     * \param mat_ids The material GUIDs for each material.
     * \return The string representing the file header
     */
    std::string getFileHeader(const double* print_time = nullptr, const std::vector<double>& filament_used = std::vector<double>(), const std::vector<std::string>& mat_ids = std::vector<std::string>());

    void setLayerNr(unsigned int layer_nr);
    
    void setOutputStream(std::ostream* stream);

    bool getExtruderUsesTemp(const int extruder_nr) const; //!< Returns whether the extruder with the given index uses temperature control, i.e. whether temperature commands will be included for this extruder

    bool getExtruderIsUsed(const int extruder_nr) const; //!< Returns whether the extruder with the given index is used up until the current meshgroup

    int getNozzleSize(const int extruder_nr) const;

    Point getExtruderOffset(const int id) const;

    std::string getMaterialGUID(const int extruder_nr) const; //!< returns the GUID of the material used for the nozzle with id \p extruder_nr

    Point getGcodePos(const int64_t x, const int64_t y, const int extruder_train) const;
    
    void setFlavor(EGCodeFlavor flavor);
    EGCodeFlavor getFlavor() const;
    
    void setZ(int z);
    
    void addLastCoastedVolume(double last_coasted_volume) 
    {
        extruder_attr[current_extruder].prime_volume += last_coasted_volume; 
    }
    
    Point3 getPosition();
    
    Point getPositionXY();

    int getPositionZ();

    int getExtruderNr();
    
    void setFilamentDiameter(unsigned int n, int diameter);
    
    double getCurrentExtrudedVolume();

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
    std::vector<double> getTotalPrintTimes();
    /*!
     * Get the total print time in seconds for the complete print
     * 
     * \return total print time in seconds for the complete print
     */
    double getSumTotalPrintTimes();
    void updateTotalPrintTime();
    void resetTotalPrintTimeAndFilament();
    
    void writeComment(std::string comment);
    void writeTypeComment(PrintFeatureType type);

    /*!
     * Write a comment saying what (estimated) time has passed up to this point
     * 
     * \param time The time passed up till this point
     */
    void writeTimeComment(const double time);
    void writeLayerComment(int layer_nr);
    void writeLayerCountComment(int layer_count);
    
    void writeLine(const char* line);
    
    /*!
     * Reset the current_e_value to prevent too high E values.
     * 
     * The current extruded volume is added to the current extruder_attr.
     */
    void resetExtrusionValue();
    
    void writeDelay(double timeAmount);

    /*!
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     * 
     * \param p location to go to
     * \param speed movement speed
     */
    void writeTravel(Point p, double speed);

    /*!
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     * 
     * \param p location to go to
     * \param speed movement speed
     * \param feature the feature that's currently printing
     */
    void writeExtrusion(Point p, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature);

    /*!
     * Go to a X/Y location with the z-hopped Z value
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     * 
     * \param p location to go to
     * \param speed movement speed
     */
    void writeTravel(Point3 p, double speed);

    /*!
     * Go to a X/Y location with the extrusion Z
     * Perform un-z-hop
     * Perform unretraction
     * 
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     * 
     * \param p location to go to
     * \param speed movement speed
     * \param feature the feature that's currently printing
     */
    void writeExtrusion(Point3 p, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature);
private:
    /*!
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     * 
     * \param x build plate x
     * \param y build plate y
     * \param z build plate z
     * \param speed movement speed
     */
    void writeTravel(int x, int y, int z, double speed);

    /*!
     * Perform un-z-hop
     * Perform unretract
     * Write extrusion move
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     * 
     * \param x build plate x
     * \param y build plate y
     * \param z build plate z
     * \param speed movement speed
     * \param extrusion_mm3_per_mm flow
     * \param feature the print feature that's currently printing
     */
    void writeExtrusion(int x, int y, int z, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature);

    /*!
     * Write the F, X, Y, Z and E value (if they are not different from the last)
     * 
     * convenience function called from writeExtrusion and writeTravel
     * 
     * This function also applies the gcode offset by calling \ref GCodeExport::getGcodePos
     * This function updates the \ref GCodeExport::total_bounding_box
     * It estimates the time in \ref GCodeExport::estimateCalculator for the correct feature
     * It updates \ref GCodeExport::currentPosition, \ref GCodeExport::current_e_value and \ref GCodeExport::currentSpeed
     */
    void writeFXYZE(double speed, int x, int y, int z, double e, PrintFeatureType feature);

    /*!
     * The writeTravel and/or writeExtrusion when flavor == BFB
     * \param x build plate x
     * \param y build plate y
     * \param z build plate z
     * \param speed movement speed
     * \param extrusion_mm3_per_mm flow
     * \param feature print feature to track print time for
     */
    void writeMoveBFB(int x, int y, int z, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature);
public:
    /*!
     * Get ready for extrusion moves:
     * - unretract (G11 or G1 E.)
     * - prime poop (G1 E)
     * 
     * It estimates the time in \ref GCodeExport::estimateCalculator
     * It updates \ref GCodeExport::current_e_value and \ref GCodeExport::currentSpeed
     */
    void writeUnretractionAndPrime();
    void writeRetraction(const RetractionConfig& config, bool force = false, bool extruder_switch = false);

    /*!
     * Start a z hop with the given \p hop_height
     * 
     * \param hop_height The height to move above the current layer
     */
    void writeZhopStart(int hop_height);

    /*!
     * End a z hop: go back to the layer height
     * 
     */
    void writeZhopEnd();

    /*!
     * Start the new_extruder: 
     * - set new extruder
     * - zero E value
     * - write extruder start gcode
     * 
     * \param new_extruder The extruder to start with
     */
    void startExtruder(int new_extruder);

    /*!
     * Switch to the new_extruder: 
     * - perform neccesary retractions
     * - fiddle with E-values
     * - write extruder end gcode
     * - set new extruder
     * - write extruder start gcode
     * 
     * \param new_extruder The extruder to switch to
     * \param retraction_config_old_extruder The extruder switch retraction config of the old extruder, to perform the extruder switch retraction with.
     */
    void switchExtruder(int new_extruder, const RetractionConfig& retraction_config_old_extruder);

    void writeCode(const char* str);
    
    /*!
     * Write the gcode for priming the current extruder train so that it can be used.
     * 
     * \param travel_speed The travel speed when priming involves a movement
     */
    void writePrimeTrain(double travel_speed);
    
    void writeFanCommand(double speed);
    
    void writeTemperatureCommand(int extruder, double temperature, bool wait = false);
    void writeBedTemperatureCommand(double temperature, bool wait = false);

    /*!
     * Write the command for setting the acceleration to a specific value
     */
    void writeAcceleration(double acceleration, bool for_travel_moves = false);

    /*!
     * Write the command for setting the jerk to a specific value
     */
    void writeJerk(double jerk);

    /*!
     * Write the command for setting the maximum z feedrate to a specific value
     */
    void writeMaxZFeedrate(double max_z_feedrate);

    /*!
     * Get the last set max z feedrate value sent in the gcode.
     * 
     * Returns a value <= 0 when no value is set.
     */
    double getCurrentMaxZFeedrate();

    /*!
     * Set member variables using the settings in \p settings
     * 
     * \param settings The meshgroup to get the global bed temp from and to get the extruder trains from which to get the nozzle temperatures
     */
    void preSetup(const MeshGroup* settings);

    /*!
     * Handle the initial (bed/nozzle) temperatures before any gcode is processed.
     * These temperatures are set in the pre-print setup in the firmware.
     * 
     * See FffGcodeWriter::processStartingCode
     * 
     * \param settings The meshgroup to get the global bed temp from and to get the extruder trains from which to get the nozzle temperatures
     * \param start_extruder_nr The extruder with which to start this print
     */
    void setInitialTemps(const MeshGroup& settings, const unsigned int start_extruder_nr);

    /*!
     * Override or set an initial nozzle temperature as written by GCodeExport::setInitialTemps
     * This is used primarily during better specification of temperatures in LayerPlanBuffer::insertPreheatCommand
     * 
     * \warning This function must be called before any of the layers in the meshgroup are written to file!
     * That's because it sets the current temperature in the gcode!
     * 
     * \param extruder_nr The extruder number for which to better specify the temp
     * \param temp The temp at which the nozzle should be at startup
     */
    void setInitialTemp(int extruder_nr, double temp);

    /*!
     * Finish the gcode: turn fans off, write end gcode and flush all gcode left in the buffer.
     * 
     * \param endCode The end gcode to be appended at the very end.
     */
    void finalize(const char* endCode);

};

}

#endif//GCODEEXPORT_H

