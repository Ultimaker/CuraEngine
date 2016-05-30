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
        int nozzle_size; //!< The nozzle size label of the nozzle (e.g. 0.4mm; irrespective of tolerances)
        Point nozzle_offset;
        char extruderCharacter;
        std::string start_code;
        std::string end_code;
        double filament_area; //!< in mm^2 for non-volumetric, cylindrical filament

        RetractionConfig extruder_switch_retraction_config; //!< Retraction configuration used when performing extruder switches

        double totalFilament; //!< total filament used per extruder in mm^3
        int currentTemperature;
        int initial_temp; //!< Temperature this nozzle needs to be at the start of the print.

        double retraction_e_amount_current; //!< The current retracted amount (in mm or mm^3), or zero(i.e. false) if it is not currently retracted (positive values mean retracted amount, so negative impact on E values)
        double retraction_e_amount_at_e_start; //!< The ExtruderTrainAttributes::retraction_amount_current value at E0, i.e. the offset (in mm or mm^3) from E0 to the situation where the filament is at the tip of the nozzle.

        double prime_volume; //!< Amount of material (in mm^3) to be primed after an unretration (due to oozing and/or coasting)
        double last_retraction_prime_speed; //!< The last prime speed (in mm/s) of the to-be-primed amount

        std::deque<double> extruded_volume_at_previous_n_retractions; // in mm^3

        ExtruderTrainAttributes()
        : nozzle_offset(0,0)
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
    Point3 machine_dimensions;
    std::string machine_name;

    std::ostream* output_stream;
    std::string new_line;

    double current_e_value; //!< The last E value written to gcode (in mm or mm^3)
    Point3 currentPosition;
    double currentSpeed; //!< The current speed (F values / 60) in mm/s
    double current_acceleration; //!< The current acceleration in the XY direction (in mm/s^2)
    double current_jerk; //!< The current jerk in the XY direction (in mm/s^3)

    int zPos; // TODO: why is this different from currentPosition.z ? zPos is set every layer, while currentPosition.z is set every move. However, the z position is generally not changed within a layer!
    int isZHopped; //!< The amount by which the print head is currently z hopped, or zero if it is not z hopped. (A z hop is used during travel moves to avoid collision with other layer parts)

    int current_extruder;
    int currentFanSpeed;
    EGCodeFlavor flavor;

    double totalPrintTime; //!< The total estimated print time in seconds
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
     * \param mat_ids The material ids for each material.
     * \return The string representing the file header
     */
    std::string getFileHeader(const double* print_time = nullptr, const std::vector<double>& filament_used = std::vector<double>(), const std::vector<int16_t>& mat_ids = std::vector<int16_t>());

    void setLayerNr(unsigned int layer_nr);
    
    void setOutputStream(std::ostream* stream);

    int getNozzleSize(int extruder_idx);

    Point getExtruderOffset(int id);
    
    Point getGcodePos(int64_t x, int64_t y, int extruder_train);
    
    void setFlavor(EGCodeFlavor flavor);
    EGCodeFlavor getFlavor();
    
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
     * Get the total estimated print time in seconds
     * 
     * \return total print time in seconds
     */
    double getTotalPrintTime();
    void updateTotalPrintTime();
    void resetTotalPrintTimeAndFilament();
    
    void writeComment(std::string comment);
    void writeTypeComment(const char* type);
    void writeTypeComment(PrintFeatureType type);
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
    
    void writeMove(Point p, double speed, double extrusion_per_mm);
    
    void writeMove(Point3 p, double speed, double extrusion_per_mm);
private:
    void writeMove(int x, int y, int z, double speed, double extrusion_per_mm);
    /*!
     * The writeMove when flavor == BFB
     */
    void writeMoveBFB(int x, int y, int z, double speed, double extrusion_per_mm);
public:
    void writeRetraction(RetractionConfig* config, bool force = false, bool extruder_switch = false);
    
    void writeRetraction_extruderSwitch();
    
    void switchExtruder(int newExtruder);
    
    void writeCode(const char* str);
    
    /*!
     * Write the gcode for priming the current extruder train so that it can be used.
     */
    void writePrimeTrain();
    
    void writeFanCommand(double speed);
    
    void writeTemperatureCommand(int extruder, double temperature, bool wait = false);
    void writeBedTemperatureCommand(double temperature, bool wait = false);

    /*!
     * Write the command for setting the acceleration to a specific value
     */
    void writeAcceleration(double acceleration);

    /*!
     * Write the command for setting the jerk to a specific value
     */
    void writeJerk(double jerk);

    /*!
     * Set member variables using the settings in \p settings
     * 
     * \param settings The meshgroup to get the global bed temp from and to get the extruder trains from which to get the nozzle temperatures
     */
    void preSetup(MeshGroup* settings);

    /*!
     * Handle the initial (bed/nozzle) temperatures before any gcode is processed.
     * These temperatures are set in the pre-print setup in the firmware.
     * 
     * See FffGcodeWriter::processStartingCode
     * 
     * \param settings The meshgroup to get the global bed temp from and to get the extruder trains from which to get the nozzle temperatures
     */
    void setInitialTemps(const MeshGroup& settings);

    /*!
     * Override or set an initial nozzle temperature as written by GCodeExport::setInitialTemps
     * This is used primarily during better specification of temperatures in LayerPlanBuffer::insertPreheatCommand
     * 
     * \param extruder_nr The extruder number for which to better specify the temp
     * \param temp The temp at which the nozzle should be at startup
     */
    void setInitialTemp(int extruder_nr, double temp);

    void finalize(double moveSpeed, const char* endCode);
};

}

#endif//GCODEEXPORT_H

