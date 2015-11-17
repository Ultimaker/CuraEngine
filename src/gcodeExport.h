/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#include <stdio.h>
#include <deque> // for extrusionAmountAtPreviousRetractions
#include <sstream> // for stream.str()

#include "settings.h"
#include "utils/intpoint.h"
#include "timeEstimate.h"
#include "MeshGroup.h"
#include "commandSocket.h"

namespace cura {

struct CoastingConfig
{
    bool coasting_enable; 
    double coasting_volume_move; 
    double coasting_speed_move; 
    double coasting_min_volume_move; 

    double coasting_volume_retract;
    double coasting_speed_retract;
    double coasting_min_volume_retract;
};
    
class RetractionConfig
{
public:
    double distance; //!< The distance retracted (in mm)
    double speed; //!< The speed with which to retract (in mm/s)
    double primeSpeed; //!< the speed with which to unretract (in mm/s)
    double prime_volume; //!< the amount of material primed after unretracting (in mm^3)
    int zHop; //!< the amount with which to lift the head during a retraction-travel
    int retraction_min_travel_distance; //!< 
    double retraction_extrusion_window; //!< in mm
    int retraction_count_max;
};

//The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
class GCodePathConfig
{
private:
    double speed; //!< movement speed (mm/s)
    int line_width; //!< width of the line extruded
    double flow; //!< extrusion flow in %
    int layer_thickness; //!< layer height
    double extrusion_mm3_per_mm;//!< mm^3 filament moved per mm line extruded
public:
    const char* name; //!< name of the feature type
    bool spiralize;
    RetractionConfig *const retraction_config;
    
    // GCodePathConfig() : speed(0), line_width(0), extrusion_mm3_per_mm(0.0), name(nullptr), spiralize(false), retraction_config(nullptr) {}
    GCodePathConfig(RetractionConfig* retraction_config, const char* name) : speed(0), line_width(0), extrusion_mm3_per_mm(0.0), name(name), spiralize(false), retraction_config(retraction_config) {}
    
    /*!
     * Initialize some of the member variables.
     * 
     * Warning! setLayerHeight still has to be called before this object can be used.
     */
    void init(double speed, int line_width, double flow)
    {
        this->speed = speed;
        this->line_width = line_width;
        this->flow = flow;
    }

    /*!
     * Set the layer height and (re)compute the extrusion_per_mm
     */
    void setLayerHeight(int layer_height)
    {
        this->layer_thickness = layer_height;
        calculateExtrusion();
    }
    
    void smoothSpeed(double min_speed, int layer_nr, double max_speed_layer) 
    {
        speed = (speed*layer_nr)/max_speed_layer + (min_speed*(max_speed_layer-layer_nr)/max_speed_layer);
    }
    
    double getExtrusionMM3perMM()
    {
        return extrusion_mm3_per_mm;
    }
    
    /*!
     * Get the movement speed in mm/s
     */
    double getSpeed()
    {
        return speed;
    }
    
    int getLineWidth()
    {
        return line_width;
    }
    
private:
    void calculateExtrusion()
    {
        extrusion_mm3_per_mm = INT2MM(line_width) * INT2MM(layer_thickness) * double(flow) / 100.0;
    }
};

//The GCodeExport class writes the actual GCode. This is the only class that knows how GCode looks and feels.
//  Any customizations on GCodes flavors are done in this class.
class GCodeExport
{
private:
    struct ExtruderTrainAttributes
    {
        Point nozzle_offset;
        char extruderCharacter;
        std::string start_code;
        std::string end_code;
        double filament_area; //!< in mm^2 for non-volumetric, cylindrical filament

        double extruder_switch_retraction_distance; //<! extruder switch retraction distance in mm
        int extruderSwitchRetractionSpeed; //!< extruder switch retraction speed in mm/s
        int extruderSwitchPrimeSpeed; //!< prime speed of extruder switch in mm/s

        double totalFilament; //!< total filament used per extruder in mm^3
        int currentTemperature;

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
        , extruder_switch_retraction_distance(0.0)
        , extruderSwitchRetractionSpeed(0)
        , extruderSwitchPrimeSpeed(0)
        , totalFilament(0)
        , currentTemperature(0)
        , retraction_e_amount_current(0.0)
        , retraction_e_amount_at_e_start(0.0)
        , prime_volume(0.0)
        , last_retraction_prime_speed(1.0)
        { }
    };
    ExtruderTrainAttributes extruder_attr[MAX_EXTRUDERS];
    bool use_extruder_offset_to_offset_coords;
    
    std::ostream* output_stream;
    double current_e_value; //!< The last E value written to gcode (in mm or mm^3)
    Point3 currentPosition;
    double currentSpeed; //!< The current speed (F values / 60) in mm/s
    int zPos; // TODO: why is this different from currentPosition.z ? zPos is set every layer, while currentPosition.z is set every move. However, the z position is generally not changed within a layer!
    int isZHopped; //!< The amount by which the print head is currently z hopped, or zero if it is not z hopped. (A z hop is used during travel moves to avoid collision with other layer parts)

    int current_extruder;
    int currentFanSpeed;
    EGCodeFlavor flavor;

    double totalPrintTime;
    TimeEstimateCalculator estimateCalculator;
    
    bool is_volumatric;
    bool firmware_retract; //!< whether retractions are done in the firmware, or hardcoded in E values.
    
    CommandSocket* commandSocket; //!< for sending travel data
    unsigned int layer_nr; //!< for sending travel data
    
public:
    
    GCodeExport();
    ~GCodeExport();
    
    void setCommandSocketAndLayerNr(CommandSocket* commandSocket, unsigned int layer_nr);
    
    void setOutputStream(std::ostream* stream);
    
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
    
    double getTotalFilamentUsed(int e);

    double getTotalPrintTime();
    void updateTotalPrintTime();
    void resetTotalPrintTimeAndFilament();
    
    void writeComment(std::string comment);
    void writeTypeComment(const char* type);
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
    void writeRetraction(RetractionConfig* config, bool force=false);
    
    void writeRetraction_extruderSwitch();
    
    void switchExtruder(int newExtruder);
    
    void writeCode(const char* str);
    
    void writeFanCommand(double speed);
    
    void writeTemperatureCommand(int extruder, double temperature, bool wait = false);
    void writeBedTemperatureCommand(double temperature, bool wait = false);
    
    void preSetup(MeshGroup* settings)
    {
        for(int n=0; n<settings->getSettingAsCount("machine_extruder_count"); n++)
        {
            ExtruderTrain* train = settings->getExtruderTrain(n);
            setFilamentDiameter(n, train->getSettingInMicrons("material_diameter")); 
            
            extruder_attr[n].nozzle_offset = Point(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
            
            extruder_attr[n].start_code = train->getSettingString("machine_extruder_start_code");
            extruder_attr[n].end_code = train->getSettingString("machine_extruder_end_code");
            
            extruder_attr[n].extruder_switch_retraction_distance = INT2MM(train->getSettingInMicrons("switch_extruder_retraction_amount")); 
            extruder_attr[n].extruderSwitchRetractionSpeed = train->getSettingInMillimetersPerSecond("switch_extruder_retraction_speed");
            extruder_attr[n].extruderSwitchPrimeSpeed = train->getSettingInMillimetersPerSecond("switch_extruder_prime_speed");
        }

        setFlavor(settings->getSettingAsGCodeFlavor("machine_gcode_flavor"));
        use_extruder_offset_to_offset_coords = settings->getSettingBoolean("machine_use_extruder_offset_to_offset_coords");
    }
    void finalize(int maxObjectHeight, double moveSpeed, const char* endCode);
    
};

}

#endif//GCODEEXPORT_H
