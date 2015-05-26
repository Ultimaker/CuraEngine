/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#include <stdio.h>
#include <deque> // for extrusionAmountAtPreviousRetractions

#include "settings.h"
#include "utils/intpoint.h"
#include "timeEstimate.h"

namespace cura {

class RetractionConfig
{
public:
    double amount; //!< The amount
    int speed;
    int primeSpeed;
    double primeAmount;
    int zHop;
};

//The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
class GCodePathConfig
{
private:
    int speed;
    int line_width;
    int filament_diameter;
    int flow;
    int layer_thickness;
    double extrusion_volume_per_mm;
    double extrusion_per_mm;
public:
    const char* name;
    bool spiralize;
    RetractionConfig* retraction_config;
    
    GCodePathConfig() : speed(0), line_width(0), extrusion_per_mm(0), name(nullptr), spiralize(false), retraction_config(nullptr) {}
    GCodePathConfig(RetractionConfig* retraction_config, const char* name) : speed(0), line_width(0), extrusion_per_mm(0), name(name), spiralize(false), retraction_config(retraction_config) {}
    
    void setSpeed(int speed)
    {
        this->speed = speed;
    }
    
    void setLineWidth(int line_width)
    {
        this->line_width = line_width;
        calculateExtrusion();
    }
    
    void setLayerHeight(int layer_height)
    {
        this->layer_thickness = layer_height;
        calculateExtrusion();
    }

    void setFilamentDiameter(int diameter)
    {
        this->filament_diameter = diameter;
        calculateExtrusion();
    }

    void setFlow(int flow)
    {
        this->flow = flow;
        calculateExtrusion();
    }
    
    void smoothSpeed(int min_speed, int layer_nr, int max_speed_layer)
    {
        speed = (speed*layer_nr)/max_speed_layer + (min_speed*(max_speed_layer-layer_nr)/max_speed_layer);
    }
    
    //volumatric extrusion means the E values in the final GCode are cubic mm. Else they are in mm filament.
    double getExtrusionPerMM(bool volumatric)
    {
        if (volumatric)
            return extrusion_volume_per_mm;
        return extrusion_per_mm;
    }
    
    int getSpeed()
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
        extrusion_volume_per_mm = INT2MM(line_width) * INT2MM(layer_thickness) * double(flow) / 100.0;
        double filament_area = M_PI * (INT2MM(filament_diameter) / 2.0) * (INT2MM(filament_diameter) / 2.0);
        extrusion_per_mm = extrusion_volume_per_mm / filament_area;
    }
};

//The GCodeExport class writes the actual GCode. This is the only class that knows how GCode looks and feels.
//  Any customizations on GCodes flavors are done in this class.
class GCodeExport
{
private:
    std::ostream* output_stream;
    double extrusion_amount;
    double extruderSwitchRetraction;
    int extruderSwitchRetractionSpeed;
    int extruderSwitchPrimeSpeed;
    double retraction_extrusion_window;
    double retraction_count_max;
    std::deque<double> extrusion_amount_at_previous_n_retractions;
    Point3 currentPosition;
    Point3 startPosition;
    Point extruderOffset[MAX_EXTRUDERS];
    char extruderCharacter[MAX_EXTRUDERS];
    int currentTemperature[MAX_EXTRUDERS];
    int currentSpeed;
    int zPos;
    bool isRetracted;
    bool isZHopped;
    int retractionPrimeSpeed;
    int extruderNr;
    int currentFanSpeed;
    EGCodeFlavor flavor;
    std::string preSwitchExtruderCode[MAX_EXTRUDERS];
    std::string postSwitchExtruderCode[MAX_EXTRUDERS];
    
    double totalFilament[MAX_EXTRUDERS];
    double totalPrintTime;
    TimeEstimateCalculator estimateCalculator;
public:
    
    GCodeExport();
    ~GCodeExport();
    
    void setOutputStream(std::ostream* stream);
    
    void setExtruderOffset(int id, Point p);
    Point getExtruderOffset(int id);
    void setSwitchExtruderCode(int id, std::string preSwitchExtruderCode, std::string postSwitchExtruderCode);
    
    void setFlavor(EGCodeFlavor flavor);
    EGCodeFlavor getFlavor();
        
    void setRetractionSettings(int extruderSwitchRetraction, int extruderSwitchRetractionSpeed, int extruderSwitchPrimeSpeed, int minimalExtrusionBeforeRetraction, int retraction_count_max);
    
    void setZ(int z);
    
    Point3 getPosition();
    
    Point getPositionXY();
    
    void resetStartPosition();

    Point getStartPositionXY();

    int getPositionZ();

    int getExtruderNr();
    
    double getTotalFilamentUsed(int e);

    double getTotalPrintTime();
    void updateTotalPrintTime();
    void resetTotalPrintTimeAndFilament();
    
    void writeComment(std::string comment);
    void writeTypeComment(const char* type);
    void writeLayerComment(int layer_nr);
    
    void writeLine(const char* line);
    
    void resetExtrusionValue();
    
    void writeDelay(double timeAmount);
    
    void writeMove(Point p, int speed, double extrusion_per_mm);
    
    void writeMove(Point3 p, int speed, double extrusion_per_mm);
private:
    void writeMove(int x, int y, int z, int speed, double extrusion_per_mm);
public:
    void writeRetraction(RetractionConfig* config, bool force=false);
    
    void switchExtruder(int newExtruder);
    
    void writeCode(const char* str);
    
    void writeFanCommand(int speed);
    
    void writeTemperatureCommand(int extruder, int temperature, bool wait = false);
    void writeBedTemperatureCommand(int temperature, bool wait = false);
    
    void finalize(int maxObjectHeight, int moveSpeed, const char* endCode);
};

}

#endif//GCODEEXPORT_H
