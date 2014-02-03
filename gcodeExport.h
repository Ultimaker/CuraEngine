/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#include <stdio.h>

#include "settings.h"
#include "comb.h"
#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "timeEstimate.h"

class GCodeExport
{
private:
    FILE* f;
    double extrusionAmount;
    double extrusionPerMM;
    double retractionAmount;
    double extruderSwitchRetraction;
    double minimalExtrusionBeforeRetraction;
    double extrusionAmountAtPreviousRetraction;
    Point3 currentPosition;
    Point extruderOffset[MAX_EXTRUDERS];
    int currentSpeed, retractionSpeed;
    int zPos;
    bool isRetracted;
    int extruderNr;
    int currentFanSpeed;
    int flavor;
    
    double totalFilament[MAX_EXTRUDERS];
    double totalPrintTime;
    TimeEstimateCalculator estimateCalculator;
public:
    
    GCodeExport();
    
    ~GCodeExport();
    
    void replaceTagInStart(const char* tag, const char* replaceValue);
    
    void setExtruderOffset(int id, Point p);
    
    void setFlavor(int flavor);
    int getFlavor();
    
    void setFilename(const char* filename);
    
    bool isValid();
    
    void setExtrusion(int layerThickness, int filamentDiameter, int flow);
    
    void setRetractionSettings(int retractionAmount, int retractionSpeed, int extruderSwitchRetraction, int minimalExtrusionBeforeRetraction);
    
    void setZ(int z);
    
    Point getPositionXY();
    
    int getPositionZ();

    int getExtruderNr();
    
    double getTotalFilamentUsed(int e);

    double getTotalPrintTime();
    void updateTotalPrintTime();
    
    void addComment(const char* comment, ...);

    void addLine(const char* line, ...);
    
    void resetExtrusionValue();
    
    void addDelay(double timeAmount);
    
    void addMove(Point p, int speed, int lineWidth);
    
    void addRetraction();
    
    void switchExtruder(int newExtruder);
    
    void addCode(const char* str);
    
    void addFanCommand(int speed);

    int getFileSize();
    void tellFileSize();
};

class GCodePathConfig
{
public:
    int speed;
    int lineWidth;
    const char* name;
    bool spiralize;
    int stretchDistance;
    
    GCodePathConfig() : speed(0), lineWidth(0), name(NULL), spiralize(false), stretchDistance(0) {}
    GCodePathConfig(int speed, int lineWidth, int _stretchDistance, const char* name) : speed(speed), lineWidth(lineWidth), name(name), spiralize(false), stretchDistance(_stretchDistance) {}
    
    void setData(int speed, int lineWidth, int _stretchDistance, const char* name)
    {
        this->speed = speed;
        this->lineWidth = lineWidth;
        this->name = name;
        this->stretchDistance = _stretchDistance;
   }
};

class GCodePath
{
public:
    GCodePathConfig* config;
    bool retract;
    int extruder;
    vector<Point> points;
    bool done;//Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.
};

class GCodePlanner
{
private:
    GCodeExport& gcode;
    
    Point lastPosition;
    vector<GCodePath> paths;
    Comb* comb;
    
    GCodePathConfig travelConfig;
    int extrudeSpeedFactor;
    int travelSpeedFactor;
    int currentExtruder;
    int retractionMinimalDistance;
    bool forceRetraction;
    bool alwaysRetract;
    double extraTime;
    double totalPrintTime;
private:
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config);
    void forceNewPathStart();
    void writeStretchedPath(vector<Point>& points, int speed, int lineWidth, int _stretchDistance);
public:
    GCodePlanner(GCodeExport& gcode, int travelSpeed, int retractionMinimalDistance);
    ~GCodePlanner();
    
    bool setExtruder(int extruder)
    {
        if (extruder == currentExtruder)
            return false;
        currentExtruder = extruder;
        return true;
    }
    
    int getExtruder()
    {
        return currentExtruder;
    }

    void setCombBoundary(Polygons* polygons)
    {
        if (comb)
            delete comb;
        if (polygons)
            comb = new Comb(*polygons);
        else
            comb = NULL;
    }
    
    void setAlwaysRetract(bool alwaysRetract)
    {
        this->alwaysRetract = alwaysRetract;
    }
    
    void forceRetract()
    {
        forceRetraction = true;
    }
    
    void setExtrudeSpeedFactor(int speedFactor)
    {
        if (speedFactor < 1) speedFactor = 1;
        this->extrudeSpeedFactor = speedFactor;
    }
    int getExtrudeSpeedFactor()
    {
        return this->extrudeSpeedFactor;
    }
    void setTravelSpeedFactor(int speedFactor)
    {
        if (speedFactor < 1) speedFactor = 1;
        this->travelSpeedFactor = speedFactor;
    }
    int getTravelSpeedFactor()
    {
        return this->travelSpeedFactor;
    }
    
    void addTravel(Point p);
    
    void addExtrusionMove(Point p, GCodePathConfig* config);
    
    void moveInsideCombBoundary(int distance);

    void addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config);

    void addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config);
    
    void forceMinimalLayerTime(double minTime, int minimalSpeed);
    
    void writeGCode(bool liftHeadIfNeeded, int layerThickness);
    
    static Point CircleCenter(Point& A, Point& B, Point& C);
};

#endif//GCODEEXPORT_H
