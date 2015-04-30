#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <vector>

#include "gcodeExport.h"
#include "comb.h"
#include "utils/polygon.h"
#include "utils/logoutput.h"

namespace cura 
{

class GCodePath
{
public:
    GCodePathConfig* config;
    bool retract;
    int extruder;
    std::vector<Point> points;
    bool done;//Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.
};

//The GCodePlanner class stores multiple moves that are planned.
// It facilitates the combing to keep the head inside the print.
// It also keeps track of the print time estimate for this planning so speed adjustments can be made for the minimal-layer-time.
class GCodePlanner
{
private:
    GCodeExport& gcode;

    Point lastPosition;
    std::vector<GCodePath> paths;
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
    
    bool is_volumatric;
private:
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config);
    void forceNewPathStart();
public:
    GCodePlanner(GCodeExport& gcode, RetractionConfig* retraction_config, int travelSpeed, int retractionMinimalDistance);
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
            comb = nullptr;
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

    void addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config);

    void forceMinimalLayerTime(double minTime, int minimalSpeed, double travelTime, double extrusionTime);
    
    void getTimes(double& travelTime, double& extrudeTime);

    void writeGCode(bool liftHeadIfNeeded, int layerThickness);
};

}//namespace cura

#endif//GCODE_PLANNER_H
