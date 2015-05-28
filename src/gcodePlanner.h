#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <vector>

#include "gcodeExport.h"
#include "comb.h"
#include "utils/polygon.h"
#include "utils/logoutput.h"


namespace cura 
{

class SliceDataStorage;

class GCodePath
{
public:
    GCodePathConfig* config; //!< The configuration settings of the path.
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path.
    int extruder; //!< The extruder used for this path.
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.
};

/*! 
 * The GCodePlanner class stores multiple moves that are planned.
 * It facilitates the combing to keep the head inside the print.
 * It also keeps track of the print time estimate for this planning so speed adjustments can be made for the minimal-layer-time.
 */
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
    bool alwaysRetract;
    double extraTime;
    double totalPrintTime;
    
    bool is_volumatric;
private:
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config);
    void forceNewPathStart();
public:
    GCodePlanner(GCodeExport& gcode, SliceDataStorage& storage, RetractionConfig* retraction_config, int travelSpeed, int retractionMinimalDistance, bool retraction_combing, unsigned int layer_nr);
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

    void setAlwaysRetract(bool alwaysRetract)
    {
        this->alwaysRetract = alwaysRetract;
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
