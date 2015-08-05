#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <vector>

#include "gcodeExport.h"
#include "comb.h"
#include "utils/polygon.h"
#include "utils/logoutput.h"
#include "wallOverlap.h"


namespace cura 
{

class SliceDataStorage;

class GCodePath
{
public:
    GCodePathConfig* config; //!< The configuration settings of the path.
    float flow; //!< A type-independent flow configuration (used for wall overlap compensation)
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path.
    int extruder; //!< The extruder used for this path.
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.
    
    double getExtrusionMM3perMM()
    {
        return flow * config->getExtrusionMM3perMM();
    }
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
    SliceDataStorage& storage;

    Point lastPosition;
    std::vector<GCodePath> paths;
    Comb* comb;

    GCodePathConfig travelConfig;
    CoastingConfig& coasting_config;
    double extrudeSpeedFactor;
    double travelSpeedFactor; // TODO: remove this unused var?
    int currentExtruder;
    int retractionMinimalDistance;
    bool alwaysRetract;
    double extraTime;
    double totalPrintTime;
    
private:
    /*!
     * Either create a new path with the given config or return the last path if it already had that config.
     * If GCodePlanner::forceNewPathStart has been called a new path will always be returned.
     * 
     * \param config The config used for the path returned
     * \param flow (optional) A ratio for the extrusion speed
     * \return A path with the given config which is now the last path in GCodePlanner::paths
     */
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config, float flow = 1.0);
    
    /*!
     * Force GCodePlanner::getLatestPathWithConfig to return a new path.
     * 
     * This function is introduced because in some cases 
     * GCodePlanner::getLatestPathWithConfig is called consecutively with the same config pointer, 
     * though the content of the config has changed.
     * 
     * Example cases: 
     * - when changing extruder, the same travel config is used, but its extruder field is changed.
     */
    void forceNewPathStart();
public:
    /*
     * 
     * \param travel_avoid_other_parts Whether to avoid other layer parts when travaeling through air.
     * \param travel_avoid_distance The distance by which to avoid other layer parts when traveling through air.
     */
    GCodePlanner(GCodeExport& gcode, SliceDataStorage& storage, RetractionConfig* retraction_config, CoastingConfig& coasting_config, double travelSpeed, int retractionMinimalDistance, bool retraction_combing, unsigned int layer_nr, int64_t wall_line_width_0, bool travel_avoid_other_parts, int64_t travel_avoid_distance);
    ~GCodePlanner();

    bool setExtruder(int extruder);

    int getExtruder()
    {
        return currentExtruder;
    }

    void setAlwaysRetract(bool alwaysRetract)
    {
        this->alwaysRetract = alwaysRetract;
    }

    void setExtrudeSpeedFactor(double speedFactor)
    {
        if (speedFactor < 1) speedFactor = 1.0;
        this->extrudeSpeedFactor = speedFactor;
    }
    double getExtrudeSpeedFactor()
    {
        return this->extrudeSpeedFactor;
    }
    void setTravelSpeedFactor(double speedFactor)
    {
        if (speedFactor < 1) speedFactor = 1.0;
        this->travelSpeedFactor = speedFactor;
    }
    double getTravelSpeedFactor()
    {
        return this->travelSpeedFactor;
    }

    void addTravel(Point p);

    void addExtrusionMove(Point p, GCodePathConfig* config, float flow = 1.0);

    void addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr);

    void addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr);

    /*!
     * Add lines to the gcode with optimized order.
     * \param polygons The lines
     * \param config The config of the lines
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     */
    void addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config, int wipe_dist = 0);

    void forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrusionTime);
    
    void getTimes(double& travelTime, double& extrudeTime);

    /*!
     * Writes a path to GCode and performs coasting, or returns false if it did nothing.
     * 
     * Coasting replaces the last piece of an extruded path by move commands and uses the oozed material to lay down lines.
     * 
     * \param path_idx The index into GCodePlanner::paths for the next path to be written to GCode.
     * \param layerThickness The height of the current layer.
     * \param coasting_volume_move The volume otherwise leaked during a normal move.
     * \param coasting_speed_move The speed at which to move during move-coasting.
     * \param coasting_min_volume_move The minimal volume a path should have which builds up enough pressure to ooze as much as \p coasting_volume_move.
     * \param coasting_volume_retract The volume otherwise leaked during a retract move.
     * \param coasting_speed_retract The speed at which to move during retract-coasting.
     * \param coasting_min_volume_retract The minimal volume a path should have which builds up enough pressure to ooze as much as \p coasting_volume_retract.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(unsigned int path_idx, int64_t layerThickness, double coasting_volume_move, double coasting_speed_move, double coasting_min_volume_move, double coasting_volume_retract, double coasting_speed_retract, double coasting_min_volume_retract);

    /*!
     * Writes a path to GCode and performs coasting, or returns false if it did nothing.
     * 
     * Coasting replaces the last piece of an extruded path by move commands and uses the oozed material to lay down lines.
     * 
     * Paths shorter than \p coasting_min_volume will use less \p coasting_volume linearly.
     * 
     * \param path The extrusion path to be written to GCode.
     * \param path_next The next travel path to be written to GCode.
     * \param layerThickness The height of the current layer.
     * \param coasting_volume The volume otherwise leaked.
     * \param coasting_speed The speed at which to move during coasting.
     * \param coasting_min_volume The minimal volume a path should have which builds up enough pressure to ooze as much as \p coasting_volume.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(GCodePath& path, GCodePath& path_next, int64_t layerThickness, double coasting_volume, double coasting_speed, double coasting_min_volume);
    
    void writeGCode(bool liftHeadIfNeeded, int layerThickness);
};

}//namespace cura

#endif//GCODE_PLANNER_H
