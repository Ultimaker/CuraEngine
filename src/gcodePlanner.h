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
    
    bool was_combing;
    bool is_going_to_comb;
    Comb* comb;

    RetractionConfig* last_retraction_config;
    
    GCodePathConfig travelConfig;
    double extrudeSpeedFactor;
    double travelSpeedFactor; // TODO: remove this unused var?
    int currentExtruder;
    
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
    GCodePlanner(GCodeExport& gcode, SliceDataStorage& storage, RetractionConfig* retraction_config_travel, double travelSpeed, bool retraction_combing, unsigned int layer_nr, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance);
    ~GCodePlanner();

    void setCombing(bool going_to_comb);
    
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

    /*!
     * Whether the current retracted path is to be an extruder switch retraction.
     * This function is used to avoid a G10 S1 after a G10.
     * 
     * \param path_idx The index of the current retracted path 
     * \return Whether the path should be an extgruder switch retracted path
     */
    bool makeRetractSwitchRetract(unsigned int path_idx);

    /*!
     * Add a travel path to a certain point, retract if needed and when avoiding boundary crossings:
     * avoiding obstacles and comb along the boundary of parts.
     * 
     * \param p The point to travel to
     */
    void addTravel(Point p);
    
    /*!
     * Add a travel path to a certain point and retract if needed.
     * 
     * No combing is performed.
     * 
     * \param p The point to travel to
     * \param path (optional) The travel path to which to add the point \p p
     */
    void addTravel_simple(Point p, GCodePath* path = nullptr);
    
    void addExtrusionMove(Point p, GCodePathConfig* config, float flow = 1.0);

    void addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr);

    void addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr, EZSeamType z_seam_type = EZSeamType::SHORTEST);

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
     * \param extruder_switch_retract (optional) For a coasted path followed by a retraction: whether to retract normally, or do an extruder switch retraction.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(GCodePath& path, GCodePath& path_next, int64_t layerThickness, double coasting_volume, double coasting_speed, double coasting_min_volume, bool extruder_switch_retract = false);
    
    /*!
     * Write a retraction: either an extruder switch retraction or a normal retraction based on the last extrusion paths retraction config.
     * \param path_idx_travel_after Index in GCodePlanner::paths to the travel move before which to do the retraction
     */
    void writeRetraction(unsigned int path_idx_travel_after);
    
    /*!
     * Write a retraction: either an extruder switch retraction or a normal retraction based on the given retraction config.
     * \param extruder_switch_retract Whether to write an extruder switch retract
     * \param retraction_config The config used.
     */
    void writeRetraction(bool extruder_switch_retract, RetractionConfig* retraction_config);
    
    void writeGCode(bool liftHeadIfNeeded, int layerThickness);
    void moveInsideCombBoundary(int arg1);
};

}//namespace cura

#endif//GCODE_PLANNER_H
