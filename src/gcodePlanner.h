#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <vector>

#include "gcodeExport.h"
#include "comb.h"
#include "utils/polygon.h"
#include "utils/logoutput.h"
#include "wallOverlap.h"
#include "commandSocket.h"
#include "FanSpeedLayerTime.h"


namespace cura 
{

class SliceDataStorage;

/*!
 * A gcode command to insert before a specific path.
 * 
 * Currently only used for preheat commands
 */
struct Insert
{
    const unsigned int path_idx; //!< The path before which to insert this command
    int extruder; //!< The extruder for which to set the temp
    double temperature; //!< The temperature of the temperature command to insert
    bool wait; //!< Whether to wait for the temperature to be reached
    Insert(unsigned int path_idx, int extruder, double temperature, bool wait)
    : path_idx(path_idx)
    , extruder(extruder)
    , temperature(temperature)
    , wait(wait)
    {}
};

class TimeMaterialEstimates
{
public:
    double extrude_time;
    double travel_time;
    double material;
    
    TimeMaterialEstimates(double extrude_time, double travel_time, double material)
    : extrude_time(extrude_time)
    , travel_time(travel_time)
    , material(material)
    {
    }
    TimeMaterialEstimates()
    : extrude_time(0.0)
    , travel_time(0.0)
    , material(0.0)
    {
    }
    
    void reset() 
    {
        extrude_time = 0.0;
        travel_time = 0.0;
        material = 0.0;
    }
    
    TimeMaterialEstimates operator+(const TimeMaterialEstimates& other)
    {
        return TimeMaterialEstimates(extrude_time+other.extrude_time, travel_time+other.travel_time, material+other.material);
    }
    
    TimeMaterialEstimates& operator+=(const TimeMaterialEstimates& other)
    {
        extrude_time += other.extrude_time;
        travel_time += other.travel_time;
        material += other.material;
        return *this;
    }
    double getTotalTime()
    {
        return extrude_time + travel_time;
    }
};

class GCodePath
{
public:
    GCodePathConfig* config; //!< The configuration settings of the path.
    float flow; //!< A type-independent flow configuration (used for wall overlap compensation)
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path. 
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.
    
    TimeMaterialEstimates estimates; //!< Naive time and material estimates
    
    double getExtrusionMM3perMM()
    {
        return flow * config->getExtrusionMM3perMM();
    }
};

class ExtruderPlan 
{
public:
    std::vector<GCodePath> paths;
    std::list<Insert> inserts;
    
    int extruder; //!< The extruder used for this paths in the current plan.
    
    bool preheat_command_inserted;
    double required_temp;
    
    TimeMaterialEstimates estimates;
    
    ExtruderPlan(int extruder)
    : extruder(extruder)
    , preheat_command_inserted(false)
    , required_temp(-1)
    {
    }
        
    /*!
     * Add a new Insert, constructed with the given arguments
     */
    template<typename... Args>
    void insertCommand(Args&&... contructor_args)
    {
        inserts.emplace_back(contructor_args...);
    }
};

class LayerPlanBuffer; // forward declaration to prevent circular dependency
/*! 
 * The GCodePlanner class stores multiple moves that are planned.
 * It facilitates the combing to keep the head inside the print.
 * It also keeps track of the print time estimate for this planning so speed adjustments can be made for the minimal-layer-time.
 */
class GCodePlanner
{
    friend class LayerPlanBuffer;
private:
    SliceDataStorage& storage;

    CommandSocket* commandSocket;
    
    int layer_nr;
    
    int z; 
    
    Point start_position;
    Point lastPosition;
    
    std::vector<ExtruderPlan> extruder_plans; //!< should always contain at least one ExtruderPlan
    
    bool was_combing;
    bool is_going_to_comb;
    Comb* comb;

    RetractionConfig* last_retraction_config;
    
    FanSpeedLayerTimeSettings& fan_speed_layer_time_settings;
    
    GCodePathConfig travelConfig; //!< The config used for travel moves (only the speed and retraction config are set!)
    
    GCodePathConfig* raft_config; //!< The config for printing the raft on this layer. Owned (and deleted) by this layer plan.
    
    double extrudeSpeedFactor;
    double travelSpeedFactor; // TODO: remove this unused var?
    
    double fan_speed;
    
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
    /*!
     * 
     * \param travel_avoid_other_parts Whether to avoid other layer parts when travaeling through air.
     * \param travel_avoid_distance The distance by which to avoid other layer parts when traveling through air.
     * \param last_position The position of the head at the start of this gcode layer
     */
    GCodePlanner(CommandSocket* commandSocket, SliceDataStorage& storage, unsigned int layer_nr, int z, Point last_position, int current_extruder, RetractionConfig* retraction_config_travel, FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, double travelSpeed, bool retraction_combing, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance);
    ~GCodePlanner();

    int getLayerNr()
    {
        return layer_nr;
    }
    
    Point getLastPosition()
    {
        return lastPosition;
    }
    
    void setCombing(bool going_to_comb);
    
    bool setExtruder(int extruder);

    int getExtruder()
    {
        return extruder_plans.back().extruder;
    }

    void setExtrudeSpeedFactor(double speedFactor)
    {
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
    
    void setFanSpeed(double _fan_speed)
    {
        fan_speed = _fan_speed;
    }

    template<typename... Args>
    GCodePathConfig& getRaftConfig(Args&&... args)
    {
        if (!raft_config)
        {
            raft_config = new GCodePathConfig(args...);
        }
        return *raft_config;
    }
    
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

    /*!
     * Compute naive time estimates (without accountign for slow down at corners etc.) and naive material estimates (without accounting for MergeInfillLines)
     * and store them in each ExtruderPlan and each GCodePath.
     * 
     * \return the total estimates of this layer
     */
    TimeMaterialEstimates computeNaiveTimeEstimates();
    
    void forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrusionTime);
    
    /*!
     * Write the planned paths to gcode
     * 
     * \param gcode The gcode to write the planned paths to
     */
    void writeGCode(GCodeExport& gcode, bool liftHeadIfNeeded, int layerThickness);
    
    /*!
     * Whether the current retracted path is to be an extruder switch retraction.
     * This function is used to avoid a G10 S1 after a G10.
     * 
     * \param gcode The gcode to write the planned paths to
     * \param extruder_plan_idx The index of the current extruder plan
     * \param path_idx The index of the current retracted path 
     * \return Whether the path should be an extgruder switch retracted path
     */
    bool makeRetractSwitchRetract(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx);
    
    /*!
     * Writes a path to GCode and performs coasting, or returns false if it did nothing.
     * 
     * Coasting replaces the last piece of an extruded path by move commands and uses the oozed material to lay down lines.
     * 
     * \param gcode The gcode to write the planned paths to
     * \param extruder_plan_idx The index of the current extruder plan
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
    bool writePathWithCoasting(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx, int64_t layerThickness, double coasting_volume_move, double coasting_speed_move, double coasting_min_volume_move, double coasting_volume_retract, double coasting_speed_retract, double coasting_min_volume_retract);

    /*!
     * Writes a path to GCode and performs coasting, or returns false if it did nothing.
     * 
     * Coasting replaces the last piece of an extruded path by move commands and uses the oozed material to lay down lines.
     * 
     * Paths shorter than \p coasting_min_volume will use less \p coasting_volume linearly.
     * 
     * \param gcode The gcode to write the planned paths to
     * \param path The extrusion path to be written to GCode.
     * \param path_next The next travel path to be written to GCode.
     * \param layerThickness The height of the current layer.
     * \param coasting_volume The volume otherwise leaked.
     * \param coasting_speed The speed at which to move during coasting.
     * \param coasting_min_volume The minimal volume a path should have which builds up enough pressure to ooze as much as \p coasting_volume.
     * \param extruder_switch_retract (optional) For a coasted path followed by a retraction: whether to retract normally, or do an extruder switch retraction.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(GCodeExport& gcode, GCodePath& path, GCodePath& path_next, int64_t layerThickness, double coasting_volume, double coasting_speed, double coasting_min_volume, bool extruder_switch_retract = false);
    
    /*!
     * Write a retraction: either an extruder switch retraction or a normal retraction based on the last extrusion paths retraction config.
     * \param gcode The gcode to write the planned paths to
     * \param extruder_plan_idx The index of the current extruder plan
     * \param path_idx_travel_after Index in GCodePlanner::paths to the travel move before which to do the retraction
     */
    void writeRetraction(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx_travel_after);
    
    /*!
     * Write a retraction: either an extruder switch retraction or a normal retraction based on the given retraction config.
     * \param gcode The gcode to write the planned paths to
     * \param extruder_switch_retract Whether to write an extruder switch retract
     * \param retraction_config The config used.
     */
    void writeRetraction(GCodeExport& gcode, bool extruder_switch_retract, RetractionConfig* retraction_config);
    
    /*!
     * Applying speed corrections for minimal layer times and determine the fanSpeed. 
     */
    void processFanSpeedAndMinimalLayerTime();
    
    void moveInsideCombBoundary(int arg1);
};

}//namespace cura

#endif//GCODE_PLANNER_H
