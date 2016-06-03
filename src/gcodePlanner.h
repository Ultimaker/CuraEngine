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
#include "SpaceFillType.h"


namespace cura 
{

class SliceDataStorage;

/*!
 * A gcode command to insert before a specific path.
 * 
 * Currently only used for preheat commands
 */
struct NozzleTempInsert
{
    const unsigned int path_idx; //!< The path before which to insert this command
    double time_after_path_start; //!< The time after the start of the path, before which to insert the command // TODO: use this to insert command in between moves in a path!
    int extruder; //!< The extruder for which to set the temp
    double temperature; //!< The temperature of the temperature command to insert
    bool wait; //!< Whether to wait for the temperature to be reached
    NozzleTempInsert(unsigned int path_idx, int extruder, double temperature, bool wait, double time_after_path_start = 0.0)
    : path_idx(path_idx)
    , time_after_path_start(time_after_path_start)
    , extruder(extruder)
    , temperature(temperature)
    , wait(wait)
    {}
    
    /*!
     * Write the temperature command at the current position in the gcode.
     * \param gcode The actual gcode writer
     */
    void write(GCodeExport& gcode)
    {
        gcode.writeTemperatureCommand(extruder, temperature, wait);
    }
};

class ExtruderPlan; // forward declaration so that TimeMaterialEstimates can be a friend


/*!
 * Time and material estimates for a portion of paths, e.g. layer, extruder plan, path.
 */
class TimeMaterialEstimates
{
    friend class ExtruderPlan; // cause there the naive estimates are calculated
private:
    double extrude_time; //!< Time in seconds occupied by extrusion
    double unretracted_travel_time; //!< Time in seconds occupied by non-retracted travel (non-extrusion)
    double retracted_travel_time; //!< Time in seconds occupied by retracted travel (non-extrusion)
    double material; //!< Material used (in mm^3)
public:
    /*!
     * Basic contructor
     * 
     * \param extrude_time Time in seconds occupied by extrusion
     * \param unretracted_travel_time Time in seconds occupied by non-retracted travel (non-extrusion)
     * \param retracted_travel_time Time in seconds occupied by retracted travel (non-extrusion)
     * \param material Material used (in mm^3)
     */
    TimeMaterialEstimates(double extrude_time, double unretracted_travel_time, double retracted_travel_time, double material)
    : extrude_time(extrude_time)
    , unretracted_travel_time(unretracted_travel_time)
    , retracted_travel_time(retracted_travel_time)
    , material(material)
    {
    }

    /*!
     * Basic constructor initializing all estimates to zero.
     */
    TimeMaterialEstimates()
    : extrude_time(0.0)
    , unretracted_travel_time(0.0)
    , retracted_travel_time(0.0)
    , material(0.0)
    {
    }

    /*!
     * Set all estimates to zero.
     */
    void reset() 
    {
        extrude_time = 0.0;
        unretracted_travel_time = 0.0;
        retracted_travel_time = 0.0;
        material = 0.0;
    }

    /*!
     * Pointwise addition of estimate stats
     * 
     * \param other The estimates to add to these estimates.
     * \return The resulting estimates
     */
    TimeMaterialEstimates operator+(const TimeMaterialEstimates& other)
    {
        return TimeMaterialEstimates(extrude_time+other.extrude_time, unretracted_travel_time+other.unretracted_travel_time, retracted_travel_time+other.retracted_travel_time, material+other.material);
    }

    /*!
     * In place pointwise addition of estimate stats
     * 
     * \param other The estimates to add to these estimates.
     * \return These estimates
     */
    TimeMaterialEstimates& operator+=(const TimeMaterialEstimates& other)
    {
        extrude_time += other.extrude_time;
        unretracted_travel_time += other.unretracted_travel_time;
        retracted_travel_time += other.retracted_travel_time;
        material += other.material;
        return *this;
    }

    /*!
     * \brief Subtracts the specified estimates from these estimates and returns
     * the result.
     * 
     * Each of the estimates in this class are individually subtracted.
     * 
     * \param other The estimates to subtract from these estimates.
     * \return These estimates with the specified estimates subtracted.
     */
    TimeMaterialEstimates operator-(const TimeMaterialEstimates& other);

    /*!
     * \brief Subtracts the specified elements from these estimates.
     * 
     * This causes the estimates in this instance to change. Each of the
     * estimates in this class are individually subtracted.
     * 
     * \param other The estimates to subtract from these estimates.
     * \return A reference to this instance.
     */
    TimeMaterialEstimates& operator-=(const TimeMaterialEstimates& other);

    /*!
     * Get total time estimate. The different time estimate member values added together.
     * 
     * \return the total of all different time estimate values 
     */
    double getTotalTime() const
    {
        return extrude_time + unretracted_travel_time + retracted_travel_time;
    }

    /*!
     * Get the total time during which the head is not retracted.
     * 
     * This includes extrusion time and non-retracted travel time
     * 
     * \return the total time during which the head is not retracted.
     */
    double getTotalUnretractedTime() const
    {
        return extrude_time + unretracted_travel_time;
    }

    /*!
     * Get the total travel time.
     * 
     * This includes the retracted travel time as well as the unretracted travel time.
     * 
     * \return the total travel time.
     */
    double getTravelTime() const
    {
        return retracted_travel_time + unretracted_travel_time;
    }

    /*!
     * Get the extrusion time.
     * 
     * \return extrusion time.
     */
    double getExtrudeTime() const
    {
        return extrude_time;
    }

    /*!
     * Get the amount of material used in mm^3.
     * 
     * \return amount of material
     */
    double getMaterial() const
    {
        return material;
    }
};

/*!
 * A class for representing a planned path.
 * 
 * A path consists of several segments of the same type of movement: retracted travel, infill extrusion, etc.
 * 
 * This is a compact premature representation in which are line segments have the same config, i.e. the config of this path.
 * 
 * In the final representation (gcode) each line segment may have different properties, 
 * which are added when the generated GCodePaths are processed.
 */
class GCodePath
{
public:
    GCodePathConfig* config; //!< The configuration settings of the path.
    SpaceFillType space_fill_type; //!< The type of space filling of which this path is a part
    float flow; //!< A type-independent flow configuration (used for wall overlap compensation)
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path. 
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.

    bool spiralize; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and end in one layer higher.

    TimeMaterialEstimates estimates; //!< Naive time and material estimates

    /*!
     * Whether this config is the config of a travel path.
     * 
     * \return Whether this config is the config of a travel path.
     */
    bool isTravelPath()
    {
        return config->isTravelPath();
    }

    /*!
     * Get the material flow in mm^3 per mm traversed.
     * 
     * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
     * 
     * \return The flow
     */
    double getExtrusionMM3perMM()
    {
        return flow * config->getExtrusionMM3perMM();
    }
    
    /*!
     * Get the actual line width (modulated by the flow)
     * \return the actual line width as shown in layer view
     */
    int getLineWidth()
    {
        return flow * config->getLineWidth() * config->getFlowPercentage() / 100.0;
    }
};

class GCodePlanner; // forward declaration so that ExtruderPlan can be a friend
class LayerPlanBuffer; // forward declaration so that ExtruderPlan can be a friend

/*!
 * An extruder plan contains all planned paths (GCodePath) pertaining to a single extruder train.
 * 
 * It allows for temperature command inserts which can be inserted in between paths.
 */
class ExtruderPlan
{
    friend class GCodePlanner; // TODO: GCodePlanner still does a lot which should actually be handled in this class.
    friend class LayerPlanBuffer; // TODO: LayerPlanBuffer handles paths directly
protected:
    std::vector<GCodePath> paths; //!< The paths planned for this extruder
    std::list<NozzleTempInsert> inserts; //!< The nozzle temperature command inserts, to be inserted in between paths

    int extruder; //!< The extruder used for this paths in the current plan.
    double required_temp; //!< The required temperature at the start of this extruder plan.

    TimeMaterialEstimates estimates; //!< Accumulated time and material estimates for all planned paths within this extruder plan.

public:
    /*!
     * Simple contructor.
     * 
     * \warning Doesn't set the required temperature yet.
     * 
     * \param extruder The extruder number for which this object is a plan.
     * \param start_position The position the head is when this extruder plan starts
     */
    ExtruderPlan(int extruder, Point start_position, int layer_nr, int layer_thickness, FanSpeedLayerTimeSettings& fan_speed_layer_time_settings);

    /*!
     * Add a new Insert, constructed with the given arguments
     * 
     * \see NozzleTempInsert
     * 
     * \param contructor_args The arguments for the constructor of an insert 
     */
    template<typename... Args>
    void insertCommand(Args&&... contructor_args)
    {
        inserts.emplace_back(contructor_args...);
    }

    /*!
     * Insert the inserts into gcode which should be inserted before \p path_idx
     * 
     * \param path_idx The index into ExtruderPlan::paths which is currently being consider for temperature command insertion
     * \param gcode The gcode exporter to which to write the temperature command.
     */
    void handleInserts(unsigned int& path_idx, GCodeExport& gcode)
    {
        while ( ! inserts.empty() && path_idx >= inserts.front().path_idx)
        { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
            inserts.front().write(gcode);
            inserts.pop_front();
        }
    }

    /*!
     * Insert all remaining temp inserts into gcode, to be called at the end of an extruder plan
     * 
     * Inserts temperature commands which should be inserted _after_ the last path.
     * Also inserts all temperatures which should have been inserted earlier,
     * but for which ExtruderPlan::handleInserts hasn't been called correctly.
     * 
     * \param gcode The gcode exporter to which to write the temperature command.
     */
    void handleAllRemainingInserts(GCodeExport& gcode)
    { 
        while ( ! inserts.empty() )
        { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
            NozzleTempInsert& insert = inserts.front();
            assert(insert.path_idx == paths.size());
            insert.write(gcode);
            inserts.pop_front();
        }
    }

    /*!
     * Applying speed corrections for minimal layer times and determine the fanSpeed. 
     */
    void processFanSpeedAndMinimalLayerTime();

    void setExtrudeSpeedFactor(double speedFactor);
    double getExtrudeSpeedFactor();
    void setTravelSpeedFactor(double speedFactor);
    double getTravelSpeedFactor(); //!< Get the travel speed factor limited to at most 1.0

    double getFanSpeed();
protected:

    Point start_position;

    int layer_nr;
    int layer_thickness;

    FanSpeedLayerTimeSettings& fan_speed_layer_time_settings; //!< The fan speed and layer time settings used to limit this extruder plan

    double extrudeSpeedFactor;
    double travelSpeedFactor;

    double extraTime; //!< Extra waiting time at the and of this extruder plan, so that the filament can cool
    double totalPrintTime; //!< The total naive time estimate for this extruder plan

    double fan_speed;

    void setFanSpeed(double _fan_speed);

    /*!
     * Force the minimal layer time to hold by slowing down and lifting the head if required.
     * 
     */
    void forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrusionTime);

    /*!
     * Compute naive time estimates (without accountign for slow down at corners etc.) and naive material estimates (without accounting for MergeInfillLines)
     * and store them in each ExtruderPlan and each GCodePath.
     * 
     * \return the total estimates of this layer
     */
    TimeMaterialEstimates computeNaiveTimeEstimates();
};

class LayerPlanBuffer; // forward declaration to prevent circular dependency

/*! 
 * The GCodePlanner class stores multiple moves that are planned.
 * 
 * 
 * It facilitates the combing to keep the head inside the print.
 * It also keeps track of the print time estimate for this planning so speed adjustments can be made for the minimal-layer-time.
 * 
 * A GCodePlanner is also knows as a 'layer plan'.
 * 
 */
class GCodePlanner : public NoCopy
{
    friend class LayerPlanBuffer;
private:
    SliceDataStorage& storage; //!< The polygon data obtained from FffPolygonProcessor

    int layer_nr; //!< The layer number of this layer plan
    
    int z; 
    
    int layer_thickness;
    
    Point start_position;
    Point lastPosition;
    
    std::vector<ExtruderPlan> extruder_plans; //!< should always contain at least one ExtruderPlan
    
    bool was_inside; //!< Whether the last planned (extrusion) move was inside a layer part
    bool is_inside; //!< Whether the destination of the next planned travel move is inside a layer part
    Polygons comb_boundary_inside; //!< The boundary within which to comb, or to move into when performing a retraction.
    Comb* comb;

    RetractionConfig* last_retraction_config;

    std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder;
    
private:
    /*!
     * Either create a new path with the given config or return the last path if it already had that config.
     * If GCodePlanner::forceNewPathStart has been called a new path will always be returned.
     * 
     * \param config The config used for the path returned
     * \param space_fill_type The type of space filling which this path employs
     * \param flow (optional) A ratio for the extrusion speed
     * \param spiralize Whether to gradually increase the z while printing. (Note that this path may be part of a sequence of spiralized paths, forming one polygon)
     * \return A path with the given config which is now the last path in GCodePlanner::paths
     */
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config, SpaceFillType space_fill_type, float flow = 1.0, bool spiralize = false);
    
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
     * \param fan_speed_layer_time_settings_per_extruder The fan speed and layer time settings for each extruder.
     * \param travel_avoid_other_parts Whether to avoid other layer parts when travaeling through air.
     * \param travel_avoid_distance The distance by which to avoid other layer parts when traveling through air.
     * \param last_position The position of the head at the start of this gcode layer
     * \param combing_mode Whether combing is enabled and full or within infill only.
     */
    GCodePlanner(SliceDataStorage& storage, unsigned int layer_nr, int z, int layer_height, Point last_position, int current_extruder, bool is_inside_mesh, std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, CombingMode combing_mode, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance);
    ~GCodePlanner();

    void overrideFanSpeeds(double speed);
private:
    /*!
     * Compute the boundary within which to comb, or to move into when performing a retraction.
     * \param combing_mode Whether combing is enabled and full or within infill only.
     * \return the comb_boundary_inside
     */
    Polygons computeCombBoundaryInside(CombingMode combing_mode);

public:
    int getLayerNr()
    {
        return layer_nr;
    }
    
    Point getLastPosition()
    {
        return lastPosition;
    }

    /*!
     * return whether the last position planned was inside the mesh (used in combing)
     */
    bool getIsInsideMesh()
    {
        return was_inside;
    }
    /*!
     * send a polygon through the command socket from the previous point to the given point
     */
    void sendPolygon(PrintFeatureType print_feature_type, Point from, Point to, int line_width)
    {
        if (CommandSocket::isInstantiated()) 
        {
            // we should send this travel as a non-retraction move
            cura::Polygons pathPoly;
            PolygonRef path = pathPoly.newPoly();
            path.add(from);
            path.add(to);
            CommandSocket::getInstance()->sendPolygons(print_feature_type, layer_nr, pathPoly, line_width);
        }
    }

    /*!
    * Set whether the next destination is inside a layer part or not.
    * 
    * Features like infill, walls, skin etc. are considered inside.
    * Features like prime tower and support are considered outside.
    */
    void setIsInside(bool going_to_comb);
    
    bool setExtruder(int extruder);

    /*!
     * Get the last planned extruder.
     */
    int getExtruder()
    {
        return extruder_plans.back().extruder;
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

    /*!
     * Add an extrusion move to a certain point, optionally with a different flow than the one in the \p config.
     * 
     * \param p The point to extrude to
     * \param config The config with which to extrude
     * \param space_fill_type Of what space filling type this extrusion move is a part
     * \param flow A modifier of the extrusion width which would follow from the \p config
     * \param spiralize Whether to gradually increase the z while printing. (Note that this path may be part of a sequence of spiralized paths, forming one polygon)
     */
    void addExtrusionMove(Point p, GCodePathConfig* config, SpaceFillType space_fill_type, float flow = 1.0, bool spiralize = false);

    /*!
     * Add polygon to the gcode starting at vertex \p startIdx
     * \param polygon The polygon
     * \param startIdx The index of the starting vertex of the \p polygon
     * \param config The config with which to print the polygon lines
     * \param wall_overlap_computation The wall overlap compensation calculator for each given segment (optionally nullptr)
     * \param spiralize Whether to gradually increase the z height from the normal layer height to the height of the next layer over this polygon
     */
    void addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr, bool spiralize = false);

    /*!
     * Add polygons to the gcode with optimized order.
     * 
     * When \p spiralize is true, each polygon will gradually increase from a z corresponding to this layer to the z corresponding to the next layer.
     * Doing this for each polygon means there is a chance for the print head to crash into already printed parts,
     * but doing it for the last polygon only would mean you are printing half of the layer in non-spiralize mode,
     * while each layer starts with a different part.
     * Two towers would result in alternating spiralize and non-spiralize layers.
     * 
     * \param polygons The polygons
     * \param config The config with which to print the polygon lines
     * \param wall_overlap_computation The wall overlap compensation calculator for each given segment (optionally nullptr)
     * \param z_seam_type The seam type / poly start optimizer
     * \param spiralize Whether to gradually increase the z height from the normal layer height to the height of the next layer over each polygon printed
     */
    void addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr, EZSeamType z_seam_type = EZSeamType::SHORTEST, bool spiralize = false);

    /*!
     * Add lines to the gcode with optimized order.
     * \param polygons The lines
     * \param config The config of the lines
     * \param space_fill_type The type of space filling used to generate the line segments (should be either Lines or PolyLines!)
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     */
    void addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config, SpaceFillType space_fill_type, int wipe_dist = 0);

    /*!
     * Compute naive time estimates (without accountign for slow down at corners etc.) and naive material estimates (without accounting for MergeInfillLines)
     * and store them in each ExtruderPlan and each GCodePath.
     * 
     * \warning This function recomputes values which are already computed if you've called processFanSpeedAndMinimalLayerTime
     * 
     * \return the total estimates of this layer
     */
    TimeMaterialEstimates computeNaiveTimeEstimates();
    
    /*!
     * Write the planned paths to gcode
     * 
     * \param gcode The gcode to write the planned paths to
     */
    void writeGCode(GCodeExport& gcode);
    
    /*!
     * Complete all GcodePathConfig s by 
     * - altering speed to conform to speed_layer_0
     * - setting the layer_height (and thereby computing the extrusionMM3perMM)
     */
    void completeConfigs();
    
    /*!
     * Interpolate between the initial layer speeds and the eventual speeds.
     */
    void processInitialLayersSpeedup();
    
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
     * \param coasting_volume The volume otherwise leaked during a normal move.
     * \param coasting_speed The speed at which to move during move-coasting.
     * \param coasting_min_volume The minimal volume a path should have (before starting to coast) which builds up enough pressure to ooze as much as \p coasting_volume.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx, int64_t layerThickness, double coasting_volume, double coasting_speed, double coasting_min_volume);

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
    
    /*!
     * Add a travel move to the layer plan to move inside the current layer part by a given distance away from the outline.
     * This is supposed to be called when the nozzle is around the boundary of a layer part, not when the nozzle is in the middle of support, or in the middle of the air.
     * 
     * \param distance The distance to the comb boundary after we moved inside it.
     */
    void moveInsideCombBoundary(int distance);
};

}//namespace cura

#endif//GCODE_PLANNER_H
