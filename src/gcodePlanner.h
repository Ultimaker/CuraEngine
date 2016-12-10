/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <vector>

#include "gcodeExport.h"
#include "pathPlanning/Comb.h"
#include "pathPlanning/GCodePath.h"
#include "pathPlanning/NozzleTempInsert.h"
#include "pathPlanning/TimeMaterialEstimates.h"
#include "utils/polygon.h"
#include "utils/logoutput.h"
#include "wallOverlap.h"
#include "commandSocket.h"
#include "FanSpeedLayerTime.h"
#include "SpaceFillType.h"
#include "GCodePathConfig.h"

#include "utils/optional.h"

namespace cura 
{

class SliceDataStorage;

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
    double heated_pre_travel_time; //!< The time at the start of this ExtruderPlan during which the head travels and has a temperature of initial_print_temperature
    double initial_printing_temperature; //!< The required temperature at the start of this extruder plan.
    double printing_temperature; //!< The normal temperature for printing this extruder plan. That start and end of this extruder plan may deviate because of the initial and final print temp
    std::optional<std::list<NozzleTempInsert>::iterator> printing_temperature_command; //!< The command to heat from the printing temperature of this extruder plan to the printing temperature of the next extruder plan (if it has the same extruder).
    std::optional<double> prev_extruder_standby_temp; //!< The temperature to which to set the previous extruder. Not used if the previous extruder plan was the same extruder.

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
    ExtruderPlan(int extruder, Point start_position, int layer_nr, bool is_initial_layer, int layer_thickness, FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config);

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
     * 
     * \param force_minimal_layer_time Whether we should apply speed changes and perhaps a head lift in order to meet the minimal layer time
     */
    void processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time);

    /*!
     * Set the extrude speed factor. This is used for printing slower than normal.
     * 
     * Leaves the extrusion speed as is for values of 1.0
     * 
     * \param speedFactor The factor by which to alter the extrusion move speed
     */
    void setExtrudeSpeedFactor(double speedFactor);

    /*!
     * Get the extrude speed factor. This is used for printing slower than normal.
     * 
     * \return The factor by which to alter the extrusion move speed
     */
    double getExtrudeSpeedFactor();

    /*!
     * Set the travel speed factor. This is used for performing non-extrusion travel moves slower than normal.
     * 
     * Leaves the extrusion speed as is for values of 1.0
     * 
     * \param speedFactor The factor by which to alter the non-extrusion move speed
     */
    void setTravelSpeedFactor(double speedFactor);

    /*!
     * Get the travel speed factor. This is used for travelling slower than normal.
     * 
     * Limited to at most 1.0
     * 
     * \return The factor by which to alter the non-extrusion move speed
     */
    double getTravelSpeedFactor();

    /*!
     * Get the fan speed computed for this extruder plan
     * 
     * \warning assumes ExtruderPlan::processFanSpeedAndMinimalLayerTime has already been called
     * 
     * \return The fan speed computed in processFanSpeedAndMinimalLayerTime
     */
    double getFanSpeed();
protected:

    Point start_position; //!< The position the print head was at at the start of this extruder plan

    int layer_nr; //!< The layer number at which we are currently printing.
    bool is_initial_layer; //!< Whether this extruder plan is printed on the very first layer (which might be raft)

    int layer_thickness; //!< The thickness of this layer in Z-direction

    FanSpeedLayerTimeSettings& fan_speed_layer_time_settings; //!< The fan speed and layer time settings used to limit this extruder plan

    const RetractionConfig& retraction_config; //!< The retraction settings for the extruder of this plan

    double extrudeSpeedFactor; //!< The factor by which to alter the extrusion move speed
    double travelSpeedFactor; //!< The factor by which to alter the non-extrusion move speed

    double extraTime; //!< Extra waiting time at the and of this extruder plan, so that the filament can cool
    double totalPrintTime; //!< The total naive time estimate for this extruder plan

    double fan_speed; //!< The fan speed to be used during this extruder plan

    /*!
     * Set the fan speed to be used while printing this extruder plan
     * 
     * \param fan_speed The speed for the fan
     */
    void setFanSpeed(double fan_speed);

    /*!
     * Force the minimal layer time to hold by slowing down and lifting the head if required.
     * 
     */
    void forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrusionTime);

    /*!
     * Compute naive time estimates (without accounting for slow down at corners etc.) and naive material estimates (without accounting for MergeInfillLines)
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
    friend class GCodePlannerTest;
private:
    SliceDataStorage& storage; //!< The polygon data obtained from FffPolygonProcessor

    int layer_nr; //!< The layer number of this layer plan
    int is_initial_layer; //!< Whether this is the first layer (which might be raft)
    
    int z; 
    
    int layer_thickness;
    
    Point start_position;
    Point lastPosition;
    
    std::vector<ExtruderPlan> extruder_plans; //!< should always contain at least one ExtruderPlan

    int last_extruder_previous_layer; //!< The last id of the extruder with which was printed in the previous layer
    SettingsBaseVirtual* last_planned_extruder_setting_base; //!< The setting base of the last planned extruder.
    bool was_inside; //!< Whether the last planned (extrusion) move was inside a layer part
    bool is_inside; //!< Whether the destination of the next planned travel move is inside a layer part
    Polygons comb_boundary_inside; //!< The boundary within which to comb, or to move into when performing a retraction.
    Comb* comb;


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

public:
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

    /*!
     * 
     * \param fan_speed_layer_time_settings_per_extruder The fan speed and layer time settings for each extruder.
     * \param travel_avoid_other_parts Whether to avoid other layer parts when travaeling through air.
     * \param travel_avoid_distance The distance by which to avoid other layer parts when traveling through air.
     * \param last_position The position of the head at the start of this gcode layer
     * \param combing_mode Whether combing is enabled and full or within infill only.
     */
    GCodePlanner(SliceDataStorage& storage, int layer_nr, int z, int layer_height, Point last_position, int current_extruder, bool is_inside_mesh, std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, CombingMode combing_mode, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance);
    ~GCodePlanner();

    void overrideFanSpeeds(double speed);
    /*!
     * Get the settings base of the last extruder planned.
     * \return the settings base of the last extruder planned.
     */
    SettingsBaseVirtual* getLastPlannedExtruderTrainSettings();
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
     * send a line segment through the command socket from the previous point to the given point \p to
     */
    void sendLineTo(PrintFeatureType print_feature_type, Point to, int line_width)
    {
        CommandSocket::sendLineTo(print_feature_type, to, line_width);
    }

    /*!
    * Set whether the next destination is inside a layer part or not.
    * 
    * Features like infill, walls, skin etc. are considered inside.
    * Features like prime tower and support are considered outside.
    */
    void setIsInside(bool going_to_comb);

    /*!
     * Plan a switch to a new extruder
     * 
     * \param extruder The extruder number to which to switch
     * \return whether the extruder has changed
     */
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
    GCodePath& addTravel(Point p);
    
    /*!
     * Add a travel path to a certain point and retract if needed.
     * 
     * No combing is performed.
     * 
     * \param p The point to travel to
     * \param path (optional) The travel path to which to add the point \p p
     */
    GCodePath& addTravel_simple(Point p, GCodePath* path = nullptr);

    /*!
     * Plan a prime poop at the current location.
     * 
     * \warning A nonretracted move is introduced so that the LayerPlanBuffer classifies this move as an extrusion move.
     */
    void planPrime();

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
     * \param wall_0_wipe_dist The distance to travel along the polygon after it has been laid down, in order to wipe the start and end of the wall together
     * \param spiralize Whether to gradually increase the z height from the normal layer height to the height of the next layer over this polygon
     */
    void addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr, coord_t wall_0_wipe_dist = 0, bool spiralize = false);

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
     * \param z_seam_pos The location near where to start each part in case \p z_seam_type is 'back'
     * \param wall_0_wipe_dist The distance to travel along each polygon after it has been laid down, in order to wipe the start and end of the wall together
     * \param spiralize Whether to gradually increase the z height from the normal layer height to the height of the next layer over each polygon printed
     */
    void addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation = nullptr, EZSeamType z_seam_type = EZSeamType::SHORTEST, Point z_seam_pos = Point(0, 0), coord_t wall_0_wipe_dist = 0, bool spiralize = false);

    /*!
     * Add lines to the gcode with optimized order.
     * \param polygons The lines
     * \param config The config of the lines
     * \param space_fill_type The type of space filling used to generate the line segments (should be either Lines or PolyLines!)
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     */
    void addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config, SpaceFillType space_fill_type, int wipe_dist = 0);

    /*!
     * Compute naive time estimates (without accounting for slow down at corners etc.) and naive material estimates (without accounting for MergeInfillLines)
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
     * Complete all GcodePathConfigs by
     * - altering speeds to conform to speed_print_layer_0 and
     *   speed_travel_layer_0
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
     * \param extruder_plan_idx The index of the current extruder plan
     * \param path_idx The index of the current retracted path 
     * \return Whether the path should be an extgruder switch retracted path
     */
    bool makeRetractSwitchRetract(unsigned int extruder_plan_idx, unsigned int path_idx);
    
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
