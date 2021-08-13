//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LAYER_PLAN_H
#define LAYER_PLAN_H

#include <optional>
#include <vector>
#ifdef BUILD_TESTS
    #include <gtest/gtest_prod.h> //Friend tests, so that they can inspect the privates.
#endif

#include "FanSpeedLayerTime.h"
#include "gcodeExport.h"
#include "PathOrderOptimizer.h"
#include "SpaceFillType.h"
#include "pathPlanning/GCodePath.h"
#include "pathPlanning/NozzleTempInsert.h"
#include "pathPlanning/TimeMaterialEstimates.h"
#include "settings/PathConfigStorage.h"
#include "settings/types/LayerIndex.h"
#include "utils/polygon.h"

#include "InsetOrderOptimizer.h"
#include "utils/ExtrusionJunction.h"

namespace cura 
{

class Comb;
class LayerPlan; // forward declaration so that ExtruderPlan can be a friend
class LayerPlanBuffer; // forward declaration so that ExtruderPlan can be a friend
class SliceDataStorage;

/*!
 * An extruder plan contains all planned paths (GCodePath) pertaining to a single extruder train.
 * 
 * It allows for temperature command inserts which can be inserted in between paths.
 */
class ExtruderPlan
{
    friend class LayerPlan; // TODO: LayerPlan still does a lot which should actually be handled in this class.
    friend class LayerPlanBuffer; // TODO: LayerPlanBuffer handles paths directly
#ifdef BUILD_TESTS
    friend class ExtruderPlanPathsParameterizedTest;
    FRIEND_TEST(ExtruderPlanPathsParameterizedTest, BackPressureCompensationZeroIsUncompensated);
    FRIEND_TEST(ExtruderPlanPathsParameterizedTest, BackPressureCompensationFull);
    FRIEND_TEST(ExtruderPlanPathsParameterizedTest, BackPressureCompensationHalf);
    FRIEND_TEST(ExtruderPlanTest, BackPressureCompensationEmptyPlan);
#endif
protected:
    std::vector<GCodePath> paths; //!< The paths planned for this extruder
    std::list<NozzleTempInsert> inserts; //!< The nozzle temperature command inserts, to be inserted in between paths

    double heated_pre_travel_time; //!< The time at the start of this ExtruderPlan during which the head travels and has a temperature of initial_print_temperature

    /*!
     * The required temperature at the start of this extruder plan
     * or the temp to which to heat gradually over the layer change between this plan and the previous with the same extruder.
     * 
     * In case this extruder plan uses a different extruder than the last extruder plan:
     * this is the temperature to which to heat and wait before starting this extruder.
     * 
     * In case this extruder plan uses the same extruder as the previous extruder plan (previous layer):
     * this is the temperature used to heat to gradually when moving from the previous extruder layer to the next.
     * In that case no temperature (and wait) command will be inserted from this value, but a NozzleTempInsert is used instead.
     * In this case this member is only used as a way to convey information between different calls of \ref LayerPlanBuffer::processBuffer
     */
    double required_start_temperature;
    std::optional<double> extrusion_temperature; //!< The normal temperature for printing this extruder plan. That start and end of this extruder plan may deviate because of the initial and final print temp (none if extruder plan has no extrusion moves)
    std::optional<std::list<NozzleTempInsert>::iterator> extrusion_temperature_command; //!< The command to heat from the printing temperature of this extruder plan to the printing temperature of the next extruder plan (if it has the same extruder).
    std::optional<double> prev_extruder_standby_temp; //!< The temperature to which to set the previous extruder. Not used if the previous extruder plan was the same extruder.

    TimeMaterialEstimates estimates; //!< Accumulated time and material estimates for all planned paths within this extruder plan.

public:
    size_t extruder_nr; //!< The extruder used for this paths in the current plan.

    /*!
     * Simple contructor.
     * 
     * \warning Doesn't set the required temperature yet.
     * 
     * \param extruder The extruder number for which this object is a plan.
     * \param layer_nr The layer index of the layer that this extruder plan is
     * part of.
     * \param is_raft_layer Whether this extruder plan is part of a raft layer.
     */
    ExtruderPlan(const size_t extruder, const LayerIndex layer_nr, const bool is_initial_layer, const bool is_raft_layer, const coord_t layer_thickness, const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config);

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
    void handleInserts(unsigned int& path_idx, GCodeExport& gcode);

    /*!
     * Insert all remaining temp inserts into gcode, to be called at the end of an extruder plan
     * 
     * Inserts temperature commands which should be inserted _after_ the last path.
     * Also inserts all temperatures which should have been inserted earlier,
     * but for which ExtruderPlan::handleInserts hasn't been called correctly.
     * 
     * \param gcode The gcode exporter to which to write the temperature command.
     */
    void handleAllRemainingInserts(GCodeExport& gcode);

    /*!
     * Applying speed corrections for minimal layer times and determine the fanSpeed. 
     * 
     * \param force_minimal_layer_time Whether we should apply speed changes and perhaps a head lift in order to meet the minimal layer time
     * \param starting_position The position the head was before starting this extruder plan
     */
    void processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time, Point starting_position);

    /*!
     * Set the extrude speed factor. This is used for printing slower than normal.
     * 
     * Leaves the extrusion speed as is for values of 1.0
     * 
     * \param speedFactor The factor by which to alter the extrusion move speed
     */
    void setExtrudeSpeedFactor(const Ratio speed_factor);

    /*!
     * Get the extrude speed factor. This is used for printing slower than normal.
     * 
     * \return The factor by which to alter the extrusion move speed
     */
    double getExtrudeSpeedFactor();

    /*!
     * Get the fan speed computed for this extruder plan
     * 
     * \warning assumes ExtruderPlan::processFanSpeedAndMinimalLayerTime has already been called
     * 
     * \return The fan speed computed in processFanSpeedAndMinimalLayerTime
     */
    double getFanSpeed();

    /*!
     * Apply back-pressure compensation to this path.
     * Since the total (filament) pressure in a feeder-system is not only dependent on the pressure that exists between the nozzle and the
     * feed-mechanism (which should be near-constant on a bowden style setup), but _also_ between the nozzle and the last-printed layer.
     * This last type is called 'back-pressure'. In this function, properties of the path-outflow are adjusted so that the back-pressure is
     * compensated for. This is conjectured to be especially important if the printer has a Bowden-tube style setup.
     *
     * \param The amount of back-pressure compensation in mm^3/s. 'Applying' a value of 0 is a no-op.
     */
    void applyBackPressureCompensation(const double back_pressure_compensation);

protected:
    LayerIndex layer_nr; //!< The layer number at which we are currently printing.
    bool is_initial_layer; //!< Whether this extruder plan is printed on the very first layer (which might be raft)
    const bool is_raft_layer; //!< Whether this is a layer which is part of the raft

    coord_t layer_thickness; //!< The thickness of this layer in Z-direction

    const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings; //!< The fan speed and layer time settings used to limit this extruder plan

    const RetractionConfig& retraction_config; //!< The retraction settings for the extruder of this plan

    Ratio extrudeSpeedFactor; //!< The factor by which to alter the extrusion move speed

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
     * Compute naive time estimates (without accounting for slow down at corners etc.) and naive material estimates.
     * and store them in each ExtruderPlan and each GCodePath.
     * 
     * \param starting_position The position the head was in before starting this layer
     * \return the total estimates of this layer
     */
    TimeMaterialEstimates computeNaiveTimeEstimates(Point starting_position);
};

class LayerPlanBuffer; // forward declaration to prevent circular dependency

/*! 
 * The LayerPlan class stores multiple moves that are planned.
 * 
 * 
 * It facilitates the combing to keep the head inside the print.
 * It also keeps track of the print time estimate for this planning so speed adjustments can be made for the minimal-layer-time.
 * 
 * A LayerPlan is also knows as a 'layer plan'.
 * 
 */
class LayerPlan : public NoCopy
{
    friend class LayerPlanBuffer;
    friend class AddTravelTest;

public:
    const PathConfigStorage configs_storage; //!< The line configs for this layer for each feature type
    coord_t z;
    coord_t final_travel_z;
    bool mode_skip_agressive_merge; //!< Whether to give every new path the 'skip_agressive_merge_hint' property (see GCodePath); default is false.

private:

    const SliceDataStorage& storage; //!< The polygon data obtained from FffPolygonProcessor
    const LayerIndex layer_nr; //!< The layer number of this layer plan
    const bool is_initial_layer; //!< Whether this is the first layer (which might be raft)
    const bool is_raft_layer; //!< Whether this is a layer which is part of the raft
    coord_t layer_thickness;

    std::vector<Point> layer_start_pos_per_extruder; //!< The starting position of a layer for each extruder
    std::vector<bool> has_prime_tower_planned_per_extruder; //!< For each extruder, whether the prime tower is planned yet or not.
    std::optional<Point> last_planned_position; //!< The last planned XY position of the print head (if known)

    std::string current_mesh; //<! A unique ID for the mesh of the last planned move.

    /*!
     * Whether the skirt or brim polygons have been processed into planned paths
     * for each extruder train.
     */
    bool skirt_brim_is_processed[MAX_EXTRUDERS];

    std::vector<ExtruderPlan> extruder_plans; //!< should always contain at least one ExtruderPlan

    size_t last_extruder_previous_layer; //!< The last id of the extruder with which was printed in the previous layer
    ExtruderTrain* last_planned_extruder; //!< The extruder for which a move has most recently been planned.

    std::optional<Point> first_travel_destination; //!< The destination of the first (travel) move (if this layer is not empty)
    bool first_travel_destination_is_inside; //!< Whether the destination of the first planned travel move is inside a layer part
    bool was_inside; //!< Whether the last planned (extrusion) move was inside a layer part
    bool is_inside; //!< Whether the destination of the next planned travel move is inside a layer part
    Polygons comb_boundary_minimum; //!< The minimum boundary within which to comb, or to move into when performing a retraction.
    Polygons comb_boundary_preferred; //!< The boundary preferably within which to comb, or to move into when performing a retraction.
    Comb* comb;
    coord_t comb_move_inside_distance;  //!< Whenever using the minimum boundary for combing it tries to move the coordinates inside by this distance after calculating the combing.
    Polygons bridge_wall_mask; //!< The regions of a layer part that are not supported, used for bridging
    Polygons overhang_mask; //!< The regions of a layer part where the walls overhang

    const std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder;

    enum CombBoundary
    {
        MINIMUM,
        PREFERRED
    };

    /*!
     * Either create a new path with the given config or return the last path if it already had that config.
     * If LayerPlan::forceNewPathStart has been called a new path will always be returned.
     * 
     * \param config The config used for the path returned
     * \param space_fill_type The type of space filling which this path employs
     * \param flow (optional) A ratio for the extrusion speed
     * \param spiralize Whether to gradually increase the z while printing. (Note that this path may be part of a sequence of spiralized paths, forming one polygon)
     * \param speed_factor (optional) a factor which the speed will be multiplied by.
     * \return A path with the given config which is now the last path in LayerPlan::paths
     */
    GCodePath* getLatestPathWithConfig(const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio flow = 1.0_r, bool spiralize = false, const Ratio speed_factor = 1.0_r);

public:
    /*!
     * Force LayerPlan::getLatestPathWithConfig to return a new path.
     * 
     * This function is introduced because in some cases 
     * LayerPlan::getLatestPathWithConfig is called consecutively with the same config pointer, 
     * though the content of the config has changed.
     * 
     * Example cases: 
     * - when changing extruder, the same travel config is used, but its extruder field is changed.
     */
    void forceNewPathStart();

    /*!
     * \brief Creates a new plan for printing a layer.
     * \param storage The data storage this plan is stored in.
     * \param layer_nr The layer index of this plan.
     * \param z The height coordinate of this layer.
     * \param start_extruder The extruder with which this layer plan starts
     * \param fan_speed_layer_time_settings_per_extruder The fan speed and layer
     * time settings for each extruder.
     * \param comb_boundary_offset How far to avoid the walls on the outside
     * while combing.
     * \param comb_move_inside_distance How far to avoid the walls on the inside
     * while combing.
     * \param travel_avoid_distance The distance by which to avoid other layer
     * parts when travelling through air.
     */
    LayerPlan(const SliceDataStorage& storage, LayerIndex layer_nr, coord_t z, coord_t layer_height, size_t start_extruder, const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, coord_t comb_boundary_offset, coord_t comb_move_inside_distance, coord_t travel_avoid_distance);

    ~LayerPlan();

    void overrideFanSpeeds(double speed);

    /*!
     * \brief Get the last planned extruder train.
     * \return The last planned extruder.
     */
    ExtruderTrain* getLastPlannedExtruderTrain();

    const Polygons* getCombBoundaryInside() const;

    int getLayerNr() const;

    /*!
     * Get the last planned position, or if no position has been planned yet, the user specified layer start position.
     * 
     * \warning The layer start position might be outside of the build plate!
     */
    Point getLastPlannedPositionOrStartingPosition() const;

    /*!
     * return whether the last position planned was inside the mesh (used in combing)
     */
    bool getIsInsideMesh() const;

    /*!
     * Whether the prime tower is already planned for the specified extruder.
     * \param extruder_nr The extruder to check.
     */
    bool getPrimeTowerIsPlanned(unsigned int extruder_nr) const;

    /*!
     * Mark the prime tower as planned for the specified extruder.
     * \param extruder_nr The extruder to mark as having its prime tower
     * planned.
     */
    void setPrimeTowerIsPlanned(unsigned int extruder_nr);

    bool getSkirtBrimIsPlanned(unsigned int extruder_nr) const;

    void setSkirtBrimIsPlanned(unsigned int extruder_nr);

    /*!
     * Get the destination state of the first travel move.
     * This consists of the location and whether the destination was inside the model, or e.g. to support
     * 
     * Returns nothing if the layer is empty and no travel move was ever made.
     */
    std::optional<std::pair<Point, bool>> getFirstTravelDestinationState() const;

    /*!
    * Set whether the next destination is inside a layer part or not.
    * 
    * Features like infill, walls, skin etc. are considered inside.
    * Features like prime tower and support are considered outside.
    */
    void setIsInside(bool is_inside);

    /*!
     * \brief Plan a switch to a new extruder.
     * \param extruder_nr The extruder number to switch to.
     * \return Whether the extruder has changed. It won't have changed if we
     * were already on this extruder.
     */
    bool setExtruder(const size_t extruder_nr);

    /*!
     * Get the last planned extruder.
     */
    size_t getExtruder() const;

    /*!
     * Track the currently printing mesh.
     * \param mesh_id A unique ID indicating the current mesh.
     */
    void setMesh(const std::string mesh_id);

    /*!
     * Set bridge_wall_mask.
     *
     * \param polys The unsupported areas of the part currently being processed that will require bridges.
     */
    void setBridgeWallMask(const Polygons& polys);

    /*!
     * Set overhang_mask.
     *
     * \param polys The overhung areas of the part currently being processed that will require modified print settings
     */
    void setOverhangMask(const Polygons& polys);

    /*!
     * Travel to a certain point, with all of the procedures necessary to do so.
     *
     * Additional procedures here are:
     * - If retraction is forced, always retract.
     * - If the travel distance is shorter than the outer diameter of the nozzle
     *   - Travel directly without combing, retraction or Z hop.
     * - If combing is enabled, try a combing move.
     *   - If combing succeeds, i.e. there is a path to the destination
     *     - If the combed path is longer than retraction_combing_max_distance
     *       - Only retract (if enabled). Don't Z hop. Then follow coming path.
     *     - If the combed path is shorter
     *       - Travel the combing path without retraction.
     *   - If combing fails, i.e. the destination is in a different part
     *     - If Z hop is enabled
     *       - Retract (if enabled) and make a straight travel move.
     *     - If Z hop is disabled
     *       - Retract (if enabled) and make a multi-part travel move.
     * - If combing is disabled
     *   - Retract (if enabled) and Z hop (if enabled) and make straight travel.
     *
     * The first travel move in a layer will result in a bogus travel move with
     * no combing and no retraction. This travel move needs to be fixed
     * afterwards.
     * \param p The point to travel to.
     * \param force_comb_retract Whether to force a retraction to occur.
     */
    GCodePath& addTravel(const Point p, const bool force_retract = false);

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
     * Plan a prime blob at the current location.
     */
    void planPrime(const float& prime_blob_wipe_length = 10.0);

    /*!
     * Add an extrusion move to a certain point, optionally with a different flow than the one in the \p config.
     * 
     * \param p The point to extrude to
     * \param config The config with which to extrude
     * \param space_fill_type Of what space filling type this extrusion move is a part
     * \param flow A modifier of the extrusion width which would follow from the \p config
     * \param speed_factor (optional) A factor the travel speed will be multipled by.
     * \param spiralize Whether to gradually increase the z while printing. (Note that this path may be part of a sequence of spiralized paths, forming one polygon)
     * \param fan_speed fan speed override for this path
     */
    void addExtrusionMove(Point p, const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio& flow = 1.0_r, bool spiralize = false, Ratio speed_factor = 1.0_r, double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT);

    /*!
     * Add polygon to the gcode starting at vertex \p startIdx
     * \param polygon The polygon
     * \param startIdx The index of the starting vertex of the \p polygon
     * \param backwards Print this polygon in reverse direction.
     * \param config The config with which to print the polygon lines
     * \param wall_0_wipe_dist The distance to travel along the polygon after it has been laid down, in order to wipe the start and end of the wall together
     * \param spiralize Whether to gradually increase the z height from the normal layer height to the height of the next layer over this polygon
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param always_retract Whether to force a retraction when moving to the start of the polygon (used for outer walls)
     */
    void addPolygon(ConstPolygonRef polygon, int startIdx, const bool reverse, const GCodePathConfig& config, coord_t wall_0_wipe_dist = 0, bool spiralize = false, const Ratio& flow_ratio = 1.0_r, bool always_retract = false);

    /*!
     * Add polygons to the gcode with optimized order.
     * 
     * When \p spiralize is true, each polygon will gradually increase from a z
     * corresponding to this layer to the z corresponding to the next layer.
     * Doing this for each polygon means there is a chance for the print head to
     * crash into already printed parts, but doing it for the last polygon only
     * would mean you are printing half of the layer in non-spiralize mode,
     * while each layer starts with a different part. Two towers would result in
     * alternating spiralize and non-spiralize layers.
     * 
     * \param polygons The polygons.
     * \param config The config with which to print the polygon lines.
     * for each given segment (optionally nullptr).
     * \param z_seam_config Optional configuration for z-seam.
     * \param wall_0_wipe_dist The distance to travel along each polygon after
     * it has been laid down, in order to wipe the start and end of the wall
     * together.
     * \param spiralize Whether to gradually increase the z height from the
     * normal layer height to the height of the next layer over each polygon
     * printed.
     * \param flow_ratio The ratio with which to multiply the extrusion amount.
     * \param always_retract Whether to force a retraction when moving to the
     * start of the polygon (used for outer walls).
     * \param reverse_order Adds polygons in reverse order.
     * \param start_near_location Start optimising the path near this location.
     * If unset, this causes it to start near the last planned location.
     */
    void addPolygonsByOptimizer(const Polygons& polygons, const GCodePathConfig& config, const ZSeamConfig& z_seam_config = ZSeamConfig(), coord_t wall_0_wipe_dist = 0, bool spiralize = false, const Ratio flow_ratio = 1.0_r, bool always_retract = false, bool reverse_order = false, const std::optional<Point> start_near_location = std::optional<Point>());

    /*!
     * Add a single line that is part of a wall to the gcode.
     * \param p0 The start vertex of the line
     * \param p1 The end vertex of the line
     * \param settings The settings which should apply to this line added to the layer plan.
     * \param non_bridge_config The config with which to print the wall lines that are not spanning a bridge
     * \param bridge_config The config with which to print the wall lines that are spanning a bridge
     * \param flow The ratio with which to multiply the extrusion amount
     * \param non_bridge_line_volume A pseudo-volume that is derived from the print speed and flow of the non-bridge lines that have preceeded this line
     * \param speed_factor This modifies the print speed when accelerating after a bridge line
     * \param distance_to_bridge_start The distance along the wall from p0 to the first bridge segment
     */

    void addWallLine(const Point& p0, const Point& p1, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, float flow, float& non_bridge_line_volume, Ratio speed_factor, double distance_to_bridge_start);

    /*!
     * Add a wall to the g-code starting at vertex \p start_idx
     * \param wall The vertices of the wall to add.
     * \param start_idx The index of the starting vertex to start at.
     * \param settings The settings which should apply to this wall added to the layer plan.
     * \param non_bridge_config The config with which to print the wall lines
     * that are not spanning a bridge.
     * \param bridge_config The config with which to print the wall lines that
     * are spanning a bridge.
     * \param wall_0_wipe_dist The distance to travel along the wall after it
     * has been laid down, in order to wipe the start and end of the wall
     * \param flow_ratio The ratio with which to multiply the extrusion amount.
     * \param always_retract Whether to force a retraction when moving to the
     * start of the wall (used for outer walls).
     */
    void addWall(ConstPolygonRef wall, int start_idx, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract);

    /*!
     * Add a wall to the g-code starting at vertex \p start_idx
     * \param wall The wall of type ExtrusionJunctions
     * \param start_idx The index of the starting vertex to start at.
     * \param mesh The current mesh being added to the layer plan.
     * \param non_bridge_config The config with which to print the wall lines
     * that are not spanning a bridge.
     * \param bridge_config The config with which to print the wall lines that
     * are spanning a bridge
     * \param wall_0_wipe_dist The distance to travel along the wall after it
     * has been laid down, in order to wipe the start and end of the wall
     * \param flow_ratio The ratio with which to multiply the extrusion amount.
     * \param always_retract Whether to force a retraction when moving to the
     * start of the wall (used for outer walls).
     * \param is_closed Whether this wall is a closed loop (a polygon) or not (a
     * polyline).
     * \param is_reversed Whether to print this wall in reverse direction.
     * \param is_linked_path Whether the path is a continuation off the previous path
     */
    void addWall(const LineJunctions& wall, int start_idx, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract, const bool is_closed, const bool is_reversed, const bool is_linked_path);

    /*!
     * Add an infill wall to the g-code
     * Walls should not be an empty vector at this time
     * \param wall he wall as ExtrusionJunctions
     * \param path_config The config with which to print the wall lines
     */
    void addInfillWall(const LineJunctions& wall, const GCodePathConfig& path_config, bool force_retract);

    /*!
     * Add walls (polygons) to the gcode with optimized order.
     * \param walls The walls
     * \param settings The settings which should apply to these walls added to the layer plan.
     * \param non_bridge_config The config with which to print the wall lines that are not spanning a bridge
     * \param bridge_config The config with which to print the wall lines that are spanning a bridge
     * \param z_seam_config Optional configuration for z-seam
     * \param wall_0_wipe_dist The distance to travel along each wall after it has been laid down, in order to wipe the start and end of the wall together
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param always_retract Whether to force a retraction when moving to the start of a wall (used for outer walls)
     */
    void addWalls(const Polygons& walls, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, const ZSeamConfig& z_seam_config = ZSeamConfig(), coord_t wall_0_wipe_dist = 0, float flow_ratio = 1.0, bool always_retract = false);
    void addWalls(const PathJunctions& walls, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, const ZSeamConfig& z_seam_config = ZSeamConfig(), coord_t wall_0_wipe_dist = 0, float flow_ratio = 1.0, bool always_retract = false);

    /*!
     * Add lines to the gcode with optimized order.
     * \param polygons The lines
     * \param config The config of the lines
     * \param space_fill_type The type of space filling used to generate the line segments (should be either Lines or PolyLines!)
     * \param enable_travel_optimization Whether to enable some potentially time consuming optimization of order the lines are printed to reduce the travel time required.
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param near_start_location Optional: Location near where to add the first line. If not provided the last position is used.
     * \param fan_speed optional fan speed override for this path
     */
    void addLinesByOptimizer(const Polygons& polygons, const GCodePathConfig& config, SpaceFillType space_fill_type, bool enable_travel_optimization = false, int wipe_dist = 0, float flow_ratio = 1.0, std::optional<Point> near_start_location = std::optional<Point>(), double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT);

    /*!
     * Add polygons to the g-code with monotonic order.
     * \param polygons The lines to add.
     * \param config The settings to print those lines with.
     * \param space_fill_type The type of space filling used to generate the
     * line segments (should be either Lines or PolyLines!)
     * \param monotonic_direction The directions in which to sort the lines
     * monotonically.
     * \param max_adjacent_distance With a monotonic order, adjacent lines must
     * be printed in a certain direction. Lines that are not adjacent may be
     * printed in any order. This limit is the longest distance at which two
     * lines are still considered to be adjacent.
     * \param wipe_dist The distance wiped without extruding after laying down a
     * line.
     * \param flow_ratio The ratio with which to multiply the extrusion amount.
     * \param fan_speed Fan speed override for this path.
     */
    void addLinesMonotonic(const Polygons& polygons, const GCodePathConfig& config, const SpaceFillType space_fill_type, const AngleRadians monotonic_direction, const coord_t max_adjacent_distance, const coord_t wipe_dist = 0, const Ratio flow_ratio = 1.0_r, const double fan_speed = 100.0);

    /*!
     * Add a spiralized slice of wall that is interpolated in X/Y between \p last_wall and \p wall.
     *
     * At the start of the wall slice, the points are closest to \p last_wall and at the end of the polygon, the points are closest to \p wall.
     *
     * \param config The config with which to print the lines
     * \param wall The wall polygon to be spiralized
     * \param last_wall The wall polygon that was spiralized below the current polygon (or \p wall if this is the first spiralized layer)
     * \param seam_vertex_idx The index of this wall slice's seam vertex
     * \param last_seam_vertex_idx The index of the seam vertex in the last wall (or -1 if this is the first spiralized layer)
     * \param is_top_layer true when the top layer of the spiral is being printed
     * \param is_bottom_layer true when the bottom layer of the spiral is being printed
     */
    void spiralizeWallSlice(const GCodePathConfig& config, ConstPolygonRef wall, ConstPolygonRef last_wall, int seam_vertex_idx, int last_seam_vertex_idx, const bool is_top_layer, const bool is_bottom_layer);


    /*!
     * Given a wall polygon and a start vertex index, return the index of the first vertex that is supported (is not above air)
     *
     * Uses bridge_wall_mask and overhang_mask to determine where there is air below
     *
     * \param wall The wall polygon
     * \param start_idx The index of the starting vertex of \p wall
     * \return The index of the first supported vertex - if no vertices are supported, start_idx is returned
     */
    template<typename T>
    unsigned locateFirstSupportedVertex(const T& wall, const unsigned start_idx) const
    {
        if (bridge_wall_mask.empty() && overhang_mask.empty())
        {
            return start_idx;
        }

        Polygons air_below(bridge_wall_mask.unionPolygons(overhang_mask));

        unsigned curr_idx = start_idx;

        while (true)
        {
            const Point& vertex = cura::make_point(wall[curr_idx]);
            if (!air_below.inside(vertex, true))
            {
                // vertex isn't above air so it's OK to use
                return curr_idx;
            }

            if (++curr_idx >= wall.size())
            {
                curr_idx = 0;
            }

            if (curr_idx == start_idx)
            {
                // no vertices are supported so just return the original index
                return start_idx;
            }
        }
    }

    /*!
     * Write the planned paths to gcode
     * 
     * \param gcode The gcode to write the planned paths to
     */
    void writeGCode(GCodeExport& gcode);

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
     * \param gcode The gcode to write the planned paths to.
     * \param extruder_plan_idx The index of the current extruder plan.
     * \param path_idx The index into LayerPlan::paths for the next path to be
     * written to GCode.
     * \param layer_thickness The height of the current layer.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(GCodeExport& gcode, const size_t extruder_plan_idx, const size_t path_idx, const coord_t layer_thickness);

    /*!
     * Applying speed corrections for minimal layer times and determine the fanSpeed. 
     * 
     * \param starting_position The position of the print head when the first extruder plan of this layer starts
     */
    void processFanSpeedAndMinimalLayerTime(Point starting_position);
    
    /*!
     * Add a travel move to the layer plan to move inside the current layer part by a given distance away from the outline.
     * This is supposed to be called when the nozzle is around the boundary of a layer part, not when the nozzle is in the middle of support, or in the middle of the air.
     * 
     * \param distance The distance to the comb boundary after we moved inside it.
     */
    void moveInsideCombBoundary(const coord_t distance);

    /*!
     * Apply back-pressure compensation to this layer-plan.
     * Since the total (filament) pressure in a feeder-system is not only dependent on the pressure that exists between the nozzle and the
     * feed-mechanism (which should be near-constant on a bowden style setup), but _also_ between the nozzle and the last-printed layer. This last
     * type is called 'back-pressure'. In this function, properties of the outflow (of the paths for each extruder) are adjusted so that
     * the back-pressure is compensated for. This is conjectured to be especially important if the printer has a Bowden-tube style setup.
     */
    void applyBackPressureCompensation();

private:

    /*!
     * \brief Compute the preferred or minimum combing boundary
     *
     * Minimum combing boundary:
     *  - If CombingMode::ALL: Add the outline offset (skin, infill and inner walls).
     *  - If CombingMode::NO_SKIN: Add the outline offset, subtract skin (infill and inner walls).
     *  - If CombingMode::INFILL: Add the infill (infill only).
     *
     * Preferred combing boundary:
     *  - If CombingMode::ALL: Add the increased outline offset (skin, infill and part of the inner walls).
     *  - If CombingMode::NO_SKIN: Add the increased outline offset, subtract skin (infill and part of the inner walls).
     *  - If CombingMode::INFILL: Add the infill (infill only).
     *
     * \param boundary_type The boundary type to compute.
     * \return the combing boundary or an empty Polygons if no combing is required
     */
    Polygons computeCombBoundary(const CombBoundary boundary_type);
};

}//namespace cura

#endif // LAYER_PLAN_H
