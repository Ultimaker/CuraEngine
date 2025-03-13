// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef LAYER_PLAN_H
#define LAYER_PLAN_H

#include "ExtruderPlan.h"
#include "FanSpeedLayerTime.h"
#include "InsetOrderOptimizer.h"
#include "PathOrderOptimizer.h"
#include "SpaceFillType.h"
#include "gcodeExport.h"
#include "geometry/LinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/Polygon.h"
#include "pathPlanning/GCodePath.h"
#include "pathPlanning/NozzleTempInsert.h"
#include "pathPlanning/TimeMaterialEstimates.h"
#include "raft.h"
#include "settings/PathConfigStorage.h"
#include "settings/types/LayerIndex.h"
#include "utils/ExtrusionJunction.h"

#ifdef BUILD_TESTS
#include <gtest/gtest_prod.h> //Friend tests, so that they can inspect the privates.
#endif

#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace cura
{

class Comb;
class SliceDataStorage;
class LayerPlanBuffer;

template<typename PathType>
class PathAdapter;

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
#ifdef BUILD_TESTS
    friend class AddTravelTest;
    friend class FffGcodeWriterTest_SurfaceGetsExtraInfillLinesUnderIt_Test;
#endif

public:
    struct OverhangMask
    {
        Shape supported_region;
        Ratio speed_ratio;
    };

    const PathConfigStorage configs_storage_; //!< The line configs for this layer for each feature type
    const coord_t z_;
    coord_t final_travel_z_;
    bool mode_skip_agressive_merge_; //!< Whether to give every new path the 'skip_agressive_merge_hint' property (see GCodePath); default is false.

private:
    // Indicates how coasting should be processed on the given path.
    enum class ApplyCoasting
    {
        NoCoasting, // Do not apply coasting on this path, extrude it normally
        CoastEntirePath, // Fully coast this path, i.e. replace it by travel moves
        PartialCoasting // Extrude the first part of the path and coast the end
    };

    struct PathCoasting
    {
        ApplyCoasting apply_coasting{ ApplyCoasting::NoCoasting };
        size_t coasting_start_index{ 0 };
        Point3LL coasting_start_pos;
    };

    const SliceDataStorage& storage_; //!< The polygon data obtained from FffPolygonProcessor
    const LayerIndex layer_nr_; //!< The layer number of this layer plan
    const bool is_initial_layer_; //!< Whether this is the first layer (which might be raft)
    const Raft::LayerType layer_type_; //!< Which part of the raft, airgap or model this layer is.
    coord_t layer_thickness_;

    std::vector<Point2LL> layer_start_pos_per_extruder_; //!< The starting position of a layer for each extruder
    std::vector<bool> has_prime_tower_planned_per_extruder_; //!< For each extruder, whether the prime tower is planned yet or not.
    std::optional<Point2LL> last_planned_position_; //!< The last planned XY position of the print head (if known)

    std::shared_ptr<const SliceMeshStorage> current_mesh_; //!< The mesh of the last planned move.

    /*!
     * Whether the skirt or brim polygons have been processed into planned paths
     * for each extruder train.
     */
    bool skirt_brim_is_processed_[MAX_EXTRUDERS];

    std::vector<ExtruderPlan> extruder_plans_; //!< should always contain at least one ExtruderPlan

    size_t last_extruder_previous_layer_; //!< The last id of the extruder with which was printed in the previous layer
    ExtruderTrain* last_planned_extruder_; //!< The extruder for which a move has most recently been planned.

    std::optional<Point2LL> first_travel_destination_; //!< The destination of the first (travel) move (if this layer is not empty)
    bool first_travel_destination_is_inside_; //!< Whether the destination of the first planned travel move is inside a layer part
    std::optional<std::pair<Acceleration, Velocity>> first_extrusion_acc_jerk_; //!< The acceleration and jerk rates of the first extruded move (if this layer is not empty).
    std::optional<std::pair<Acceleration, Velocity>> next_layer_acc_jerk_; //!< If there is a next layer, the first acceleration and jerk it starts with.
    bool was_inside_; //!< Whether the last planned (extrusion) move was inside a layer part
    bool is_inside_; //!< Whether the destination of the next planned travel move is inside a layer part
    Shape comb_boundary_minimum_; //!< The minimum boundary within which to comb, or to move into when performing a retraction.
    Shape comb_boundary_preferred_; //!< The boundary preferably within which to comb, or to move into when performing a retraction.
    Comb* comb_;
    coord_t comb_move_inside_distance_; //!< Whenever using the minimum boundary for combing it tries to move the coordinates inside by this distance after calculating the combing.
    Shape bridge_wall_mask_; //!< The regions of a layer part that are not supported, used for bridging
    std::vector<OverhangMask> overhang_masks_; //!< The regions of a layer part where the walls overhang, calculated for multiple overhang angles. The latter is the most
                                               //!< overhanging. For a visual explanation of the result, see doc/gradual_overhang_speed.svg
    Shape seam_overhang_mask_; //!< The regions of a layer part where the walls overhang, specifically as defined for the seam

    Shape roofing_mask_; //!< The regions of a layer part where the walls are exposed to the air above
    Shape flooring_mask_; //!< The regions of a layer part where the walls are exposed to the air below

    bool currently_overhanging_{ false }; //!< Indicates whether the last extrusion move was overhanging
    coord_t current_overhang_length_{ 0 }; //!< When doing consecutive overhanging moves, this is the current accumulated overhanging length
    coord_t max_overhang_length_{ 0 }; //!< From all consecutive overhanging moves in the layer, this is the longest one

    bool min_layer_time_used = false; //!< Wether or not the minimum layer time (cool_min_layer_time) was actually used in this layerplan.

    const std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_;

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
     * \param z_offset (optional) Vertical offset w.r.t current layer height, defaults to 0
     * \param flow (optional) A ratio for the extrusion speed
     * \param spiralize Whether to gradually increase the z while printing. (Note that this path may be part of a sequence of spiralized paths, forming one polygon)
     * \param speed_factor (optional) a factor which the speed will be multiplied by.
     * \return A path with the given config which is now the last path in LayerPlan::paths
     */
    GCodePath* getLatestPathWithConfig(
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const coord_t z_offset = 0,
        const Ratio flow = 1.0_r,
        const Ratio width_factor = 1.0_r,
        const bool spiralize = false,
        const Ratio speed_factor = 1.0_r);

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
    LayerPlan(
        const SliceDataStorage& storage,
        LayerIndex layer_nr,
        coord_t z,
        coord_t layer_height,
        size_t start_extruder,
        const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder,
        coord_t comb_boundary_offset,
        coord_t comb_move_inside_distance,
        coord_t travel_avoid_distance);

    ~LayerPlan();

    void overrideFanSpeeds(double speed);

    /*!
     * \brief Get the last planned extruder train.
     * \return The last planned extruder.
     */
    ExtruderTrain* getLastPlannedExtruderTrain();

    const Shape* getCombBoundaryInside() const;

    LayerIndex getLayerNr() const;

    /*!
     * Get the last planned position, or if no position has been planned yet, the user specified layer start position.
     *
     * \warning The layer start position might be outside of the build plate!
     */
    Point2LL getLastPlannedPositionOrStartingPosition() const;

    /*!
     * return whether the last position planned was inside the mesh (used in combing)
     */
    bool getIsInsideMesh() const;

    /*!
     * Whether the prime tower is already planned for the specified extruder.
     * \param extruder_nr The extruder to check.
     */
    bool getPrimeTowerIsPlanned(size_t extruder_nr) const;

    /*!
     * Mark the prime tower as planned for the specified extruder.
     * \param extruder_nr The extruder to mark as having its prime tower
     * planned.
     */
    void setPrimeTowerIsPlanned(size_t extruder_nr);

    /*!
     * Whether the prime tower extra base is already planned.
     */
    bool getPrimeTowerBaseIsPlanned() const;

    /*!
     * Mark the prime tower extra base as planned.
     */
    void setPrimeTowerBaseIsPlanned();

    /*!
     * Whether the prime tower extra inset is already planned.
     */
    bool getPrimeTowerInsetIsPlanned() const;

    /*!
     * Mark the prime tower extra inset as planned.
     */
    void setPrimeTowerInsetIsPlanned();

    bool getSkirtBrimIsPlanned(unsigned int extruder_nr) const;

    void setSkirtBrimIsPlanned(unsigned int extruder_nr);

    /*!
     * Get the destination state of the first travel move.
     * This consists of the location and whether the destination was inside the model, or e.g. to support
     *
     * Returns nothing if the layer is empty and no travel move was ever made.
     */
    std::optional<std::pair<Point2LL, bool>> getFirstTravelDestinationState() const;

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
    void setMesh(const std::shared_ptr<const SliceMeshStorage>& mesh);

    /*!
     * Set bridge_wall_mask.
     *
     * \param polys The unsupported areas of the part currently being processed that will require bridges.
     */
    void setBridgeWallMask(const Shape& polys);

    /*!
     * Set overhang_masks.
     *
     * \param masks The overhung areas of the part currently being processed that will require modified print settings
     */
    void setOverhangMasks(const std::vector<OverhangMask>& masks);

    /*!
     * Set seam_overhang_mask.
     *
     * \param polys The overhung areas of the part currently being processed that will require modified print settings w.r.t. seams
     */
    void setSeamOverhangMask(const Shape& polys);

    /*!
     * Get the seam overhang mask, which contains the areas where we don't want to place the seam because they are overhanding
     */
    const Shape& getSeamOverhangMask() const;

    /*!
     * Set roofing_mask.
     *
     * \param polys The areas of the part currently being processed that will require roofing.
     */
    void setRoofingMask(const Shape& polys);

    /*!
     * Set flooring_mask.
     *
     * \param shape The areas of the part currently being processed that will require flooring.
     */
    void setFlooringMask(const Shape& shape);

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
     * \param force_retract Whether to force a retraction to occur.
     */
    GCodePath& addTravel(const Point2LL& p, const bool force_retract = false, const coord_t z_offset = 0);

    /*!
     * Add a travel path to a certain point and retract if needed.
     *
     * No combing is performed.
     *
     * \param p The point to travel to
     * \param path (optional) The travel path to which to add the point \p p
     */
    GCodePath& addTravel_simple(const Point2LL& p, GCodePath* path = nullptr);

    /*!
     * Plan a prime blob at the current location.
     */
    void planPrime(double prime_blob_wipe_length = 10.0);

    /*!
     * Add an extrusion move to a certain point, optionally with a different flow than the one in the \p config.
     *
     * \param p The point to extrude to
     * \param config The config with which to extrude
     * \param space_fill_type Of what space filling type this extrusion move is
     * a part.
     * \param flow A modifier of the flow rate which would follow from the
     * \p config.
     * \param width_factor A modifier of the line width which would follow from
     * the \p config.
     * \param speed_factor (optional) A factor the travel speed will be
     * multiplied by.
     * \param spiralize Whether to gradually increase the z while printing.
     * (Note that this path may be part of a sequence of spiralized paths,
     * forming one polygon.)
     * \param fan_speed Fan speed override for this path.
     */
    void addExtrusionMove(
        const Point3LL& p,
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const Ratio& flow = 1.0_r,
        const Ratio width_factor = 1.0_r,
        const bool spiralize = false,
        const Ratio speed_factor = 1.0_r,
        const double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT,
        const bool travel_to_z = true);

    void addExtrusionMoveWithGradualOverhang(
        const Point3LL& p,
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const Ratio& flow = 1.0_r,
        const Ratio width_factor = 1.0_r,
        const bool spiralize = false,
        const Ratio speed_factor = 1.0_r,
        const double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT,
        const bool travel_to_z = true);

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
     * \param scarf_seam Indicates whether we may use a scarf seam for the path
     * \param smooth_speed Indicates whether we may use a speed gradient for the path
     */
    void addPolygon(
        const Polygon& polygon,
        int startIdx,
        const bool reverse,
        const Settings& settings,
        const GCodePathConfig& config,
        coord_t wall_0_wipe_dist = 0,
        bool spiralize = false,
        const Ratio& flow_ratio = 1.0_r,
        bool always_retract = false,
        bool scarf_seam = false,
        bool smooth_speed = false);

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
     * \param scarf_seam Indicates whether we may use a scarf seam for the path
     * \param smooth_speed Indicates whether we may use a speed gradient for the path
     */
    void addPolygonsByOptimizer(
        const Shape& polygons,
        const GCodePathConfig& config,
        const Settings& settings,
        const ZSeamConfig& z_seam_config = ZSeamConfig(),
        coord_t wall_0_wipe_dist = 0,
        bool spiralize = false,
        const Ratio flow_ratio = 1.0_r,
        bool always_retract = false,
        bool reverse_order = false,
        const std::optional<Point2LL> start_near_location = std::optional<Point2LL>(),
        bool scarf_seam = false,
        bool smooth_acceleration = false);

    /*!
     * Add a single line that is part of a wall to the gcode.
     * \param p0 The start vertex of the line.
     * \param p1 The end vertex of the line.
     * \param settings The settings which should apply to this line added to the
     * layer plan.
     * \param default_config The config with which to print the wall lines
     * that are not spanning a bridge or are exposed to air.
     * \param roofing_config The config with which to print the wall lines
     * that are exposed to air.
     * \param flooring_config The config with which to print the wall lines
     * that are exposed to air below.
     * \param bridge_config The config with which to print the wall lines that
     * are spanning a bridge.
     * \param flow The ratio with which to multiply the extrusion amount.
     * \param width_ratio The ratio with which to multiply the line width.
     * \param non_bridge_line_volume A pseudo-volume that is derived from the
     * print speed and flow of the non-bridge lines that have preceded this
     * line.
     * \param speed_factor This modifies the print speed when accelerating after
     * a bridge line.
     * \param distance_to_bridge_start The distance along the wall from p0 to
     * the first bridge segment.
     */
    void addWallLine(
        const Point3LL& p0,
        const Point3LL& p1,
        const Settings& settings,
        const GCodePathConfig& default_config,
        const GCodePathConfig& roofing_config,
        const GCodePathConfig& flooring_config,
        const GCodePathConfig& bridge_config,
        double flow,
        const Ratio width_factor,
        double& non_bridge_line_volume,
        Ratio speed_factor,
        double distance_to_bridge_start,
        const bool travel_to_z = true);

    /*!
     * Add a wall to the g-code starting at vertex \p start_idx
     * \param wall The vertices of the wall to add.
     * \param start_idx The index of the starting vertex to start at.
     * \param settings The settings which should apply to this wall added to the layer plan.
     * \param default_config The config with which to print the wall lines
     * that are not spanning a bridge or are exposed to air.
     * \param roofing_config The config with which to print the wall lines
     * that are exposed to air.
     * \param flooring_config The config with which to print the wall lines
     * that are exposed to air below.
     * \param wall_0_wipe_dist The distance to travel along the wall after it
     * has been laid down, in order to wipe the start and end of the wall
     * \param flow_ratio The ratio with which to multiply the extrusion amount.
     * \param always_retract Whether to force a retraction when moving to the
     * start of the wall (used for outer walls).
     */
    void addWall(
        const Polygon& wall,
        size_t start_idx,
        const Settings& settings,
        const GCodePathConfig& default_config,
        const GCodePathConfig& roofing_config,
        const GCodePathConfig& flooring_config,
        const GCodePathConfig& bridge_config,
        coord_t wall_0_wipe_dist,
        double flow_ratio,
        bool always_retract);

    /*!
     * Add a wall to the g-code starting at vertex \p start_idx
     * \param wall The wall of type ExtrusionJunctions
     * \param start_idx The index of the starting vertex to start at.
     * \param mesh The current mesh being added to the layer plan.
     * \param default_config The config with which to print the wall lines
     * that are not spanning a bridge or are exposed to air.
     * \param roofing_config The config with which to print the wall lines
     * that are exposed to air.
     * \param flooring_config The config with which to print the wall lines
     * that are exposed to air below.
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
     * \param scarf_seam Indicates whether we may use a scarf seam for the path
     * \param smooth_speed Indicates whether we may use a speed gradient for the path
     */
    void addWall(
        const ExtrusionLine& wall,
        size_t start_idx,
        const Settings& settings,
        const GCodePathConfig& default_config,
        const GCodePathConfig& roofing_config,
        const GCodePathConfig& flooring_config,
        const GCodePathConfig& bridge_config,
        coord_t wall_0_wipe_dist,
        double flow_ratio,
        bool always_retract,
        const bool is_closed,
        const bool is_reversed,
        const bool is_linked_path,
        const bool scarf_seam = false,
        const bool smooth_speed = false);

    /*!
     * Add an infill wall to the g-code
     * Walls should not be an empty vector at this time
     * \param wall he wall as ExtrusionJunctions
     * \param path_config The config with which to print the wall lines
     */
    void addInfillWall(const ExtrusionLine& wall, const GCodePathConfig& path_config, bool force_retract);

    /*!
     * Add walls (polygons) to the gcode with optimized order.
     * \param walls The walls
     * \param settings The settings which should apply to these walls added to the layer plan.
     * \param default_config The config with which to print the wall lines
     * that are not spanning a bridge or are exposed to air.
     * \param roofing_config The config with which to print the wall lines
     * that are exposed to air above.
     * \param flooring_config The config with which to print the wall lines
     * that are exposed to air below.
     * \param bridge_config The config with which to print the wall lines that are spanning a bridge
     * \param z_seam_config Optional configuration for z-seam
     * \param wall_0_wipe_dist The distance to travel along each wall after it has been laid down, in order to wipe the start and end of the wall together
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param always_retract Whether to force a retraction when moving to the start of a wall (used for outer walls)
     * \param alternate_inset_direction_modifier Whether to alternate the direction of the walls for each inset.
     */
    void addWalls(
        const Shape& walls,
        const Settings& settings,
        const GCodePathConfig& default_config,
        const GCodePathConfig& roofing_config,
        const GCodePathConfig& flooring_config,
        const GCodePathConfig& bridge_config,
        const ZSeamConfig& z_seam_config = ZSeamConfig(),
        coord_t wall_0_wipe_dist = 0,
        double flow_ratio = 1.0,
        bool always_retract = false);

    /*!
     * Add lines to the gcode with optimized order.
     * \param lines The lines
     * \param config The config of the lines
     * \param space_fill_type The type of space filling used to generate the line segments (should be either Lines or PolyLines!)
     * \param enable_travel_optimization Whether to enable some potentially time consuming optimization of order the lines are printed to reduce the travel time required.
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param near_start_location Optional: Location near where to add the first line. If not provided the last position is used.
     * \param fan_speed optional fan speed override for this path
     * \param reverse_print_direction Whether to reverse the optimized order and their printing direction.
     * \param order_requirements Pairs where first needs to be printed before second. Pointers are pointing to elements of \p lines
     */
    template<class LineType>
    void addLinesByOptimizer(
        const LinesSet<LineType>& lines,
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const bool enable_travel_optimization = false,
        const coord_t wipe_dist = 0,
        const Ratio flow_ratio = 1.0,
        const std::optional<Point2LL> near_start_location = std::optional<Point2LL>(),
        const double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT,
        const bool reverse_print_direction = false,
        const std::unordered_multimap<const Polyline*, const Polyline*>& order_requirements = PathOrderOptimizer<const Polyline*>::no_order_requirements_);

    /*!
     * Add lines to the gcode with optimized order.
     * \param lines The lines
     * \param config The config of the lines
     * \param space_fill_type The type of space filling used to generate the line segments (should be either Lines or PolyLines!)
     * \param enable_travel_optimization Whether to enable some potentially time consuming optimization of order the lines are printed to reduce the travel time required.
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param near_start_location Optional: Location near where to add the first line. If not provided the last position is used.
     * \param fan_speed optional fan speed override for this path
     * \param reverse_print_direction Whether to reverse the optimized order and their printing direction.
     * \param order_requirements Pairs where first needs to be printed before second. Pointers are pointing to elements of \p lines
     */
    void addLinesByOptimizer(
        const MixedLinesSet& lines,
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const bool enable_travel_optimization = false,
        const coord_t wipe_dist = 0,
        const Ratio flow_ratio = 1.0,
        const std::optional<Point2LL> near_start_location = std::optional<Point2LL>(),
        const double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT,
        const bool reverse_print_direction = false,
        const std::unordered_multimap<const Polyline*, const Polyline*>& order_requirements = PathOrderOptimizer<const Polyline*>::no_order_requirements_);

    /*!
     * Add lines to the g-code with monotonic order.
     * \param lines The lines to add.
     * \param config The settings to print those lines with.
     * \param space_fill_type The type of space filling used to generate the
     * line segments (should be either Lines or PolyLines!)
     * \param monotonic_direction The directions in which to sort the lines
     * monotonically.
     * \param max_adjacent_distance With a monotonic order, adjacent lines must
     * be printed in a certain direction. Lines that are not adjacent may be
     * printed in any order. This limit is the longest distance at which two
     * lines are still considered to be adjacent.
     * \param exclude_distance Excuse lines that are wholly closer to the walls than this, and of which the neighbours
     * they would be connected to are also close to the walls (for efficiencies sake, this is really 'shorter than this
     * distance and having a midpoint within this distance from the walls' from the required monotonicity. An exlcude
     * distance of 0 (the default value) will truly lay out the lines monotonically however.
     * \param wipe_dist The distance wiped without extruding after laying down a
     * line.
     * \param flow_ratio The ratio with which to multiply the extrusion amount.
     * \param fan_speed Fan speed override for this path.
     */
    void addLinesMonotonic(
        const Shape& area,
        const OpenLinesSet& lines,
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const AngleRadians monotonic_direction,
        const coord_t max_adjacent_distance,
        const coord_t exclude_distance = 0,
        const coord_t wipe_dist = 0,
        const Ratio flow_ratio = 1.0_r,
        const double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT);

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
    void spiralizeWallSlice(
        const GCodePathConfig& config,
        const Polygon& wall,
        const Polygon& last_wall,
        int seam_vertex_idx,
        int last_seam_vertex_idx,
        const bool is_top_layer,
        const bool is_bottom_layer);

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
     * \return Whether the path should be an extruder switch retracted path
     */
    bool makeRetractSwitchRetract(unsigned int extruder_plan_idx, unsigned int path_idx);

    /*!
     * Applying speed corrections for minimal layer times and determine the fanSpeed.
     *
     * \param starting_position The position of the print head when the first extruder plan of this layer starts
     */
    void processFanSpeedAndMinimalLayerTime(Point2LL starting_position);

    /*!
     * Add a travel move to the layer plan to move inside the current layer part
     * by a given distance away from the outline.
     *
     * This is supposed to be called when the nozzle is around the boundary of a
     * layer part, not when the nozzle is in the middle of support, or in the
     * middle of the air.
     * \param distance The distance to the comb boundary after we moved inside
     * it.
     * \param part If given, stay within the boundary of this part.
     */
    void moveInsideCombBoundary(const coord_t distance, const std::optional<SliceLayerPart>& part = std::nullopt, GCodePath* path = nullptr);

    /*!
     * If enabled, apply the modify plugin to the layer-plan.
     */
    void applyModifyPlugin();

    /*!
     * Apply back-pressure compensation to this layer-plan.
     * Since the total (filament) pressure in a feeder-system is not only dependent on the pressure that exists between the nozzle and the
     * feed-mechanism (which should be near-constant on a bowden style setup), but _also_ between the nozzle and the last-printed layer. This last
     * type is called 'back-pressure'. In this function, properties of the outflow (of the paths for each extruder) are adjusted so that
     * the back-pressure is compensated for. This is conjectured to be especially important if the printer has a Bowden-tube style setup.
     */
    void applyBackPressureCompensation();

    /*!
     * If enabled, applies the gradual flow acceleration splitting, that improves printing quality when printing at very high speed,
     * especially with a bowden extruder.
     */
    void applyGradualFlow();

    /*!
     * Gets the mesh being printed first on this layer
     */
    std::shared_ptr<const SliceMeshStorage> findFirstPrintedMesh() const;

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
     * \return the combing boundary or an empty Shape if no combing is required
     */
    Shape computeCombBoundary(const CombBoundary boundary_type);

    /*!
     * Add order optimized lines to the gcode.
     * \param lines The lines in order
     * \param config The config of the lines
     * \param space_fill_type The type of space filling used to generate the line segments (should be either Lines or PolyLines!)
     * \param wipe_dist (optional) the distance wiped without extruding after laying down a line.
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param fan_speed optional fan speed override for this path
     */
    void addLinesInGivenOrder(
        const std::vector<PathOrdering<const Polyline*>>& lines,
        const GCodePathConfig& config,
        const SpaceFillType space_fill_type,
        const coord_t wipe_dist,
        const Ratio flow_ratio,
        const double fan_speed);

    /*!
     *  @brief Send a GCodePath line to the communication object, applying proper Z offsets
     *  @param path The path to be sent
     *  @param position The start position (which is not included in the path points)
     *  @param extrude_speed The actual used extrusion speed
     */
    void sendLineTo(const GCodePath& path, const Point3LL& position, const double extrude_speed);

    /*!
     *  @brief Write a travel move and properly apply the various Z offsets
     *  @param gcode The actual GCode exporter
     *  @param position The position to move to. The Z coordinate is an offset to the current layer position
     *  @param speed The actual used speed
     *  @param path_z_offset The global path Z offset to be applied
     *  @note This function is to be used when dealing with 3D coordinates. If you have 2D coordinates, just call gcode.writeTravel()
     */
    void writeTravelRelativeZ(GCodeExport& gcode, const Point3LL& position, const Velocity& speed, const coord_t path_z_offset);

    /*!
     * \brief Write an extrusion move and properly apply the various Z offsets
     * \param gcode The actual GCode exporter
     * \param position The position to move to. The Z coordinate is an offset to the current layer position
     * \param speed The actual used speed
     * \param path_z_offset The global path Z offset to be applied
     * \param extrusion_mm3_per_mm The desired flow rate
     * \param feature The current feature being printed
     * \param update_extrusion_offset whether to update the extrusion offset to match the current flow rate
     */
    void writeExtrusionRelativeZ(
        GCodeExport& gcode,
        const Point3LL& position,
        const Velocity& speed,
        const coord_t path_z_offset,
        double extrusion_mm3_per_mm,
        PrintFeatureType feature,
        bool update_extrusion_offset = false);

    /*!
     * \brief Alias for a function definition that adds an extrusion segment
     * \param start The start position of the segment
     * \param end The end position of the segment
     * \param speed_factor The speed factor to be applied when extruding this specific segment (relative to nominal speed for the entire path)
     * \param flow_ratio The flow ratio to be applied when extruding this specific segment (relative to nominal flow for the entire path)
     * \param line_width_ratio The line width ratio to be applied when extruding this specific segment (relative to nominal line width for the entire path)
     * \param distance_to_bridge_start The calculate distance to the next bridge start, which may be irrelevant in some cases
     */
    using AddExtrusionSegmentFunction = std::function<void(
        const Point3LL& start,
        const Point3LL& end,
        const Ratio& speed_factor,
        const Ratio& flow_ratio,
        const Ratio& line_width_ratio,
        const coord_t distance_to_bridge_start)>;

    /*!
     * \brief Add a wall to the gcode with optimized order, but split into pieces in order to facilitate the scarf seam and/or speed gradient.
     * \tparam PathType The type of path to be processed, either ExtrusionLine or some subclass of Polyline
     * \param wall The full wall to be added
     * \param wall_length The pre-calculated full wall length
     * \param start_idx The index of the point where to start printing the wall
     * \param direction The direction along which to print the wall, which should be 1 or -1
     * \param max_index The last index to be used when iterating over the wall segments
     * \param default_config The config with which to print the wall lines that are not spanning a bridge or are exposed to air
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param nominal_line_width The nominal line width for the wall
     * \param min_bridge_line_len The minimum line width to allow an extrusion move to be processed as a bridge move
     * \param always_retract Whether to force a retraction when moving to the start of the polygon (used for outer walls)
     * \param is_small_feature Indicates whether the wall is so small that it should be processed differently
     * \param small_feature_speed_factor The speed factor to be applied to small feature walls
     * \param max_area_deviation The maximum allowed area deviation to split a segment into pieces
     * \param max_resolution The maximum resolution to split a segment into pieces
     * \param scarf_seam_length The length of the scarf joint seam, which may be 0 if there is none
     * \param scarf_seam_start_ratio The ratio of the line thickness to start the scarf seam with
     * \param scarf_split_distance The maximum length of a segment to apply the scarf seam gradient, longer segments will be splitted
     * \param scarf_max_z_offset The maximum Z offset te be applied at the lowest position of the scarf seam
     * \param speed_split_distance The maximum length of a segment to apply the acceleration/deceleration gradient, longer segments will be splitted
     * \param start_speed_ratio The ratio of the top speed to be applied when starting the segment, then accelerate gradually to full speed
     * \param accelerate_length The pre-calculated length of the acceleration phase
     * \param end_speed_ratio The ratio of the top speed to be applied when finishing a segment
     * \param decelerate_length The pre-calculated length of the deceleration phase
     * \param is_scarf_closure Indicates whether this function is called to make the scarf closure (overlap over the first scarf pass) or the normal first pass of the wall
     * \param compute_distance_to_bridge_start Whether we should compute the distance to start of bridge. This is
     *                                         possible only if PathType is ExtrusionLine and will be ignored otherwise.
     * \param func_add_segment The function to be called to actually add an extrusion segment with the given parameters
     * \return The index of the last traversed point, and the final position with the scarf seam
     */
    template<class PathType>
    std::tuple<size_t, Point2LL> addSplitWall(
        const PathAdapter<PathType>& wall,
        const coord_t wall_length,
        const size_t start_idx,
        const size_t max_index,
        const int direction,
        const GCodePathConfig& default_config,
        const bool always_retract,
        const bool is_small_feature,
        Ratio small_feature_speed_factor,
        const coord_t max_area_deviation,
        const auto max_resolution,
        const double flow_ratio,
        const coord_t nominal_line_width,
        const coord_t min_bridge_line_len,
        const auto scarf_seam_length,
        const auto scarf_seam_start_ratio,
        const auto scarf_split_distance,
        const coord_t scarf_max_z_offset,
        const coord_t speed_split_distance,
        const Ratio start_speed_ratio,
        const coord_t accelerate_length,
        const Ratio end_speed_ratio,
        const coord_t decelerate_length,
        const bool is_scarf_closure,
        const bool compute_distance_to_bridge_start,
        const AddExtrusionSegmentFunction& func_add_segment);

    /*!
     * \brief Add a wall to the gcode with optimized order, possibly adding a scarf seam / speed gradient according to settings
     * \tparam PathType The type of path to be processed, either ExtrusionLine or some subclass of Polyline
     * \param wall The full wall to be added
     * \param start_idx The index of the point where to start printing the wall
     * \param settings The settings which should apply to this wall added to the layer plan
     * \param default_config The config with which to print the wall lines that are not spanning a bridge or are exposed to air
     * \param flow_ratio The ratio with which to multiply the extrusion amount
     * \param always_retract Whether to force a retraction when moving to the start of the polygon (used for outer walls)
     * \param is_closed Indicates whether the path is closed (or open)
     * \param is_reversed Indicates if the path is to be processed backwards
     * \param is_candidate_small_feature Indicates whether the path should be tested for being treated as a smell feature
     * \param scarf_seam Indicates whether we may use a scarf seam for the path
     * \param smooth_speed Indicates whether we may use a speed gradient for the path
     * \param func_add_segment The function to be called to actually add an extrusion segment with the given parameters
     * \return The index of the last traversed point, and the final position with the scarf seam
     */
    template<class PathType>
    std::tuple<size_t, Point2LL> addWallWithScarfSeam(
        const PathAdapter<PathType>& wall,
        size_t start_idx,
        const Settings& settings,
        const GCodePathConfig& default_config,
        const double flow_ratio,
        bool always_retract,
        const bool is_closed,
        const bool is_reversed,
        const bool is_candidate_small_feature,
        const bool scarf_seam,
        const bool smooth_speed,
        const AddExtrusionSegmentFunction& func_add_segment);

    /*!
     * \brief Add a wipe travel after the given path has been extruded
     * \tparam PathType The type of path to be processed, either ExtrusionLine or some subclass of Polyline
     * \param path The path that has just been extruded
     * \param wipe_distance The length of the wipe move to be added
     * \param backwards Indicates if the path has been processed backwards
     * \param start_index The index of the point where o start printing the path
     * \param last_path_position The actual last position of the extruder, which may be slightly forwards on the last printed segment
     */
    template<class PathType>
    void addWipeTravel(const PathAdapter<PathType>& path, const coord_t wipe_distance, const bool backwards, const size_t start_index, const Point2LL& last_path_position);

    /*!
     * Pre-calculates the coasting to be applied on the paths
     *
     * \param extruder_settings The current extruder settings
     * \param paths The current set of paths to be written to GCode
     * \param current_position The last position set in the gcode writer
     * \return The list of coasting settings to be applied on the paths. It will always have the same size as paths.
     */
    std::vector<PathCoasting> calculatePathsCoasting(const Settings& extruder_settings, const std::vector<GCodePath>& paths, const Point3LL& current_position) const;

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
     * \param insertTempOnTime A function that inserts temperature changes at a given time.
     * \param path_coasting The actual coasting to be applied to the path.
     * \return Whether any GCode has been written for the path.
     */
    bool writePathWithCoasting(
        GCodeExport& gcode,
        const size_t extruder_plan_idx,
        const size_t path_idx,
        const std::function<void(const double, const int64_t)> insertTempOnTime,
        const PathCoasting& path_coasting);

    /*!
     * \brief Helper function to calculate the distance from the start of the current wall line to the first bridge segment
     * \param wall The currently processed wall
     * \param current_index The index of the currently processed point
     * \param min_bridge_line_len The minimum line width to allow an extrusion move to be processed as a bridge move
     * \return The distance from the start of the current wall line to the first bridge segment
     */
    coord_t computeDistanceToBridgeStart(const ExtrusionLine& wall, const size_t current_index, const coord_t min_bridge_line_len) const;
};

} // namespace cura

#endif // LAYER_PLAN_H
