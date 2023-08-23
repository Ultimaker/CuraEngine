// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef EXTRUDERPLAN_H
#define EXTRUDERPLAN_H

#include "FanSpeedLayerTime.h"
#include "RetractionConfig.h"
#include "gcodeExport.h"
#include "pathPlanning/GCodePath.h"
#include "pathPlanning/NozzleTempInsert.h"
#include "pathPlanning/TimeMaterialEstimates.h"
#include "settings/types/LayerIndex.h"
#include "settings/types/Ratio.h"
#include "utils/IntPoint.h"

#ifdef BUILD_TESTS
#include <gtest/gtest_prod.h> //Friend tests, so that they can inspect the privates.
#endif

#include <limits>
#include <list>
#include <optional>
#include <vector>

namespace cura
{
class LayerPlanBuffer;
class LayerPlan;
/*!
 * An extruder plan contains all planned paths (GCodePath) pertaining to a single extruder train.
 *
 * It allows for temperature command inserts which can be inserted in between paths.
 */
class ExtruderPlan
{
    friend class LayerPlanBuffer;
    friend class LayerPlan;
#ifdef BUILD_TESTS
    friend class ExtruderPlanPathsParameterizedTest;
    FRIEND_TEST(ExtruderPlanPathsParameterizedTest, BackPressureCompensationZeroIsUncompensated);
    FRIEND_TEST(ExtruderPlanPathsParameterizedTest, BackPressureCompensationFull);
    FRIEND_TEST(ExtruderPlanPathsParameterizedTest, BackPressureCompensationHalf);
    FRIEND_TEST(ExtruderPlanTest, BackPressureCompensationEmptyPlan);
#endif
public:
    size_t extruder_nr{ 0 }; //!< The extruder used for this paths in the current plan.

    ExtruderPlan() noexcept = default;

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
    ExtruderPlan(
        const size_t extruder,
        const LayerIndex layer_nr,
        const bool is_initial_layer,
        const bool is_raft_layer,
        const coord_t layer_thickness,
        const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings,
        const RetractionConfig& retraction_config);


    void insertCommand(NozzleTempInsert&& insert);

    /*!
     * Insert the inserts into gcode which should be inserted before \p path_idx
     *
     * \param path_idx The index into ExtruderPlan::paths which is currently being consider for temperature command insertion
     * \param gcode The gcode exporter to which to write the temperature command.
     * \param cumulative_path_time The time spend on this path up to this point.
     */
    void handleInserts(const size_t path_idx, GCodeExport& gcode, const double cumulative_path_time = std::numeric_limits<double>::infinity());

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
     * Applying fan speed changes for minimal layer times.
     *
     * \param starting_position The position the head was before starting this extruder plan
     * \param minTime Maximum minimum layer time for all extruders in this layer
     * \param time_other_extr_plans The time spent on the other extruder plans in this layer
     */
    void processFanSpeedForMinimalLayerTime(Point starting_position, Duration maximum_cool_min_layer_time, double time_other_extr_plans);

    /*!
     * Applying fan speed changes for the first layers.
     */
    void processFanSpeedForFirstLayers();

    /*!
     * Get the fan speed computed for this extruder plan
     *
     * \warning assumes ExtruderPlan::processFanSpeedForMinimalLayerTime has already been called
     *
     * \return The fan speed computed in processFanSpeedForMinimalLayerTime
     */
    double getFanSpeed();

    /*!
     * Apply back-pressure compensation to this path.
     * Since the total (filament) pressure in a feeder-system is not only dependent on the pressure that exists between the nozzle and the
     * feed-mechanism (which should be near-constant on a bowden style setup), but _also_ between the nozzle and the last-printed layer.
     * This last type is called 'back-pressure'. In this function, properties of the path-outflow are adjusted so that the back-pressure is
     * compensated for. This is conjectured to be especially important if the printer has a Bowden-tube style setup.
     *
     * \param The amount of back-pressure compensation as a ratio. 'Applying' a value of 0 is a no-op.
     */
    void applyBackPressureCompensation(const Ratio back_pressure_compensation);

private:
    LayerIndex layer_nr{ 0 }; //!< The layer number at which we are currently printing.
    bool is_initial_layer{ false }; //!< Whether this extruder plan is printed on the very first layer (which might be raft)
    bool is_raft_layer{ false }; //!< Whether this is a layer which is part of the raft

    coord_t layer_thickness{ 200 }; //!< The thickness of this layer in Z-direction

    FanSpeedLayerTimeSettings fan_speed_layer_time_settings{}; //!< The fan speed and layer time settings used to limit this extruder plan

    RetractionConfig retraction_config{}; //!< The retraction settings for the extruder of this plan


    std::vector<GCodePath> paths; //!< The paths planned for this extruder
    std::list<NozzleTempInsert> inserts; //!< The nozzle temperature command inserts, to be inserted in between segments
    double heated_pre_travel_time{ 0.0 }; //!< The time at the start of this ExtruderPlan during which the head travels and has a temperature of initial_print_temperature

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
    double required_start_temperature{ -1.0 };
    std::optional<double> extrusion_temperature{ std::nullopt }; //!< The normal temperature for printing this extruder plan. That start and end of this extruder plan may deviate
                                                                 //!< because of the initial and final print temp (none if extruder plan has no extrusion moves)
    std::optional<std::list<NozzleTempInsert>::iterator> extrusion_temperature_command{
        std::nullopt
    }; //!< The command to heat from the printing temperature of this extruder plan to the printing
       //!< temperature of the next extruder plan (if it has the same extruder).
    std::optional<double> prev_extruder_standby_temp{
        std::nullopt
    }; //!< The temperature to which to set the previous extruder. Not used if the previous extruder plan was the same extruder.

    TimeMaterialEstimates estimates{}; //!< Accumulated time and material estimates for all planned paths within this extruder plan.
    double slowest_path_speed{ 0.0 };

    double extraTime{ 0.0 }; //!< Extra waiting time at the and of this extruder plan, so that the filament can cool

    double fan_speed{ 0.0 }; //!< The fan speed to be used during this extruder plan

    double temperatureFactor{ 0.0 }; //!< Temperature reduction factor for small layers

    /*!
     * Set the fan speed to be used while printing this extruder plan
     *
     * \param fan_speed The speed for the fan
     */
    void setFanSpeed(double fan_speed);

    /*!
     * Force the minimal layer time to hold by slowing down and lifting the head if required.
     *
     * \param maximum_cool_min_layer_time Maximum minimum layer time for all extruders in this layer
     * \param time_other_extr_plans Time spend on other extruders in this layer
     */
    void forceMinimalLayerTime(double maximum_cool_min_layer_time, double time_other_extr_plans);

    /*!
     * @return The time needed for (un)retract the path
     */
    double getRetractTime(const GCodePath& path);

    /*!
     * @return distance between p0 and p1 as well as the time spend on the segment
     */
    std::pair<double, double> getPointToPointTime(const Point& p0, const Point& p1, const GCodePath& path);

    /*!
     * Compute naive time estimates (without accounting for slow down at corners etc.) and naive material estimates.
     * and store them in each ExtruderPlan and each GCodePath.
     *
     * \param starting_position The position the head was in before starting this layer
     * \return the total estimates of this layer
     */
    TimeMaterialEstimates computeNaiveTimeEstimates(Point starting_position);
};

} // namespace cura

#endif // EXTRUDERPLAN_H
