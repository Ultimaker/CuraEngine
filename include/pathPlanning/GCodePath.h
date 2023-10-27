// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATH_PLANNING_G_CODE_PATH_H
#define PATH_PLANNING_G_CODE_PATH_H

#include <memory>
#include <vector>

#include "GCodePathConfig.h"
#include "SpaceFillType.h"
#include "TimeMaterialEstimates.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "utils/IntPoint.h"

namespace cura
{

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
struct GCodePath
{
    coord_t z_offset{}; //<! vertical offset from 'full' layer height
    GCodePathConfig config{}; //!< The configuration settings of the path.
    std::shared_ptr<const SliceMeshStorage> mesh; //!< Which mesh this path belongs to, if any. If it's not part of any mesh, the mesh should be nullptr;
    SpaceFillType space_fill_type{}; //!< The type of space filling of which this path is a part
    Ratio flow{}; //!< A type-independent flow configuration
    Ratio width_factor{}; //!< Adjustment to the line width. Similar to flow, but causes the speed_back_pressure_factor to be adjusted.
    bool spiralize{}; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and
                      //!< end in one layer higher.
    Ratio speed_factor{ 1.0 }; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
    Ratio speed_back_pressure_factor{ 1.0 }; // <! The factor the (non-travel) speed should be multiplied with as a consequence of back pressure compensation.
    bool retract{ false }; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path.
    bool unretract_before_last_travel_move{ false }; //!< Whether the last move of the path should be preceded by an unretraction. Used to unretract in the last travel move before
                                                     //!< an outer wall
    bool perform_z_hop{ false }; //!< Whether to perform a z_hop in this path, which is assumed to be a travel path.
    bool perform_prime{ false }; //!< Whether this path is preceded by a prime (blob)
    bool skip_agressive_merge_hint{ false }; //!< Wheter this path needs to skip merging if any travel paths are in between the extrusions.
    std::vector<Point> points; //!< The points constituting this path.
    bool done{ false }; //!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.
    double fan_speed{ GCodePathConfig::FAN_SPEED_DEFAULT }; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
    TimeMaterialEstimates estimates{}; //!< Naive time and material estimates

    /*!
     * Whether this config is the config of a travel path.
     *
     * \return Whether this config is the config of a travel path.
     */
    [[nodiscard]] bool isTravelPath() const noexcept;

    /*!
     * Get the material flow in mm^3 per mm traversed.
     *
     * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
     *
     * \return The flow
     */
    [[nodiscard]] double getExtrusionMM3perMM() const noexcept;

    /*!
     * Get the actual line width (modulated by the flow)
     * \return the actual line width as shown in layer view
     */
    [[nodiscard]] coord_t getLineWidthForLayerView() const noexcept;

    /*!
     * Set fan_speed
     *
     * \param fan_speed the fan speed to use for this path
     */
    void setFanSpeed(const double fanspeed) noexcept;

    /*!
     * Get the fan speed for this path
     * \return the value of fan_speed if it is in the range 0-100, otherwise the value from the config
     */
    [[nodiscard]] double getFanSpeed() const noexcept;
};

} // namespace cura

#endif // PATH_PLANNING_G_CODE_PATH_H
