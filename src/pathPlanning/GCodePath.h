//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATH_PLANNING_G_CODE_PATH_H
#define PATH_PLANNING_G_CODE_PATH_H

#include "../SpaceFillType.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

#include "TimeMaterialEstimates.h"

namespace cura 
{

class GCodePathConfig;

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
    const GCodePathConfig* config; //!< The configuration settings of the path.
    std::string mesh_id; //!< Which mesh this path belongs to, if any. If it's not part of any mesh, the mesh ID should be 0.
    SpaceFillType space_fill_type; //!< The type of space filling of which this path is a part
    Ratio flow; //!< A type-independent flow configuration
    Ratio speed_factor; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
    Ratio speed_back_pressure_factor; // <! The factor the (non-travel) speed should be multiplied with as a consequence of back pressure compensation.
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path. 
    bool unretract_before_last_travel_move; //!< Whether the last move of the path should be preceded by an unretraction. Used to unretract in the last travel move before an outer wall
    bool perform_z_hop; //!< Whether to perform a z_hop in this path, which is assumed to be a travel path.
    bool perform_prime; //!< Whether this path is preceded by a prime (blob)
    bool skip_agressive_merge_hint; //!< Wheter this path needs to skip merging if any travel paths are in between the extrusions.
    std::vector<Point> points; //!< The points constituting this path.
    bool done; //!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.

    bool spiralize; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and end in one layer higher.

    double fan_speed; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise

    TimeMaterialEstimates estimates; //!< Naive time and material estimates

    /*!
     * \brief Creates a new g-code path.
     *
     * \param config The line configuration to use when printing this path.
     * \param mesh_id The mesh that this path is part of.
     * \param space_fill_type The type of space filling of which this path is a
     * part.
     * \param flow The flow rate to print this path with.
     * \param spiralize Gradually increment the z-coordinate while traversing
     * \param speed_factor The factor that the travel speed will be multiplied with
     * this path.
     */
    GCodePath(const GCodePathConfig& config, std::string mesh_id, const SpaceFillType space_fill_type, const Ratio flow, const bool spiralize, const Ratio speed_factor = 1.0);

    /*!
     * Whether this config is the config of a travel path.
     * 
     * \return Whether this config is the config of a travel path.
     */
    bool isTravelPath() const;

    /*!
     * Get the material flow in mm^3 per mm traversed.
     * 
     * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
     * 
     * \return The flow
     */
    double getExtrusionMM3perMM() const;

    /*!
     * Get the actual line width (modulated by the flow)
     * \return the actual line width as shown in layer view
     */
    coord_t getLineWidthForLayerView() const;

    /*!
     * Set fan_speed
     *
     * \param fan_speed the fan speed to use for this path
     */
     void setFanSpeed(double fan_speed);

    /*!
     * Get the fan speed for this path
     * \return the value of fan_speed if it is in the range 0-100, otherwise the value from the config
     */
    double getFanSpeed() const;
};

}//namespace cura

#endif//PATH_PLANNING_G_CODE_PATH_H
