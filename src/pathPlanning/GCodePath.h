//Copyright (C) 2017 Ultimaker
//Released under terms of the AGPLv3 License

#ifndef PATH_PLANNING_G_CODE_PATH_H
#define PATH_PLANNING_G_CODE_PATH_H

#include "../SpaceFillType.h"
#include "../GCodePathConfig.h"

#include "TimeMaterialEstimates.h"

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
class GCodePath
{
public:
    GCodePath();

    const GCodePathConfig* config; //!< The configuration settings of the path.
    SpaceFillType space_fill_type; //!< The type of space filling of which this path is a part
    float flow; //!< A type-independent flow configuration (used for wall overlap compensation)
    double speed_factor; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path.
    bool perform_z_hop; //!< Whether to perform a z_hop in this path, which is assumed to be a travel path.
    bool perform_prime; //!< Whether this path is preceded by a prime (blob)
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.

    bool spiralize; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and end in one layer higher.

    double extrusion_offset; //!< TODO: for flow-rate compensation

    TimeMaterialEstimates estimates; //!< Naive time and material estimates

    /*!
     * \brief Creates a new g-code path.
     *
     * \param config The line configuration to use when printing this path.
     * \param space_fill_type The type of space filling of which this path is a
     * part.
     * \param flow The flow rate to print this path with.
     * \param spiralize Gradually increment the z-coordinate while traversing
     * \param speed_factor The factor that the travel speed will be multiplied with
     * this path.
     */
    GCodePath(const GCodePathConfig& config, SpaceFillType space_fill_type, float flow, bool spiralize, double speed_factor = 1.0, double extrusion_offset = -1);

    /*!
     * Whether this config is the config of a travel path.
     *
     * \return Whether this config is the config of a travel path.
     */
    bool isTravelPath();

    /*!
     * Get the material flow in mm^3 per mm traversed.
     *
     * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
     *
     * \return The flow
     */
    double getExtrusionMM3perMM();

    double getExtrusionOffsetMM();

    /*!
     * Get the actual line width (modulated by the flow)
     * \return the actual line width as shown in layer view
     */
    int getLineWidthForLayerView();
};

}//namespace cura

#endif//PATH_PLANNING_G_CODE_PATH_H
