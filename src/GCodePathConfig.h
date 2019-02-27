//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef G_CODE_PATH_CONFIG_H
#define G_CODE_PATH_CONFIG_H

#include "PrintFeature.h"
#include "settings/types/Ratio.h" //For flow rate.
#include "settings/types/Velocity.h"
#include "utils/Coord_t.h"

namespace cura 
{

struct LayerIndex;

/*!
 * The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
 */
class GCodePathConfig
{
public:
    /*!
     * A simple wrapper class for all derivatives of position which are used when printing a line
     */
    struct SpeedDerivatives
    {
        Velocity speed; //!< movement speed (mm/s)
        Acceleration acceleration; //!< acceleration of head movement (mm/s^2)
        Velocity jerk; //!< jerk of the head movement (around stand still) as instantaneous speed change (mm/s)
    };
    const PrintFeatureType type; //!< name of the feature type
    static constexpr double FAN_SPEED_DEFAULT = -1;
private:
    SpeedDerivatives speed_derivatives; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
    const coord_t line_width; //!< width of the line extruded
    const coord_t layer_thickness; //!< current layer height in micron
    const Ratio flow; //!< extrusion flow modifier.
    const double extrusion_mm3_per_mm;//!< current mm^3 filament moved per mm line traversed
    const bool is_bridge_path; //!< whether current config is used when bridging
    const double fan_speed; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
public:
    GCodePathConfig(const PrintFeatureType& type, const coord_t line_width, const coord_t layer_height, const Ratio& flow, const SpeedDerivatives speed_derivatives, const bool is_bridge_path = false, const double fan_speed = FAN_SPEED_DEFAULT);

    /*!
     * copy constructor
     */
    GCodePathConfig(const GCodePathConfig& other);

    /*!
     * Set the speed to somewhere between the speed of @p first_layer_config and the iconic speed.
     * 
     * \warning This functions should not be called with @p layer_nr > @p max_speed_layer !
     * 
     * \warning Calling this function twice will smooth the speed more toward \p first_layer_config
     * 
     * \param first_layer_config The speed settings at layer zero
     * \param layer_nr The layer number 
     * \param max_speed_layer The layer number for which the speed_iconic should be used.
     */
    void smoothSpeed(SpeedDerivatives first_layer_config, const LayerIndex& layer_nr, const LayerIndex& max_speed_layer);

    /*!
     * Can only be called after the layer height has been set (which is done while writing the gcode!)
     */
    double getExtrusionMM3perMM() const;

    /*!
     * Get the movement speed in mm/s
     */
    Velocity getSpeed() const;

    /*!
     * Get the current acceleration of this config
     */
    Acceleration getAcceleration() const;

    /*!
     * Get the current jerk of this config
     */
    Velocity getJerk() const;

    coord_t getLineWidth() const;

    bool isTravelPath() const;

    bool isBridgePath() const;

    double getFanSpeed() const;

    Ratio getFlowRatio() const;

    coord_t getLayerThickness() const;

    const PrintFeatureType& getPrintFeatureType() const;

private:
    double calculateExtrusion() const;
};


}//namespace cura

#endif // G_CODE_PATH_CONFIG_H
