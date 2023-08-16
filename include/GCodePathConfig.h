// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef G_CODE_PATH_CONFIG_H
#define G_CODE_PATH_CONFIG_H

#include "PrintFeature.h"
#include "pathPlanning/SpeedDerivatives.h"
#include "settings/types/LayerIndex.h"
#include "settings/types/Ratio.h"
#include "settings/types/Velocity.h"
#include "utils/Coord_t.h"

namespace cura
{

/*!
 * The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
 */
struct GCodePathConfig
{
    PrintFeatureType type {}; //!< name of the feature type
    coord_t line_width {}; //!< width of the line extruded
    coord_t layer_thickness {}; //!< current layer height in micron
    Ratio flow {}; //!< extrusion flow modifier.
    SpeedDerivatives speed_derivatives {}; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
    bool is_bridge_path { false }; //!< whether current config is used when bridging
    double fan_speed { FAN_SPEED_DEFAULT }; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
    double extrusion_mm3_per_mm { calculateExtrusion() }; //!< current mm^3 filament moved per mm line traversed
    static constexpr double FAN_SPEED_DEFAULT = -1;

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


} // namespace cura

#endif // G_CODE_PATH_CONFIG_H
