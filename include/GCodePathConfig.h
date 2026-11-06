// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "PrintFeatureType.h"
#include "routing/SpeedDerivatives.h"
#include "types/Acceleration.h"
#include "types/Coord_t.h"
#include "types/Ratio.h"
#include "types/Temperature.h"
#include "types/Velocity.h"

namespace curaengine
{

/*!
 * The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
 */
struct GCodePathConfig
{
    static constexpr double fan_speed_default = -1.0;

    types::coord_t z_offset{}; //<! vertical offset from 'full' layer height, to be applied to the whole path
    PrintFeatureType type{}; //!< name of the feature type
    types::coord_t line_width{}; //!< width of the line extruded
    types::coord_t layer_thickness{}; //!< current layer height in micron
    types::Ratio flow{}; //!< extrusion flow modifier.
    routing::SpeedDerivatives speed_derivatives{}; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
    bool is_bridge_path{ false }; //!< whether current config is used when bridging
    double fan_speed{ fan_speed_default }; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
    double extrusion_mm3_per_mm{ calculateExtrusion() }; //!< current mm^3 filament moved per mm line traversed
    types::Temperature temperature_delta;

    [[nodiscard]] constexpr bool operator==(const GCodePathConfig& other) const noexcept = default;
    [[nodiscard]] constexpr auto operator<=>(const GCodePathConfig& other) const = default;

    /*!
     * Can only be called after the layer height has been set (which is done while writing the gcode!)
     */
    [[nodiscard]] double getExtrusionMM3perMM() const noexcept;

    /*!
     * Get the movement speed in mm/s
     */
    [[nodiscard]] types::Velocity getSpeed() const noexcept;

    /*!
     * Get the current acceleration of this config
     */
    [[nodiscard]] types::Acceleration getAcceleration() const noexcept;

    /*!
     * Get the current jerk of this config
     */
    [[nodiscard]] types::Velocity getJerk() const noexcept;

    [[nodiscard]] types::coord_t getLineWidth() const noexcept;

    [[nodiscard]] bool isTravelPath() const noexcept;

    [[nodiscard]] bool isBridgePath() const noexcept;

    [[nodiscard]] double getFanSpeed() const noexcept;

    [[nodiscard]] types::Ratio getFlowRatio() const noexcept;

    [[nodiscard]] types::coord_t getLayerThickness() const noexcept;

    [[nodiscard]] PrintFeatureType getPrintFeatureType() const noexcept;

    [[nodiscard]] const types::Temperature& getTemperatureDelta() const noexcept;

private:
    [[nodiscard]] double calculateExtrusion() const noexcept;
};


} // namespace curaengine
