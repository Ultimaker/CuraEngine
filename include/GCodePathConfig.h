// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "PrintFeatureType.h"
#include "path_planning/SpeedDerivatives.h"
#include "types/Acceleration.h"
#include "types/Ratio.h"
#include "types/Temperature.h"
#include "types/Velocity.h"
#include "utils/Coord_t.h"

namespace curaengine
{

/*!
 * The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
 */
struct GCodePathConfig
{
    static constexpr double FAN_SPEED_DEFAULT = -1.0;

    ::curaengine::utils::coord_t z_offset{}; //<! vertical offset from 'full' layer height, to be applied to the whole path
    PrintFeatureType type{}; //!< name of the feature type
    ::curaengine::utils::coord_t line_width{}; //!< width of the line extruded
    ::curaengine::utils::coord_t layer_thickness{}; //!< current layer height in micron
    ::curaengine::types::Ratio flow{}; //!< extrusion flow modifier.
    ::curaengine::path_planning::SpeedDerivatives speed_derivatives{}; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
    bool is_bridge_path{ false }; //!< whether current config is used when bridging
    double fan_speed{ FAN_SPEED_DEFAULT }; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
    double extrusion_mm3_per_mm{ calculateExtrusion() }; //!< current mm^3 filament moved per mm line traversed
    ::curaengine::types::Temperature temperature_delta;

    [[nodiscard]] constexpr bool operator==(const GCodePathConfig& other) const noexcept = default;
    [[nodiscard]] constexpr auto operator<=>(const GCodePathConfig& other) const = default;

    /*!
     * Can only be called after the layer height has been set (which is done while writing the gcode!)
     */
    [[nodiscard]] double getExtrusionMM3perMM() const noexcept;

    /*!
     * Get the movement speed in mm/s
     */
    [[nodiscard]] ::curaengine::types::Velocity getSpeed() const noexcept;

    /*!
     * Get the current acceleration of this config
     */
    [[nodiscard]] ::curaengine::types::Acceleration getAcceleration() const noexcept;

    /*!
     * Get the current jerk of this config
     */
    [[nodiscard]] ::curaengine::types::Velocity getJerk() const noexcept;

    [[nodiscard]] ::curaengine::utils::coord_t getLineWidth() const noexcept;

    [[nodiscard]] bool isTravelPath() const noexcept;

    [[nodiscard]] bool isBridgePath() const noexcept;

    [[nodiscard]] double getFanSpeed() const noexcept;

    [[nodiscard]] ::curaengine::types::Ratio getFlowRatio() const noexcept;

    [[nodiscard]] ::curaengine::utils::coord_t getLayerThickness() const noexcept;

    [[nodiscard]] PrintFeatureType getPrintFeatureType() const noexcept;

    [[nodiscard]] const ::curaengine::types::Temperature& getTemperatureDelta() const noexcept;

private:
    [[nodiscard]] double calculateExtrusion() const noexcept;
};


} // namespace curaengine
