/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef G_CODE_PATH_CONFIG_H
#define G_CODE_PATH_CONFIG_H

#include "RetractionConfig.h"
#include "PrintFeature.h"

namespace cura 
{

/*!
 * The GCodePathConfig is the configuration for moves/extrusion actions. This defines at which width the line is printed and at which speed.
 */
class GCodePathConfig
{
    friend class LayerPlanTest;
public:
    /*!
     * A simple wrapper class for all derivatives of position which are used when printing a line
     */
    struct SpeedDerivatives
    {
        double speed; //!< movement speed (mm/s)
        double acceleration; //!< acceleration of head movement (mm/s^2)
        double jerk; //!< jerk of the head movement (around stand still) as instantaneous speed change (mm/s)
    };
    const PrintFeatureType type; //!< name of the feature type
private:
    SpeedDerivatives speed_derivatives; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
    const int line_width; //!< width of the line extruded
    const int layer_thickness; //!< current layer height in micron
    const double flow; //!< extrusion flow modifier in %
    const double extrusion_mm3_per_mm;//!< current mm^3 filament moved per mm line traversed
public:
    GCodePathConfig(PrintFeatureType type, int line_width, int layer_height, double flow, SpeedDerivatives speed_derivatives); // , SpeedDerivatives slowdown_speed_derivatives, int layer_nr, int max_speed_layer_nr);

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
    void smoothSpeed(SpeedDerivatives first_layer_config, int layer_nr, int max_speed_layer);

    /*!
     * Can only be called after the layer height has been set (which is done while writing the gcode!)
     */
    double getExtrusionMM3perMM() const;

    /*!
     * Get the movement speed in mm/s
     */
    double getSpeed() const;

    /*!
     * Get the current acceleration of this config
     */
    double getAcceleration() const;

    /*!
     * Get the current jerk of this config
     */
    double getJerk() const;

    int getLineWidth() const;

    bool isTravelPath() const;

    double getFlowPercentage() const;

private:
    double calculateExtrusion() const;
};


}//namespace cura

#endif // G_CODE_PATH_CONFIG_H
