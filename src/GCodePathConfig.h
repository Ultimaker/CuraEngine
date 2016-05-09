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
public:
    /*!
     * The path config settings which may change from layer to layer
     */
    struct BasicConfig
    {
        double speed; //!< movement speed (mm/s)
        double acceleration; //!< acceleration of head movement (mm/s^2)
        double jerk; //!< jerk of the head movement (around stand still) (mm/s^3)
        int line_width; //!< width of the line extruded
        double flow; //!< extrusion flow modifier in %
        BasicConfig(); //!< basic contructor initializing with inaccurate values
        BasicConfig(double speed, double acceleration, double jerk, int line_width, double flow); //!< basic contructor initializing all values
        void set(double speed, double acceleration, double jerk, int line_width, double flow); //!< Set all config values
    };
private:
    BasicConfig iconic_config; //!< The basic path configuration iconic to this print feature type
    BasicConfig current_config; //!< The current path configuration for the current layer
    int layer_thickness; //!< current layer height in micron
    double extrusion_mm3_per_mm;//!< current mm^3 filament moved per mm line traversed
public:
    const PrintFeatureType type; //!< name of the feature type
    bool spiralize; //!< Whether the Z should increment slowly over the whole layer when printing this feature. TODO: the fact that this option is here introduces a bug in combination with the fact that we have a LayerPlanBuffer; This value is set in FffGcodeWriter, meaning it has effect on previous layers!
    RetractionConfig *const retraction_config; //!< The retraction configuration to use when retracting after a part of this feature has been printed.

    /*!
     * Basic constructor.
     */
    GCodePathConfig(RetractionConfig* retraction_config, PrintFeatureType type);
    
    /*!
     * Initialize some of the member variables.
     * 
     * \warning GCodePathConfig::setLayerHeight still has to be called before this object can be used.
     * 
     * \param speed The regular speed with which to print this feature
     * \param line_width The line width for this feature
     * \param flow The flow modifier to apply to the extruded filament when printing this feature
     */
    void init(double speed, double acceleration, double jerk, int line_width, double flow);

    /*!
     * Set the layer height and (re)compute the extrusion_per_mm
     */
    void setLayerHeight(int layer_height);

    /*!
     * Set the speed to somewhere between the speed of @p first_layer_config and the iconic speed.
     * 
     * \warning This functions should not be called with @p layer_nr > @p max_speed_layer !
     * 
     * \param first_layer_config The speed settings at layer zero
     * \param layer_nr The layer number 
     * \param max_speed_layer The layer number for which the speed_iconic should be used.
     */
    void smoothSpeed(BasicConfig first_layer_config, int layer_nr, double max_speed_layer);

    /*!
     * Set the speed config to the iconic speed config, i.e. the normal speed of the feature type for which this is a config.
     * 
     * Does the same for acceleration and jerk.
     */
    void setSpeedIconic();

    /*!
     * Can only be called after the layer height has been set (which is done while writing the gcode!)
     */
    double getExtrusionMM3perMM();

    /*!
     * Get the movement speed in mm/s
     */
    double getSpeed();

    /*!
     * Get the current acceleration of this config
     */
    double getAcceleration();

    /*!
     * Get the current jerk of this config
     */
    double getJerk();

    int getLineWidth();

    bool isTravelPath();

    double getFlowPercentage();

private:
    void calculateExtrusion();
};


}//namespace cura

#endif // G_CODE_PATH_CONFIG_H
