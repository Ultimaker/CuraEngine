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
private:
    double speed_iconic; //!< movement speed (mm/s) specific to this print feature
    double speed; //!< current movement speed (mm/s) (modified by layer_nr etc.)
    int line_width; //!< width of the line extruded
    double flow; //!< extrusion flow modifier in %
    int layer_thickness; //!< layer height in micron
    double extrusion_mm3_per_mm;//!< mm^3 filament moved per mm line traversed
public:
    PrintFeatureType type; //!< name of the feature type
    bool spiralize; //!< Whether the Z should increment slowly over the whole layer when printing this feature.
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
    void init(double speed, int line_width, double flow);

    /*!
     * Set the layer height and (re)compute the extrusion_per_mm
     */
    void setLayerHeight(int layer_height);

    /*!
     * Set the speed to somewhere between the @p min_speed and the speed_iconic.
     * 
     * This functions should not be called with @p layer_nr > @p max_speed_layer !
     * 
     * \param min_speed The speed at layer zero
     * \param layer_nr The layer number 
     * \param max_speed_layer The layer number for which the speed_iconic should be used.
     */
    void smoothSpeed(double min_speed, int layer_nr, double max_speed_layer);

    /*!
     * Set the speed to the iconic speed, i.e. the normal speed of the feature type for which this is a config.
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

    int getLineWidth();

    bool isTravelPath();

    double getFlowPercentage();

private:
    void calculateExtrusion();
};


}//namespace cura

#endif // G_CODE_PATH_CONFIG_H
