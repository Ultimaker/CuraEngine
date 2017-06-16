#ifndef COASTINGCONFIG_H
#define COASTINGCONFIG_H

namespace cura {

/*!
 * Coasting configuration used during printing.
 * Can differ per extruder.
 *
 * Might be used in the future to have different coasting per feature, e.g. outer wall only.
 */
struct CoastingConfig
{
    bool coasting_enable; //!< Whether coasting is enabled on the extruder to which this config is attached
    double coasting_volume; //!< The volume leeked when printing without feeding
    double coasting_speed; //!< A modifier (0-1) on the last used travel speed to move slower during coasting
    double coasting_min_volume;  //!< The minimal volume printed to build up enough pressure to leek the coasting_volume
};


} // cura

#endif // COASTINGCONFIG_H
