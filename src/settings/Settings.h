//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_SETTINGS_H
#define SETTINGS_SETTINGS_H

#include <vector>
#include <map>
#include <unordered_map>
#include <sstream>

#include "Setting.h" //The individual setting.
#include "SettingsBaseVirtual.h"
#include "../utils/string.h"

namespace cura
{

class ExtruderTrain; //Forward declaration to save compilation time.

/*!
 * \brief Container for a set of settings.
 *
 * You can ask this container for the value of a certain setting that should be
 * used in the context where this settings container is located.
 *
 * Before the settings can be returned, the settings have to be added first
 * using the add() function.
 */
class Settings
{
public:
    /*!
     * \brief Adds a new setting.
     * \param key The name by which the setting is identified.
     * \param value The value of the setting. This value can never change over
     * the course of the lifetime of this ``Settings`` container (which is
     * normally over the course of a slice. The value is always added and stored
     * in serialised form as a string.
     * \param limit_to_extruder Ask the extruder in question first for the value
     * of a setting. If this is not set, the setting should not be limited to an
     * extruder.
     */
    void add(const std::string& key, const std::string value, ExtruderTrain* limit_to_extruder = nullptr);

    /*!
     * \brief Get the value of a setting.
     *
     * This value is then evaluated using the following technique:
     *  1a. If this container contains a value for the setting, it uses that
     *      value directly.
     *  1b. Otherwise it asks its parent settings container for the setting
     *      value and returns that. The parent then goes through the same
     *      process. The root of this inheritance structure should always have a
     *      value for all settings.
     *   2. If the setting obtained in step 1 has a valid limit_to_extruder
     *      value, then the setting is obtained from that extruder. Then the
     *      value of that setting is returned. If there was no valid
     *      limit_to_extruder, then the value of the originally obtained setting
     *      is returned.
     *   3. If a setting is not known at all, an error is returned and the
     *      application is closed with an error value of 2.
     */
    template<typename A> A get(const std::string& key) const;

    /*
     * \brief Get a string containing all settings in this container.
     *
     * The string is formatted in the same way as the command line arguments
     * when slicing using CuraEngine from the command line. In theory you could
     * put the output of this command in a call to CuraEngine.
     * \return A string containing all settings and their values.
     */
    const std::string getAllSettingsString() const;

    /*
     * Change the extruder that this setting needs to be obtained from.
     */
    void setLimitToExtruder(const std::string& key, ExtruderTrain* limit_to_extruder);

    /*
     * Change the parent settings object.
     *
     * If this set of settings has no value for a setting, the parent is asked.
     */
    void setParent(Settings* new_parent);

private:
    /*!
     * Optionally, a parent setting container to ask for the value of a setting
     * if this container has no value for it.
     */
    Settings* parent;

    /*!
     * \brief A dictionary to map the setting keys to the actual setting values.
     */
    std::unordered_map<std::string, Setting> settings;
};



////////////////////////////OLD IMPLEMENTATION BELOW////////////////////////////

#ifndef VERSION
#define VERSION "DEV"
#endif

/*!
 * Converts a gcode flavor type to string so that it can be included in the gcode.
 */
std::string toString(EGCodeFlavor flavor);

#define MAX_EXTRUDERS 16

//Maximum number of infill layers that can be combined into a single infill extrusion area.
#define MAX_INFILL_COMBINE 8

}//namespace cura
#endif//SETTINGS_SETTINGS_H

