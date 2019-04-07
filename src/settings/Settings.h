//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_SETTINGS_H
#define SETTINGS_SETTINGS_H

#ifndef VERSION
#define VERSION "DEV"
#endif

#define MAX_EXTRUDERS 16

//Maximum number of infill layers that can be combined into a single infill extrusion area.
#define MAX_INFILL_COMBINE 8

#include <vector>
#include <map>
#include <unordered_map>
#include <sstream>

namespace cura
{

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
    /*
     * \brief Properly initialises the Settings instance.
     */
    Settings();

    /*!
     * \brief Adds a new setting.
     * \param key The name by which the setting is identified.
     * \param value The value of the setting. The value is always added and
     * stored in serialised form as a string.
     */
    void add(const std::string& key, const std::string value);

    /*!
     * \brief Get the value of a setting.
     *
     * This value is then evaluated using the following technique:
     *  1. If this container contains a value for the setting, it uses that
     *     value directly.
     *  2. Otherwise it checks if the setting is limited to an extruder, and if
     *     so, takes the setting value from that extruder. It applies the
     *     limiting only once at most.
     *  3. Otherwise it asks its parent settings container for the setting value
     *     and returns that. The parent then goes through the same process. The
     *     root of this inheritance structure should always have a value for all
     *     settings.
     *  4. If a setting is not known at all, an error is returned and the
     *     application is closed with an error value of 2.
     * \param key The key of the setting to get.
     * \return The setting's value, cast to the desired type.
     */
    template<typename A> A get(const std::string& key) const;

    /*!
     * \brief Get a string containing all settings in this container.
     *
     * The string is formatted in the same way as the command line arguments
     * when slicing using CuraEngine from the command line. In theory you could
     * put the output of this command in a call to CuraEngine.
     * \return A string containing all settings and their values.
     */
    const std::string getAllSettingsString() const;

    /*!
     * \brief Indicate whether this settings instance has an entry for the
     * specified setting.
     *
     * If this returns ``false``, that means that the setting would be obtained
     * via some inheritance.
     * \param key The setting to check.
     * \return Whether that setting is contained in this particular Settings
     * instance (``true``) or would be obtained via inheritance (``false``).
     */
    bool has(const std::string& key) const;

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
    std::unordered_map<std::string, std::string> settings;

    /*!
     * \brief Get the value of a setting, but without looking at the limiting to
     * extruder.
     *
     * This is the same as the normal ``get`` function, but skipping step 2 and
     * only for strings.
     * \param key The key of the setting to get.
     * \return The setting's value.
     */
    std::string getWithoutLimiting(const std::string& key) const;
};

} //namespace cura

#endif //SETTINGS_SETTINGS_H

