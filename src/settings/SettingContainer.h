/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_SETTING_CONTAINER_H
#define SETTINGS_SETTING_CONTAINER_H

#include <vector>
#include <list>
#include <unordered_map>
#include <string>
#include <iostream> // debug out

#include "../utils/NoCopy.h"
#include "rapidjson/document.h"

namespace cura
{

// Forward declaration
class SettingConfig;
class SettingRegistry;

/*!
 * Setting container for a settings base of definitions and default values.
 * Filled from the .def.json files. Contains one or more children settings.
 */
class SettingContainer
{
    friend class SettingConfig;
    friend class SettingRegistry;
private:
    std::string key;
    std::string label;
    std::list<SettingConfig> children; // must be a list cause the pointers to individual children are mapped to in SettingRegistry::settings.
    std::list<std::string> path; //!< The path of parents (internal names) to this container
public:
    std::string getKey() const { return key; }
    std::string getLabel() const { return label; }
    SettingContainer(std::string key, std::string label);

    /*!
     * Get the SettingConfig::children.
     *
     * This is used to get the extruder trains; see Settingsbase::setExtruderTrainDefaults
     *
     * \return SettingConfig::children
     */
    const std::list<SettingConfig>& getChildren() const { return children; }

    SettingConfig* addChild(std::string key, std::string label);

    /*!
     * Get the \p idx th child.
     *
     * This is used to get a specific extruder train in Settingsbase::setExtruderTrainDefaults
     *
     * \param idx The index in the list of children
     * \return The \p idx th child
     */
    const SettingConfig* getChild(unsigned int idx) const
    {
        if (idx < children.size())
        {
            auto it = children.begin();
            while (idx > 0) { ++it; idx--; }
            return &*it;
        }
        else
            return nullptr;
    }

private:
    /*!
     * Get the (direct) child with key \p key, or create one with key \p key and label \p label as well.
     *
     * \param key the key
     * \param label the label for creating a new child
     * \return The existing or newly created child setting.
     */
    SettingConfig& getOrCreateChild(std::string key, std::string label);
public:
    void debugOutputAllSettings() const;
};
}//namespace cura
#endif//SETTINGS_SETTING_CONTAINER_H
