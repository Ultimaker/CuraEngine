/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_SETTING_REGISTRY_H
#define SETTINGS_SETTING_REGISTRY_H

#include <vector>
#include <list>
#include <unordered_map>
#include <string>
#include <iostream> // debug out

#include "SettingConfig.h"
#include "SettingContainer.h"

#include "../utils/NoCopy.h"
#include "rapidjson/document.h"

namespace cura
{

/*!
 * Setting registry.
 * There is a single global setting registry.
 * This registry contains all known setting keys and (some of) their attributes.
 * The default values are stored and retrieved in case a given setting doesn't get a value from the command line or the frontend.
 */
class SettingRegistry : NoCopy
{
private:
    static SettingRegistry instance;

    SettingRegistry();
    
    std::unordered_map<std::string, SettingConfig*> setting_key_to_config; //!< Mapping from setting keys to their configurations

    SettingContainer setting_definitions; //!< All setting configurations (A flat list)
    
    std::vector<SettingContainer> extruder_trains; //!< The setting overrides per extruder train as defined in the json file

    bool warn_duplicates = false; //!< whether to warn for duplicate setting definitions
public:
    /*!
     * Get the SettingRegistry.
     * 
     * This is a singleton class.
     * 
     * \return The SettingRegistry
     */
    static SettingRegistry* getInstance() { return &instance; }
    
    bool settingExists(std::string key) const;
    SettingConfig* getSettingConfig(std::string key) const;

    /*!
     * Retrieve the setting definitions container for all settings of a given extruder train.
     * 
     * \param extruder_nr The extruder train to retrieve
     * \return The extruder train or nullptr if \p extruder_nr refers to an extruder train which is undefined in the json.
     */
    SettingContainer* getExtruderTrain(unsigned int extruder_nr);
protected:
    /*!
     * Whether this json settings object is a definition of a CuraEngine setting,
     * or only a shorthand setting to control other settings.
     * Only settings used by the engine will be recordedd in the registry.
     * 
     * \param setting The setting to check whether CuraEngine uses it.
     * \return Whether CuraEngine uses the setting.
     */
    bool settingIsUsedByEngine(const rapidjson::Value& setting);
private:
    /*!
     * Return the first category with the given key as name, or a new one.
     * 
     * \param cat_name the key as it is in the JSON file
     * \param category the JSON Value associated with the key or nullptr if no Value is available to get the label from
     * \return The first category in the list having the \p key (or a new one)
     */
    SettingContainer& getOrCreateCategory(std::string cat_name, const rapidjson::Value* category = nullptr);
public:
    bool settingsLoaded() const;
    /*!
     * Load settings from a json file and all the parents it inherits from.
     * 
     * Uses recursion to load the parent json file.
     * 
     * \param filename The filename of the json file to parse
     * \return an error code or zero of succeeded
     */
    int loadJSONsettings(std::string filename);
    
    void debugOutputAllSettings() const
    {
        setting_definitions.debugOutputAllSettings();
    }
    
private:
    
    /*!
     * \param type type to convert to string
     * \return human readable version of json type
     */
    static std::string toString(rapidjson::Type type);
public:
    /*!
     * Load a json document.
     * 
     * \param filename The filename of the json file to parse
     * \param json_document (output) the document to be loaded
     * \return an error code or zero of succeeded
     */
    static int loadJSON(std::string filename, rapidjson::Document& json_document);
private:
    /*!
     * Load settings from a single json file.
     * 
     * \param filename The filename of the json file to parse
     * \param warn_duplicates whether to warn for duplicate definitions
     * \return an error code or zero of succeeded
     */
    int loadJSONsettingsFromDoc(rapidjson::Document& json_document, bool warn_duplicates);
    
    /*!
     * Get the string from a json value (generally the default value field of a setting)
     * \param dflt The value to convert to string
     * \param setting_name The name of the setting (in case we need to display an error message)
     * \return The string
     */
    static std::string toString(const rapidjson::Value& dflt, std::string setting_name = "?");

    /*!
     * Create a new SettingConfig and add it to the registry.
     * 
     * \param name The internal key of the setting
     * \param label The human readable name for the frontend
     * \return The config created
     */
    SettingConfig& addSetting(std::string name, std::string label);

    void _loadSettingValues(SettingConfig* config, const rapidjson::Value::ConstMemberIterator& json_object_it);

    /*!
     * Handle a json object which contains a list of settings.
     * 
     * \param settings_list The object containing one or more setting definitions
     * \param path The path of (internal) setting names traversed to get to this object
     */
    void handleChildren(const rapidjson::Value& settings_list, std::list<std::string>& path);
    
    /*!
     * Handle a json object for a setting.
     * 
     * \param json_setting_it Iterator for the setting which contains the key (setting name) and attributes info
     * \param path The path of (internal) setting names traversed to get to this object
     */
    void handleSetting(const rapidjson::Value::ConstMemberIterator& json_setting_it, std::list<std::string>& path);
};

}//namespace cura
#endif//SETTINGS_SETTING_REGISTRY_H
