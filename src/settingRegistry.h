#ifndef SETTING_REGISTRY_H
#define SETTING_REGISTRY_H

#include <vector>
#include <list>
#include <unordered_map>
#include <string>
#include <iostream> // debug out

#include "utils/NoCopy.h"
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

/*!
 * Single setting data.
 * Filled from the fdmprinter.json file. Can contain child settings, and is registered in the
 * setting registry with it's key.
 */
class SettingConfig : public SettingContainer
{
private:
    std::string type;
    std::string default_value;
    std::string unit;
public:
    SettingConfig(std::string key, std::string label);
    
    std::string getKey() const
    {
        return key;
    }
    
    void setType(std::string type)
    {
        this->type = type;
    }
    
    std::string getType() const
    {
        return type;
    }

    void setDefault(std::string default_value)
    {
        this->default_value = default_value;
    }
    
    std::string getDefaultValue() const
    {
        return default_value;
    }

    void setUnit(std::string unit)
    {
        this->unit = unit;
    }
    
    std::string getUnit() const
    {
        return unit;
    }
    
    void debugOutputAllSettings() const
    {
        std::cerr << key <<  "(" << default_value << ")" << std::endl;
        for (const SettingConfig& child : children)
        {
            child.debugOutputAllSettings();
        }
    }
};

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
    
    std::unordered_map<std::string, SettingConfig*> settings; //!< Mapping from setting keys to their configurations

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
     * 
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
#endif//SETTING_REGISTRY_H
