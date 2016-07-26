/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_TO_GV_H
#define SETTINGS_TO_GV_H


#include <stdio.h> // for file output
#include <sstream>
#include <iostream> // debug IO
#include <libgen.h> // dirname
#include <string>
#include <algorithm> // find_if
#include <regex> // regex_search
#include <cassert>
#include <fstream>
#include <set>
#include <unordered_map>

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "../utils/logoutput.h"
#include "SettingRegistry.h"

namespace cura
{

class SettingsToGv
{
    enum class RelationType
    {
        PARENT_CHILD,
        INHERIT_FUNCTION,
        ERROR_FUNCTION,
        WARNING_FUNCTION
    };
    
    FILE* out;
    std::set<std::string> engine_settings;
    
    std::unordered_map<std::string, std::string> setting_to_color;
    bool parent_child_viz, inherit_viz, error_viz, warning_viz, global_only_viz;
public: 
    SettingsToGv(std::string output_filename, std::string engine_settings_filename, bool parent_child_viz, bool inherit_viz, bool error_viz, bool warning_viz, bool global_only_viz)
    : parent_child_viz(parent_child_viz)
    , inherit_viz(inherit_viz)
    , error_viz(error_viz)
    , warning_viz(warning_viz)
    , global_only_viz(global_only_viz)
    {
        out = fopen(output_filename.c_str(), "w");
        fprintf(out, "digraph G {\n");
        
        
        std::ifstream engine_settings_file(engine_settings_filename.c_str());
        std::string line;
        while (std::getline(engine_settings_file, line))
        {
            engine_settings.insert(line);
            //fprintf(out, "%s [color=green];\n", line.c_str());
        }
        engine_settings_file.close();
    }
private:
    void generateEdge(const std::string& parent, const std::string& child, RelationType relation_type)
    {
        if (global_only_viz)
        {
            auto parent_it = setting_to_color.find(parent);
            if (parent_it != setting_to_color.end())
            {
                fprintf(out, "%s [color=%s];\n", parent_it->first.c_str(), parent_it->second.c_str());
            }
            auto child_it = setting_to_color.find(child);
            if (child_it != setting_to_color.end())
            {
                fprintf(out, "%s [color=%s];\n", child_it->first.c_str(), child_it->second.c_str());
            }
        }
        else
        {
            if (engine_settings.find(parent) != engine_settings.end())
            {
                fprintf(out, "%s [color=green];\n", parent.c_str());
            }
            if (engine_settings.find(child) != engine_settings.end())
            {
                fprintf(out, "%s [color=green];\n", child.c_str());
            }
        }

        std::string color;
        switch (relation_type)
        {
            case SettingsToGv::RelationType::INHERIT_FUNCTION:
                if (!inherit_viz)
                {
                    return;
                }
                color = "blue";
                break;
            case SettingsToGv::RelationType::PARENT_CHILD:
                if (!parent_child_viz)
                {
                    return;
                }
                color = "black";
                break;
            case SettingsToGv::RelationType::ERROR_FUNCTION:
                if (!error_viz)
                {
                    return;
                }
                color = "red";
                break;
            case SettingsToGv::RelationType::WARNING_FUNCTION:
                if (!warning_viz)
                {
                    return;
                }
                color = "orange";
                break;
        }
        fprintf(out, "edge [color=%s];\n", color.c_str());
        fprintf(out, "%s -> %s;\n", parent.c_str(), child.c_str());
    }
    
    bool createFunctionEdges(const rapidjson::Value& data, std::string function_key, const std::string& parent, const std::string& name, const RelationType relation_type)
    {
        bool generated_edge = false;
        if (data.HasMember(function_key.c_str()) && data[function_key.c_str()].IsString())
        {
            std::string function = data[function_key.c_str()].GetString();
            
            std::regex setting_name_regex("[a-zA-Z0-9_]+"); // matches mostly with setting names
            std::smatch regex_match;
            while (std::regex_search (function, regex_match, setting_name_regex))
            {
                std::string inherited_setting_string = regex_match[0];
                if (inherited_setting_string == "parent_value")
                {
                    generateEdge(parent, name, RelationType::PARENT_CHILD);
                    generated_edge = true;
                }
                else if ( ! std::regex_match(inherited_setting_string, std::regex("[0-9]+")) && // exclude numbers
                    // result != "parent_value" && 
                    inherited_setting_string != "if" && inherited_setting_string != "else" && inherited_setting_string != "and" 
                    && inherited_setting_string != "or" && inherited_setting_string != "math" && inherited_setting_string != "ceil" 
                    && inherited_setting_string != "int" && inherited_setting_string != "round" && inherited_setting_string != "max" // exclude operators and functions
                    && inherited_setting_string != "log" // exclude functions
                    && inherited_setting_string != "grid" && inherited_setting_string != "triangles" // exclude enum values
                    && inherited_setting_string != "cubic" && inherited_setting_string != "tetrahedral" // exclude enum values
                    && inherited_setting_string != "raft" // exclude enum values
                    && function.c_str()[regex_match.position() + regex_match.length()] != '\'') // exclude enum terms
                {
                    if (inherited_setting_string == parent)
                    {
                        generated_edge = true;
                        generateEdge(inherited_setting_string, name, RelationType::PARENT_CHILD);
                    }
                    else 
                    {
                        generateEdge(inherited_setting_string, name, relation_type);
                    }
                }
                function = regex_match.suffix().str();
            }
        }
        return generated_edge;
    }
    
    void parseSetting(const std::string& parent, rapidjson::Value::ConstMemberIterator json_object_it)
    {
        std::string name = json_object_it->name.GetString();
        
//         std::cerr << "parsed: " << name <<"\n";
        
        bool generated_edge = false;
        
        const rapidjson::Value& data = json_object_it->value;
        
        if (data.HasMember("type") && data["type"].IsString() && data["type"].GetString() != std::string("category"))
        {
            if (global_only_viz)
            {
                std::string color;
                if (!data.HasMember("settable_per_mesh") || data["settable_per_mesh"].GetBool() == true)
                {
                    color = "green";
                }
                else if (data.HasMember("settable_per_mesh") && data["settable_per_mesh"].GetBool() == false)
                {
                    if (!data.HasMember("settable_per_extruder") || data["settable_per_extruder"].GetBool() == true)
                    {
                        color = "yellow";
                    }
                    else if (data.HasMember("settable_per_extruder") && data["settable_per_extruder"].GetBool() == false)
                    {
                        if (!data.HasMember("settable_per_meshgroup") || data["settable_per_meshgroup"].GetBool() == true)
                        {
                            color = "orange";
                        }
                        else if (data.HasMember("settable_per_meshgroup") && data["settable_per_meshgroup"].GetBool() == false)
                        {
                            color = "red";
                        }
                    }
                }
                setting_to_color.emplace(name, color);
//                 fprintf(out, "%s [color=%s];\n", name.c_str(), color.c_str());
            }
            
            
            bool generated_edge_inherit = createFunctionEdges(data, "value", parent, name, RelationType::INHERIT_FUNCTION);
            bool generated_edge_max = createFunctionEdges(data, "maximum_value", parent, name, RelationType::ERROR_FUNCTION);
            bool generated_edge_min = createFunctionEdges(data, "minimum_value", parent, name, RelationType::ERROR_FUNCTION);
            bool generated_edge_max_warn = createFunctionEdges(data, "maximum_value_warning", parent, name, RelationType::WARNING_FUNCTION);
            bool generated_edge_min_warn = createFunctionEdges(data, "minimum_value_warning", parent, name, RelationType::WARNING_FUNCTION);
            if (generated_edge_inherit || generated_edge_max_warn || generated_edge_min_warn || generated_edge_max || generated_edge_min)
            {
                generated_edge = true;
            }
            
            if (!generated_edge && parent != "")
            {
                generateEdge(parent, name, RelationType::PARENT_CHILD);
            }
        }
        else
        {
            name = "";
        }
        
        // recursive part
        if (data.HasMember("children") && data["children"].IsObject())
        {
            const rapidjson::Value& json_object_container = data["children"];
            for (rapidjson::Value::ConstMemberIterator setting_iterator = json_object_container.MemberBegin(); setting_iterator != json_object_container.MemberEnd(); ++setting_iterator)
            {
                parseSetting(name, setting_iterator);
            }
        }
    }
    
    void parseJson(const rapidjson::Document& json_document)
    {
        if (json_document.HasMember("settings"))
        {
            for (rapidjson::Value::ConstMemberIterator setting_iterator = json_document["settings"].MemberBegin(); setting_iterator != json_document["settings"].MemberEnd(); ++setting_iterator)
            {
                parseSetting("", setting_iterator);
            }
        }
    }

    int generateRecursive(std::string filename)
    {
        rapidjson::Document json_document;
        
        int err = SettingRegistry::loadJSON(filename, json_document);
        if (err) { return err; }
        
        if (json_document.HasMember("inherits"))
        {
            std::string filename_copy = std::string(filename.c_str()); // copy the string because dirname(.) changes the input string!!!
            char* filename_cstr = (char*)filename_copy.c_str();
            int err = generate(std::string(dirname(filename_cstr)) + std::string("/") + json_document["inherits"].GetString());
            if (err)
            {
                return err;
            }
        }
        parseJson(json_document);
        return 0;
    }
public:
    int generate(std::string json_filename)
    {
        int err = generateRecursive(json_filename);
        fprintf(out, "}\n");
        fclose(out);
        return err;
    }



};

} // namespace cura

#endif // SETTINGS_TO_GV_H