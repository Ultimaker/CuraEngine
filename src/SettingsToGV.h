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

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "utils/logoutput.h"
#include "settingRegistry.h"

namespace cura
{

class SettingsToGv
{
    enum class RelationType
    {
        PARENT_CHILD,
        INHERIT_FUNCTION
    };
    
    FILE* out;
    std::set<std::string> engine_settings;
public: 
    SettingsToGv(std::string output_filename, std::string engine_settings_filename)
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
        if (engine_settings.find(parent) != engine_settings.end())
        {
            fprintf(out, "%s [color=green];\n", parent.c_str());
        }
        if (engine_settings.find(child) != engine_settings.end())
        {
            fprintf(out, "%s [color=green];\n", child.c_str());
        }
        std::string color;
        switch (relation_type)
        {
            case SettingsToGv::RelationType::INHERIT_FUNCTION:
                color = "red";
                break;
            case SettingsToGv::RelationType::PARENT_CHILD:
                color = "black";
                break;
        }
        fprintf(out, "edge [color=%s];\n", color.c_str());
        fprintf(out, "%s -> %s;\n", parent.c_str(), child.c_str());
    }
    
    void parseSetting(const std::string& parent, rapidjson::Value::ConstMemberIterator json_object_it)
    {
        std::string name = json_object_it->name.GetString();
        
//         std::cerr << "parsed: " << name <<"\n";
        
        bool generated_edge = false;
        
        const rapidjson::Value& data = json_object_it->value;
        if (data.HasMember("inherit_function"))
        {
            assert(data["inherit_function"].IsString());
            std::string inherit_function = data["inherit_function"].GetString();
            std::regex setting_name_regex("[a-zA-Z0-9_]+"); // matches mostly with setting names
            std::smatch regex_match;
            while (std::regex_search (inherit_function, regex_match, setting_name_regex))
            {
                std::string inherited_setting_string = regex_match[0];
                if (inherited_setting_string == "parent_value")
                {
                    generateEdge(parent, name, RelationType::PARENT_CHILD);
                    generated_edge = true;
                }
                else if ( ! std::regex_match(inherited_setting_string, std::regex("[0-9]+")) && // exclude numbers
                    // result != "parent_value" && 
                    inherited_setting_string != "if" && inherited_setting_string != "else" && inherited_setting_string != "and" && inherited_setting_string != "or" && inherited_setting_string != "math" && inherited_setting_string != "ceil" && inherited_setting_string != "int" && inherited_setting_string != "round" && inherited_setting_string != "max" // exclude operators and functions
                    && inherit_function.c_str()[regex_match.position() + regex_match.length()] != '\'') // exclude enum terms
                {
                    if (inherited_setting_string == parent)
                    {
                        generated_edge = true;
                        generateEdge(inherited_setting_string, name, RelationType::PARENT_CHILD);
                    }
                    else 
                    {
                        generateEdge(inherited_setting_string, name, RelationType::INHERIT_FUNCTION);
                    }
                }
                inherit_function = regex_match.suffix().str();
            }
        }
        
        if (!generated_edge && parent != "")
        {
            generateEdge(parent, name, RelationType::PARENT_CHILD);
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
        if (json_document.HasMember("machine_settings"))
        {
//             std::cerr << "machine settings:\n";
            const rapidjson::Value& machine_settings = json_document["machine_settings"];
            for (rapidjson::Value::ConstMemberIterator setting_iterator = machine_settings.MemberBegin(); setting_iterator != machine_settings.MemberEnd(); ++setting_iterator)
            {
                parseSetting("", setting_iterator);
            }
        }
        if (json_document.HasMember("categories"))
        {
//             std::cerr << "categories:\n";
            for (rapidjson::Value::ConstMemberIterator category_iterator = json_document["categories"].MemberBegin(); category_iterator != json_document["categories"].MemberEnd(); ++category_iterator)
            {
                const rapidjson::Value& json_object_container = category_iterator->value["settings"];
                for (rapidjson::Value::ConstMemberIterator setting_iterator = json_object_container.MemberBegin(); setting_iterator != json_object_container.MemberEnd(); ++setting_iterator)
                {
                    parseSetting("", setting_iterator);
                }
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