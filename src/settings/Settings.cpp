//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include <regex> // regex parsing for temp flow graph
#include <string> //Parsing strings (stod, stoul).

#include "EnumSettings.h"
#include "FlowTempGraph.h"
#include "Settings.h"
#include "types/Angle.h"
#include "types/Duration.h" //For duration and time settings.
#include "types/LayerIndex.h" //For layer index settings.
#include "types/Ratio.h" //For ratio settings and percentages.
#include "types/Temperature.h" //For temperature settings.
#include "types/Velocity.h" //For velocity settings.
#include "../Application.h" //To get the extruders.
#include "../ExtruderTrain.h"
#include "../Slice.h"
#include "../utils/FMatrix4x3.h"
#include "../utils/logoutput.h"
#include "../utils/string.h" //For Escaped.
#include "../BeadingStrategy/BeadingStrategyFactory.h"

namespace cura
{

Settings::Settings()
{
    parent = nullptr; //Needs to be properly initialised because we check against this if the parent is not set.
}

void Settings::add(const std::string& key, const std::string value)
{
    if (settings.find(key) != settings.end()) //Already exists.
    {
        settings[key] = value;
    }
    else //New setting.
    {
        settings.emplace(key, value);
    }
}

template<> std::string Settings::get<std::string>(const std::string& key) const
{
    //If this settings base has a setting value for it, look that up.
    if (settings.find(key) != settings.end())
    {
        return settings.at(key);
    }

    const std::unordered_map<std::string, ExtruderTrain*>& limit_to_extruder = Application::getInstance().current_slice->scene.limit_to_extruder;
    if (limit_to_extruder.find(key) != limit_to_extruder.end())
    {
        return limit_to_extruder.at(key)->settings.getWithoutLimiting(key);
    }

    if (parent)
    {
        return parent->get<std::string>(key);
    }

    logError("Trying to retrieve setting with no value given: '%s'\n", key.c_str());
    std::exit(2);
}

template<> double Settings::get<double>(const std::string& key) const
{
    return atof(get<std::string>(key).c_str());
}

template<> size_t Settings::get<size_t>(const std::string& key) const
{
    return std::stoul(get<std::string>(key).c_str());
}

template<> int Settings::get<int>(const std::string& key) const
{
    return atoi(get<std::string>(key).c_str());
}

template<> bool Settings::get<bool>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "on" || value == "yes" || value == "true" || value == "True")
    {
        return true;
    }
    const int num = atoi(value.c_str());
    return num != 0;
}

template<> ExtruderTrain& Settings::get<ExtruderTrain&>(const std::string& key) const
{
    int extruder_nr = std::atoi(get<std::string>(key).c_str());
    if (extruder_nr < 0)
    {
        extruder_nr = get<size_t>("extruder_nr");
    }
    return Application::getInstance().current_slice->scene.extruders[extruder_nr];
}

template<> LayerIndex Settings::get<LayerIndex>(const std::string& key) const
{
    return std::atoi(get<std::string>(key).c_str()) - 1; //For the user we display layer numbers starting from 1, but we start counting from 0. Still it may be negative for Raft layers.
}

template<> coord_t Settings::get<coord_t>(const std::string& key) const
{
    return MM2INT(get<double>(key)); //The settings are all in millimetres, but we need to interpret them as microns.
}

template<> AngleRadians Settings::get<AngleRadians>(const std::string& key) const
{
    return get<double>(key) * M_PI / 180; //The settings are all in degrees, but we need to interpret them as radians.
}

template<> AngleDegrees Settings::get<AngleDegrees>(const std::string& key) const
{
    return get<double>(key);
}

template<> Temperature Settings::get<Temperature>(const std::string& key) const
{
    return get<double>(key);
}

template<> Velocity Settings::get<Velocity>(const std::string& key) const
{
    return get<double>(key);
}

template<> Ratio Settings::get<Ratio>(const std::string& key) const
{
    return get<double>(key) / 100.0; //The settings are all in percentages, but we need to interpret them as radians.
}

template<> Duration Settings::get<Duration>(const std::string& key) const
{
    return get<double>(key);
}

template<> DraftShieldHeightLimitation Settings::get<DraftShieldHeightLimitation>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "limited")
    {
        return DraftShieldHeightLimitation::LIMITED;
    }
    else //if (value == "full") or default.
    {
        return DraftShieldHeightLimitation::FULL;
    }
}

template<> FlowTempGraph Settings::get<FlowTempGraph>(const std::string& key) const
{
    std::string value_string = get<std::string>(key);

    FlowTempGraph result;
    if (value_string.empty())
    {
        return result; //Empty at this point.
    }
    /* Match with:
     * - the last opening bracket '['
     * - then a bunch of characters until the first comma
     * - a comma
     * - a bunch of characters until the first closing bracket ']'.
     * This matches with any substring which looks like "[ 124.512 , 124.1 ]".
     */
    std::regex regex("(\\[([^,\\[]*),([^,\\]]*)\\])");

    //Default constructor = end-of-sequence:
    std::regex_token_iterator<std::string::iterator> rend;

    const int submatches[] = {1, 2, 3}; //Match whole pair, first number and second number of a pair.
    std::regex_token_iterator<std::string::iterator> match_iter(value_string.begin(), value_string.end(), regex, submatches);
    while (match_iter != rend)
    {
        match_iter++; //Match the whole pair.
        if (match_iter == rend)
        {
            break;
        }
        std::string first_substring = *match_iter++;
        std::string second_substring = *match_iter++;
        try
        {
            double first = std::stod(first_substring);
            double second = std::stod(second_substring);
            result.data.emplace_back(first, second);
        }
        catch (const std::invalid_argument& e)
        {
            logError("Couldn't read 2D graph element [%s,%s] in setting '%s'. Ignored.\n", first_substring.c_str(), second_substring.c_str(), key.c_str());
        }
    }

    return result;
}

template<> FMatrix4x3 Settings::get<FMatrix4x3>(const std::string& key) const
{
    const std::string value_string = get<std::string>(key);

    FMatrix4x3 result;
    if (value_string.empty())
    {
        return result; //Standard matrix ([[1,0,0], [0,1,0], [0,0,1]]).
    }

    std::string num("([^,\\] ]*)"); //Match with anything but the next ',' ']' or space and capture the match.
    std::ostringstream row; //Match with "[num,num,num]" and ignore whitespace.
    row << "\\s*\\[\\s*" << num << "\\s*,\\s*" << num << "\\s*,\\s*" << num << "\\s*\\]\\s*";
    std::ostringstream matrix; //Match with "[row,row,row]" and ignore whitespace.
    matrix << "\\s*\\[\\s*" << row.str() << "\\s*,\\s*" << row.str() << "\\s*,\\s*" << row.str() << "\\]\\s*";

    std::regex point_matrix_regex(matrix.str());
    std::cmatch sub_matches; //Same as std::match_results<const char*> cm;
    std::regex_match(value_string.c_str(), sub_matches, point_matrix_regex);
    if (sub_matches.size() != 10) //One match for the whole string, nine for the cells.
    {
        logWarning("Mesh transformation matrix could not be parsed!\n\tFormat should be [[f,f,f], [f,f,f], [f,f,f]] allowing whitespace anywhere in between.\n\tWhile what was given was \"%s\".\n", value_string.c_str());
        return result; //Standard matrix ([[1,0,0], [0,1,0], [0,0,1]]).
    }

    size_t sub_match_index = 1; //Skip the first because the first submatch is the whole string.
    for (size_t x = 0; x < 3; x++)
    {
        for (size_t y = 0; y < 3; y++)
        {
            std::sub_match<const char*> sub_match = sub_matches[sub_match_index];
            result.m[y][x] = strtod(std::string(sub_match.str()).c_str(), nullptr);
            sub_match_index++;
        }
    }

    return result;
}

template<> EGCodeFlavor Settings::get<EGCodeFlavor>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    //I wish that switch statements worked for std::string...
    if (value == "Griffin")
    {
        return EGCodeFlavor::GRIFFIN;
    }
    else if (value == "UltiGCode")
    {
        return EGCodeFlavor::ULTIGCODE;
    }
    else if (value == "Makerbot")
    {
        return EGCodeFlavor::MAKERBOT;
    }
    else if (value == "BFB")
    {
        return EGCodeFlavor::BFB;
    }
    else if (value == "MACH3")
    {
        return EGCodeFlavor::MACH3;
    }
    else if (value == "RepRap (Volumetric)")
    {
        return EGCodeFlavor::MARLIN_VOLUMATRIC;
    }
    else if (value == "Repetier")
    {
        return EGCodeFlavor::REPETIER;
    }
    else if (value == "RepRap (RepRap)")
    {
        return EGCodeFlavor::REPRAP;
    }
    //Default:
    return EGCodeFlavor::MARLIN;
}

template<> EFillMethod Settings::get<EFillMethod>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "lines")
    {
        return EFillMethod::LINES;
    }
    else if (value == "grid")
    {
        return EFillMethod::GRID;
    }
    else if (value == "cubic")
    {
        return EFillMethod::CUBIC;
    }
    else if (value == "cubicsubdiv")
    {
        return EFillMethod::CUBICSUBDIV;
    }
    else if (value == "tetrahedral")
    {
        return EFillMethod::TETRAHEDRAL;
    }
    else if (value == "quarter_cubic")
    {
        return EFillMethod::QUARTER_CUBIC;
    }
    else if (value == "triangles")
    {
        return EFillMethod::TRIANGLES;
    }
    else if (value == "trihexagon")
    {
        return EFillMethod::TRIHEXAGON;
    }
    else if (value == "concentric")
    {
        return EFillMethod::CONCENTRIC;
    }
    else if (value == "zigzag")
    {
        return EFillMethod::ZIG_ZAG;
    }
    else if (value == "cross")
    {
        return EFillMethod::CROSS;
    }
    else if (value == "cross_3d")
    {
        return EFillMethod::CROSS_3D;
    }
    else if (value == "gyroid")
    {
        return EFillMethod::GYROID;
    }
    else if (value == "lightning")
    {
        return EFillMethod::LIGHTNING;
    }
    else //Default.
    {
        return EFillMethod::NONE;
    }
}

template<>  StrategyType Settings::get<StrategyType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "center_deviation")
    {
        return StrategyType::Center;
    }
    else if (value == "distributed")
    {
        return StrategyType::Distributed;
    }
    else if (value == "inward_distributed")
    {
        return StrategyType::InwardDistributed;
    }
    else //Default.
    {
        return StrategyType::None;
    }
}

template<> EPlatformAdhesion Settings::get<EPlatformAdhesion>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "brim")
    {
        return EPlatformAdhesion::BRIM;
    }
    else if (value == "raft")
    {
        return EPlatformAdhesion::RAFT;
    }
    else if (value == "none")
    {
        return EPlatformAdhesion::NONE;
    }
    else //Default.
    {
        return EPlatformAdhesion::SKIRT;
    }
}

template<> ESupportType Settings::get<ESupportType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "everywhere")
    {
        return ESupportType::EVERYWHERE;
    }
    else if (value == "buildplate")
    {
        return ESupportType::PLATFORM_ONLY;
    }
    else //Default.
    {
        return ESupportType::NONE;
    }
}

template<> ESupportStructure Settings::get<ESupportStructure>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "normal")
    {
        return ESupportStructure::NORMAL;
    }
    else if (value == "tree")
    {
        return ESupportStructure::TREE;
    }
    else //Default.
    {
        return ESupportStructure::NORMAL;
    }
}


template<> EZSeamType Settings::get<EZSeamType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "random")
    {
        return EZSeamType::RANDOM;
    }
    else if (value == "back") //It's called 'back' internally because originally this was intended to allow the user to put the seam in the back of the object where it's less visible.
    {
        return EZSeamType::USER_SPECIFIED;
    }
    else if (value == "sharpest_corner")
    {
        return EZSeamType::SHARPEST_CORNER;
    }
    else //Default.
    {
        return EZSeamType::SHORTEST;
    }
}

template<> EZSeamCornerPrefType Settings::get<EZSeamCornerPrefType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "z_seam_corner_inner")
    {
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER;
    }
    else if (value == "z_seam_corner_outer")
    {
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER;
    }
    else if (value == "z_seam_corner_any")
    {
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY;
    }
    else if (value == "z_seam_corner_weighted")
    {
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED;
    }
    else //Default.
    {
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE;
    }
}

template<> ESurfaceMode Settings::get<ESurfaceMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "surface")
    {
        return ESurfaceMode::SURFACE;
    }
    else if (value == "both")
    {
        return ESurfaceMode::BOTH;
    }
    else //Default.
    {
        return ESurfaceMode::NORMAL;
    }
}

template<> FillPerimeterGapMode Settings::get<FillPerimeterGapMode>(const std::string& key) const
{
    if (get<std::string>(key) == "everywhere")
    {
        return FillPerimeterGapMode::EVERYWHERE;
    }
    else //Default.
    {
        return FillPerimeterGapMode::NOWHERE;
    }
}

template<> BuildPlateShape Settings::get<BuildPlateShape>(const std::string& key) const
{
    if (get<std::string>(key) == "elliptic")
    {
        return BuildPlateShape::ELLIPTIC;
    }
    else //Default.
    {
        return BuildPlateShape::RECTANGULAR;
    }
}

template<> CombingMode Settings::get<CombingMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "off")
    {
        return CombingMode::OFF;
    }
    else if (value == "noskin")
    {
        return CombingMode::NO_SKIN;
    }
    else if (value == "no_outer_surfaces")
    {
        return CombingMode::NO_OUTER_SURFACES;
    }
    else if (value == "infill")
    {
        return CombingMode::INFILL;
    }
    else //Default.
    {
        return CombingMode::ALL;
    }
}

template<> SupportDistPriority Settings::get<SupportDistPriority>(const std::string& key) const
{
    if (get<std::string>(key) == "z_overrides_xy")
    {
        return SupportDistPriority::Z_OVERRIDES_XY;
    }
    else //Default.
    {
        return SupportDistPriority::XY_OVERRIDES_Z;
    }
}

template<> SlicingTolerance Settings::get<SlicingTolerance>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "inclusive")
    {
        return SlicingTolerance::INCLUSIVE;
    }
    else if (value == "exclusive")
    {
        return SlicingTolerance::EXCLUSIVE;
    }
    else //Default.
    {
        return SlicingTolerance::MIDDLE;
    }
}

template<> InsetDirection Settings::get<InsetDirection>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if(value == "center_last")
    {
        return InsetDirection::CENTER_LAST;
    }
    else if(value == "outside_in")
    {
        return InsetDirection::OUTSIDE_IN;
    }
    else //Default.
    {
        return InsetDirection::INSIDE_OUT;
    }
}

template<> std::vector<double> Settings::get<std::vector<double>>(const std::string& key) const
{
    const std::string& value_string = get<std::string>(key);

    std::vector<double> result;
    if (value_string.empty())
    {
        return result;
    }

    /* We're looking to match one or more floating point values separated by
     * commas and surrounded by square brackets. Note that because the QML
     * RegExpValidator only stops unrecognised characters being input and
     * doesn't actually barf if the trailing ']' is missing, we are lenient here
     * and make that bracket optional.
     */
    std::regex list_contents_regex(R"(\[([^\]]*)\]?)");
    std::smatch list_contents_match;
    if (std::regex_search(value_string, list_contents_match, list_contents_regex) && list_contents_match.size() > 1)
    {
        std::string elements = list_contents_match.str(1);
        std::regex element_regex(R"(\s*([+-]?[0-9]*\.?[0-9]+)\s*,?)");
        std::regex_token_iterator<std::string::iterator> rend; //Default constructor gets the end-of-sequence iterator.

        std::regex_token_iterator<std::string::iterator> match_iter(elements.begin(), elements.end(), element_regex, 0);
        while (match_iter != rend)
        {
            std::string value = *match_iter++;
            try
            {
                result.push_back(std::stod(value));
            }
            catch (const std::invalid_argument& e)
            {
                logError("Couldn't read floating point value (%s) in setting '%s'. Ignored.\n", value.c_str(), key.c_str());
            }
        }
    }
    return result;
}

template<> std::vector<int> Settings::get<std::vector<int>>(const std::string& key) const
{
    std::vector<double> values_doubles = get<std::vector<double>>(key);
    std::vector<int> values_ints;
    values_ints.reserve(values_doubles.size());
    for (double value : values_doubles)
    {
        values_ints.push_back(std::round(value)); //Round to nearest integer.
    }
    return values_ints;
}

template<> std::vector<AngleDegrees> Settings::get<std::vector<AngleDegrees>>(const std::string& key) const
{
    std::vector<double> values_doubles = get<std::vector<double>>(key);
    return std::vector<AngleDegrees>(values_doubles.begin(), values_doubles.end()); //Cast them to AngleDegrees.
}

const std::string Settings::getAllSettingsString() const
{
    std::stringstream sstream;
    for (const std::pair<std::string, std::string> pair : settings)
    {
        char buffer[4096];
        snprintf(buffer, 4096, " -s %s=\"%s\"", pair.first.c_str(), Escaped{pair.second.c_str()}.str);
        sstream << buffer;
    }
    return sstream.str();
}

bool Settings::has(const std::string& key) const
{
    return settings.find(key) != settings.end();
}

void Settings::setParent(Settings* new_parent)
{
    parent = new_parent;
}

std::string Settings::getWithoutLimiting(const std::string& key) const
{
    if (settings.find(key) != settings.end())
    {
        return settings.at(key);
    }
    else if (parent)
    {
        return parent->get<std::string>(key);
    }
    else
    {
        logError("Trying to retrieve setting with no value given: '%s'\n", key.c_str());
        std::exit(2);
    }
}

}//namespace cura

