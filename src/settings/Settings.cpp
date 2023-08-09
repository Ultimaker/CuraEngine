// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "settings/Settings.h"

#include "Application.h" //To get the extruders.
#include "BeadingStrategy/BeadingStrategyFactory.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "settings/EnumSettings.h"
#include "settings/FlowTempGraph.h"
#include "settings/types/Angle.h"
#include "settings/types/Duration.h" //For duration and time settings.
#include "settings/types/LayerIndex.h" //For layer index settings.
#include "settings/types/Ratio.h" //For ratio settings and percentages.
#include "settings/types/Temperature.h" //For temperature settings.
#include "settings/types/Velocity.h" //For velocity settings.
#include "utils/FMatrix4x3.h"
#include "utils/polygon.h"
#include "utils/string.h" //For Escaped.
#include "utils/types/string_switch.h" //For string switch.

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/map.hpp>
#include <spdlog/spdlog.h>

#include <cctype>
#include <fstream>
#include <regex> // regex parsing for temp flow graph
#include <sstream> // ostringstream
#include <stdio.h>
#include <string> //Parsing strings (stod, stoul).

namespace cura
{

Settings::Settings()
{
    parent = nullptr; // Needs to be properly initialised because we check against this if the parent is not set.
}

void Settings::add(const std::string& key, const std::string value)
{
    if (settings.find(key) != settings.end()) // Already exists.
    {
        settings[key] = value;
    }
    else // New setting.
    {
        settings.emplace(key, value);
    }
}

template<>
std::string Settings::get<std::string>(const std::string& key) const
{
    // If this settings base has a setting value for it, look that up.
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

    spdlog::error("Trying to retrieve setting with no value given: {}", key);
    std::exit(2);
}

template<>
double Settings::get<double>(const std::string& key) const
{
    return atof(get<std::string>(key).c_str());
}

template<>
size_t Settings::get<size_t>(const std::string& key) const
{
    return std::stoul(get<std::string>(key).c_str());
}

template<>
int Settings::get<int>(const std::string& key) const
{
    return atoi(get<std::string>(key).c_str());
}

template<>
bool Settings::get<bool>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "on" || value == "yes" || value == "true" || value == "True")
    {
        return true;
    }
    const int num = atoi(value.c_str());
    return num != 0;
}

template<>
ExtruderTrain& Settings::get<ExtruderTrain&>(const std::string& key) const
{
    int extruder_nr = std::atoi(get<std::string>(key).c_str());
    if (extruder_nr < 0)
    {
        extruder_nr = get<size_t>("extruder_nr");
    }
    return Application::getInstance().current_slice->scene.extruders[extruder_nr];
}

template<>
std::vector<ExtruderTrain*> Settings::get<std::vector<ExtruderTrain*>>(const std::string& key) const
{
    int extruder_nr = std::atoi(get<std::string>(key).c_str());
    std::vector<ExtruderTrain*> ret;
    if (extruder_nr < 0)
    {
        for (ExtruderTrain& train : Application::getInstance().current_slice->scene.extruders)
        {
            ret.emplace_back(&train);
        }
    }
    else
    {
        ret.emplace_back(&Application::getInstance().current_slice->scene.extruders[extruder_nr]);
    }
    return ret;
}

template<>
LayerIndex Settings::get<LayerIndex>(const std::string& key) const
{
    // For the user we display layer numbers starting from 1, but we start counting from 0. Still it may be negative for Raft layers.
    return std::atoi(get<std::string>(key).c_str()) - 1;
}

template<>
coord_t Settings::get<coord_t>(const std::string& key) const
{
    return MM2INT(get<double>(key)); // The settings are all in millimetres, but we need to interpret them as microns.
}

template<>
AngleRadians Settings::get<AngleRadians>(const std::string& key) const
{
    return get<double>(key) * M_PI / 180; // The settings are all in degrees, but we need to interpret them as radians.
}

template<>
AngleDegrees Settings::get<AngleDegrees>(const std::string& key) const
{
    return get<double>(key);
}

template<>
Temperature Settings::get<Temperature>(const std::string& key) const
{
    return get<double>(key);
}

template<>
Velocity Settings::get<Velocity>(const std::string& key) const
{
    return get<double>(key);
}

template<>
Acceleration Settings::get<Acceleration>(const std::string& key) const
{
    return get<double>(key);
}

template<>
Ratio Settings::get<Ratio>(const std::string& key) const
{
    return get<double>(key) / 100.0; // The settings are all in percentages, but we need to interpret them as radians.
}

template<>
Duration Settings::get<Duration>(const std::string& key) const
{
    return get<double>(key);
}

template<>
DraftShieldHeightLimitation Settings::get<DraftShieldHeightLimitation>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "full"_sw:
        return DraftShieldHeightLimitation::FULL;
    case "limited"_sw:
        return DraftShieldHeightLimitation::LIMITED;
    case "plugin"_sw:
        return DraftShieldHeightLimitation::PLUGIN;
    default:
        return DraftShieldHeightLimitation::FULL;
    }
}

template<>
FlowTempGraph Settings::get<FlowTempGraph>(const std::string& key) const
{
    std::string value_string = get<std::string>(key);

    FlowTempGraph result;
    if (value_string.empty())
    {
        return result; // Empty at this point.
    }
    /* Match with:
     * - the last opening bracket '['
     * - then a bunch of characters until the first comma
     * - a comma
     * - a bunch of characters until the first closing bracket ']'.
     * This matches with any substring which looks like "[ 124.512 , 124.1 ]".
     */
    std::regex regex("(\\[([^,\\[]*),([^,\\]]*)\\])");

    // Default constructor = end-of-sequence:
    std::regex_token_iterator<std::string::iterator> rend;

    const int submatches[] = { 1, 2, 3 }; // Match whole pair, first number and second number of a pair.
    std::regex_token_iterator<std::string::iterator> match_iter(value_string.begin(), value_string.end(), regex, submatches);
    while (match_iter != rend)
    {
        match_iter++; // Match the whole pair.
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
            spdlog::error("Couldn't read 2D graph element [{},{}] in setting {}. Ignored.", first_substring, second_substring, key);
        }
    }

    return result;
}

template<>
Polygons Settings::get<Polygons>(const std::string& key) const
{
    std::string value_string = get<std::string>(key);

    Polygons result;
    if (value_string.empty())
    {
        return result; // Empty at this point.
    }
    /* We're looking to match one or more floating point values separated by
     * commas and surrounded by square brackets. Note that because the QML
     * RegExpValidator only stops unrecognised characters being input and
     * doesn't actually barf if the trailing ']' is missing, we are lenient here
     * and make that bracket optional.
     */
    std::regex polygons_regex(R"(\[(.*)\]?)");
    std::smatch polygons_match;
    if (std::regex_search(value_string, polygons_match, polygons_regex) && polygons_match.size() > 1)
    {
        std::string polygons_string = polygons_match.str(1);

        std::regex polygon_regex(R"(\[((\[[^\[\]]*\]\s*,?\s*)*)\]\s*,?)"); // matches with a list of lists (a list of 2D vertices)
        std::smatch polygon_match;

        std::regex_token_iterator<std::string::iterator> rend; // Default constructor gets the end-of-sequence iterator.
        std::regex_token_iterator<std::string::iterator> polygon_match_iter(polygons_string.begin(), polygons_string.end(), polygon_regex, 0);
        while (polygon_match_iter != rend)
        {
            std::string polygon_str = *polygon_match_iter++;

            result.emplace_back();
            PolygonRef poly = result.back();

            std::regex point2D_regex(R"(\[([^,\[]*),([^,\]]*)\])"); // matches to a list of exactly two things

            const int submatches[] = { 1, 2 }; // Match first number and second number of a pair.
            std::regex_token_iterator<std::string::iterator> match_iter(polygon_str.begin(), polygon_str.end(), point2D_regex, submatches);
            while (match_iter != rend)
            {
                std::string first_substring = *match_iter++;
                std::string second_substring = *match_iter++;
                try
                {
                    double first = std::stod(first_substring);
                    double second = std::stod(second_substring);
                    poly.emplace_back(MM2INT(first), MM2INT(second));
                }
                catch (const std::invalid_argument& e)
                {
                    spdlog::error("Couldn't read 2D graph element [{},{}] in setting '{}'. Ignored.\n", first_substring.c_str(), second_substring.c_str(), key.c_str());
                }
                if (match_iter == rend)
                {
                    break;
                }
            }
        }
    }
    return result;
}

template<>
FMatrix4x3 Settings::get<FMatrix4x3>(const std::string& key) const
{
    const std::string value_string = get<std::string>(key);

    FMatrix4x3 result;
    if (value_string.empty())
    {
        return result; // Standard matrix ([[1,0,0], [0,1,0], [0,0,1]]).
    }

    std::string num("([^,\\] ]*)"); // Match with anything but the next ',' ']' or space and capture the match.
    std::ostringstream row; // Match with "[num,num,num]" and ignore whitespace.
    row << "\\s*\\[\\s*" << num << "\\s*,\\s*" << num << "\\s*,\\s*" << num << "\\s*\\]\\s*";
    std::ostringstream matrix; // Match with "[row,row,row]" and ignore whitespace.
    matrix << "\\s*\\[\\s*" << row.str() << "\\s*,\\s*" << row.str() << "\\s*,\\s*" << row.str() << "\\]\\s*";

    std::regex point_matrix_regex(matrix.str());
    std::cmatch sub_matches; // Same as std::match_results<const char*> cm;
    std::regex_match(value_string.c_str(), sub_matches, point_matrix_regex);
    if (sub_matches.size() != 10) // One match for the whole string, nine for the cells.
    {
        spdlog::warn("Mesh transformation matrix could not be parsed!");
        spdlog::debug("Format should be [[f,f,f], [f,f,f], [f,f,f]] allowing whitespace anywhere in between. While what was given was {}", value_string);
        return result; // Standard matrix ([[1,0,0], [0,1,0], [0,0,1]]).
    }

    size_t sub_match_index = 1; // Skip the first because the first submatch is the whole string.
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

template<>
EGCodeFlavor Settings::get<EGCodeFlavor>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "Marlin"_sw:
        return EGCodeFlavor::MARLIN;
    case "Griffin"_sw:
        return EGCodeFlavor::GRIFFIN;
    case "UltiGCode"_sw:
        return EGCodeFlavor::ULTIGCODE;
    case "Makerbot"_sw:
        return EGCodeFlavor::MAKERBOT;
    case "BFB"_sw:
        return EGCodeFlavor::BFB;
    case "MACH3"_sw:
        return EGCodeFlavor::MACH3;
    case "RepRap (Volumetric)"_sw:
        return EGCodeFlavor::MARLIN_VOLUMATRIC;
    case "Repetier"_sw:
        return EGCodeFlavor::REPETIER;
    case "RepRap (RepRap)"_sw:
        return EGCodeFlavor::REPRAP;
    case "plugin"_sw:
        return EGCodeFlavor::PLUGIN;
    default:
        return EGCodeFlavor::MARLIN;
    }
}

template<>
EFillMethod Settings::get<EFillMethod>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "none"_sw:
        return EFillMethod::NONE;
    case "lines"_sw:
        return EFillMethod::LINES;
    case "grid"_sw:
        return EFillMethod::GRID;
    case "cubic"_sw:
        return EFillMethod::CUBIC;
    case "cubicsubdiv"_sw:
        return EFillMethod::CUBICSUBDIV;
    case "tetrahedral"_sw:
        return EFillMethod::TETRAHEDRAL;
    case "quarter_cubic"_sw:
        return EFillMethod::QUARTER_CUBIC;
    case "triangles"_sw:
        return EFillMethod::TRIANGLES;
    case "trihexagon"_sw:
        return EFillMethod::TRIHEXAGON;
    case "concentric"_sw:
        return EFillMethod::CONCENTRIC;
    case "zigzag"_sw:
        return EFillMethod::ZIG_ZAG;
    case "cross"_sw:
        return EFillMethod::CROSS;
    case "cross_3d"_sw:
        return EFillMethod::CROSS_3D;
    case "gyroid"_sw:
        return EFillMethod::GYROID;
    case "lightning"_sw:
        return EFillMethod::LIGHTNING;
    case "plugin"_sw:
        return EFillMethod::PLUGIN;
    default:
        return EFillMethod::NONE;
    }
}

template<>
EPlatformAdhesion Settings::get<EPlatformAdhesion>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "skirt"_sw:
        return EPlatformAdhesion::SKIRT;
    case "brim"_sw:
        return EPlatformAdhesion::BRIM;
    case "raft"_sw:
        return EPlatformAdhesion::RAFT;
    case "none"_sw:
        return EPlatformAdhesion::NONE;
    case "plugin"_sw:
        return EPlatformAdhesion::PLUGIN;
    default:
        return EPlatformAdhesion::SKIRT;
    }
}

template<>
ESupportType Settings::get<ESupportType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "none"_sw:
        return ESupportType::NONE;
    case "everywhere"_sw:
        return ESupportType::EVERYWHERE;
    case "buildplate"_sw:
        return ESupportType::PLATFORM_ONLY;
    case "plugin"_sw:
        return ESupportType::PLUGIN;
    default:
        return ESupportType::NONE;
    }
}

template<>
ESupportStructure Settings::get<ESupportStructure>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "normal"_sw:
        return ESupportStructure::NORMAL;
    case "tree"_sw:
        return ESupportStructure::TREE;
    case "plugin"_sw:
        return ESupportStructure::PLUGIN;
    default:
        return ESupportStructure::NORMAL;
    }
}


template<>
EZSeamType Settings::get<EZSeamType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "shortest"_sw:
        return EZSeamType::SHORTEST;
    case "random"_sw:
        return EZSeamType::RANDOM;
    case "back"_sw:
        return EZSeamType::USER_SPECIFIED;
    case "sharpest_corner"_sw:
        return EZSeamType::SHARPEST_CORNER;
    case "plugin"_sw:
        return EZSeamType::PLUGIN;
    default:
        return EZSeamType::SHORTEST;
    }
}

template<>
EZSeamCornerPrefType Settings::get<EZSeamCornerPrefType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "z_seam_corner_none"_sw:
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE;
    case "z_seam_corner_inner"_sw:
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER;
    case "z_seam_corner_outer"_sw:
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER;
    case "z_seam_corner_any"_sw:
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY;
    case "z_seam_corner_weighted"_sw:
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED;
    case "plugin"_sw:
        return EZSeamCornerPrefType::PLUGIN;
    default:
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE;
    }
}

template<>
ESurfaceMode Settings::get<ESurfaceMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "normal"_sw:
        return ESurfaceMode::NORMAL;
    case "surface"_sw:
        return ESurfaceMode::SURFACE;
    case "both"_sw:
        return ESurfaceMode::BOTH;
    case "plugin"_sw:
        return ESurfaceMode::PLUGIN;
    default:
        return ESurfaceMode::NORMAL;
    }
}

template<>
FillPerimeterGapMode Settings::get<FillPerimeterGapMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "nowhere"_sw:
        return FillPerimeterGapMode::NOWHERE;
    case "everywhere"_sw:
        return FillPerimeterGapMode::EVERYWHERE;
    case "plugin"_sw:
        return FillPerimeterGapMode::PLUGIN;
    default:
        return FillPerimeterGapMode::NOWHERE;
    }
}

template<>
BuildPlateShape Settings::get<BuildPlateShape>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "rectangular"_sw:
        return BuildPlateShape::RECTANGULAR;
    case "elliptic"_sw:
        return BuildPlateShape::ELLIPTIC;
    case "plugin"_sw:
        return BuildPlateShape::PLUGIN;
    default:
        return BuildPlateShape::RECTANGULAR;
    }
}

template<>
CombingMode Settings::get<CombingMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "all"_sw:
        return CombingMode::ALL;
    case "off"_sw:
        return CombingMode::OFF;
    case "noskin"_sw:
        return CombingMode::NO_SKIN;
    case "no_outer_surfaces"_sw:
        return CombingMode::NO_OUTER_SURFACES;
    case "infill"_sw:
        return CombingMode::INFILL;
    case "plugin"_sw:
        return CombingMode::PLUGIN;
    default:
        return CombingMode::ALL;
    }
}

template<>
SupportDistPriority Settings::get<SupportDistPriority>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "xy_overrides_z"_sw:
        return SupportDistPriority::XY_OVERRIDES_Z;
    case "z_overrides_xy"_sw:
        return SupportDistPriority::Z_OVERRIDES_XY;
    case "plugin"_sw:
        return SupportDistPriority::PLUGIN;
    default:
        return SupportDistPriority::XY_OVERRIDES_Z;
    }
}

template<>
SlicingTolerance Settings::get<SlicingTolerance>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "middle"_sw:
        return SlicingTolerance::MIDDLE;
    case "inclusive"_sw:
        return SlicingTolerance::INCLUSIVE;
    case "exclusive"_sw:
        return SlicingTolerance::EXCLUSIVE;
    case "plugin"_sw:
        return SlicingTolerance::PLUGIN;
    default:
        return SlicingTolerance::MIDDLE;
    }
}

template<>
InsetDirection Settings::get<InsetDirection>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using namespace cura::utils;
    switch (hash_enum(value))
    {
    case "inside_out"_sw:
        return InsetDirection::INSIDE_OUT;
    case "outside_in"_sw:
        return InsetDirection::OUTSIDE_IN;
    case "plugin"_sw:
        return InsetDirection::PLUGIN;
    default:
        return InsetDirection::INSIDE_OUT;
    }
}

template<>
std::vector<double> Settings::get<std::vector<double>>(const std::string& key) const
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
        std::regex_token_iterator<std::string::iterator> rend; // Default constructor gets the end-of-sequence iterator.

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
                spdlog::error("Couldn't read floating point value ({}) in setting {}. Ignored.", value, key);
            }
        }
    }
    return result;
}

template<>
std::vector<int> Settings::get<std::vector<int>>(const std::string& key) const
{
    std::vector<double> values_doubles = get<std::vector<double>>(key);
    std::vector<int> values_ints;
    values_ints.reserve(values_doubles.size());
    for (double value : values_doubles)
    {
        values_ints.push_back(std::round(value)); // Round to nearest integer.
    }
    return values_ints;
}

template<>
std::vector<AngleDegrees> Settings::get<std::vector<AngleDegrees>>(const std::string& key) const
{
    std::vector<double> values_doubles = get<std::vector<double>>(key);
    return std::vector<AngleDegrees>(values_doubles.begin(), values_doubles.end()); // Cast them to AngleDegrees.
}

const std::string Settings::getAllSettingsString() const
{
    std::stringstream sstream;
    for (const std::pair<std::string, std::string> pair : settings)
    {
        char buffer[4096];
        snprintf(buffer, 4096, " -s %s=\"%s\"", pair.first.c_str(), Escaped{ pair.second.c_str() }.str);
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
        spdlog::error("Trying to retrieve setting with no value given: {}", key);
        std::exit(2);
    }
}

std::unordered_map<std::string, std::string> Settings::getFlattendSettings() const
{
    auto keys = getKeys();
    return keys
         | ranges::views::transform(
               [&](const auto& key)
               {
                   return std::pair<std::string, std::string>(key, get<std::string>(key));
               })
         | ranges::to<std::unordered_map<std::string, std::string>>();
}

std::vector<std::string> Settings::getKeys() const
{
    if (parent)
    {
        return parent->getKeys();
    }
    return ranges::views::keys(settings) | ranges::to_vector;
}

} // namespace cura
