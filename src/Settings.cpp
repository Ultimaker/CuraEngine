// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Settings.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <regex> // regex parsing for temp flow graph
#include <sstream> // ostringstream
#include <string> //Parsing strings (stod, stoul).

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/map.hpp>
#include <spdlog/spdlog.h>

#include "Application.h" //To get the extruders.
#include "beading_strategy/BeadingStrategyFactory.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "geometry/Matrix4x3D.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "types/BrimLocation.h"
#include "types/BuildPlateShape.h"
#include "types/CombingMode.h"
#include "types/CoolDuringExtruderSwitch.h"
#include "types/DraftShieldHeightLimitation.h"
#include "types/EFillMethod.h"
#include "types/EGCodeFlavor.h"
#include "types/EPlatformAdhesion.h"
#include "types/ESupportStructure.h"
#include "types/ESupportType.h"
#include "types/ESurfaceMode.h"
#include "types/EZSeamCornerPrefType.h"
#include "types/EZSeamType.h"
#include "types/FillPerimeterGapMode.h"
#include "types/InfillStartEndPreference.h"
#include "types/InsetDirection.h"
#include "types/PrimeTowerMode.h"
#include "types/RetractBeforeOuterWall.h"
#include "types/SlicingTolerance.h"
#include "types/SupportDistPriority.h"
#include "types/Acceleration.h"
#include "types/AngleDegrees.h"
#include "types/AngleRadians.h"
#include "types/Duration.h" //For duration and time settings.
#include "types/LayerIndex.h" //For layer index settings.
#include "types/Ratio.h" //For ratio settings and percentages.
#include "types/Temperature.h" //For temperature settings.
#include "types/Velocity.h" //For velocity settings.
#include "utils/Escaped.h"
#include "types/string_switch.h" //For string switch.

namespace curaengine
{

Settings::Settings()
{
    parent = nullptr; // Needs to be properly initialised because we check against this if the parent is not set.
}

void Settings::add(const std::string& key, const std::string& value)
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

void Settings::remove(const std::string& key)
{
    const auto iterator = settings.find(key);
    if (iterator != settings.end())
    {
        settings.erase(iterator);
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

    const std::unordered_map<std::string, ExtruderTrain*>& limit_to_extruder = Application::getInstance().current_slice_->scene.limit_to_extruder;
    if (limit_to_extruder.find(key) != limit_to_extruder.end())
    {
        return limit_to_extruder.at(key)->settings_.getWithoutLimiting(key);
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
    return Application::getInstance().current_slice_->scene.extruders[extruder_nr];
}

template<>
std::vector<ExtruderTrain*> Settings::get<std::vector<ExtruderTrain*>>(const std::string& key) const
{
    int extruder_nr = std::atoi(get<std::string>(key).c_str());
    std::vector<ExtruderTrain*> ret;
    if (extruder_nr < 0)
    {
        for (ExtruderTrain& train : Application::getInstance().current_slice_->scene.extruders)
        {
            ret.emplace_back(&train);
        }
    }
    else
    {
        ret.emplace_back(&Application::getInstance().current_slice_->scene.extruders[extruder_nr]);
    }
    return ret;
}

template<>
::curaengine::types::LayerIndex Settings::get<::curaengine::types::LayerIndex>(const std::string& key) const
{
    // For the user we display layer numbers starting from 1, but we start counting from 0. Still it may be negative for Raft layers.
    return std::atoi(get<std::string>(key).c_str()) - 1;
}

template<>
::curaengine::utils::coord_t Settings::get<::curaengine::utils::coord_t>(const std::string& key) const
{
    return MM2INT(get<double>(key)); // The settings are all in millimetres, but we need to interpret them as microns.
}

template<>
::curaengine::types::AngleRadians Settings::get<::curaengine::types::AngleRadians>(const std::string& key) const
{
    return get<double>(key) * std::numbers::pi / 180; // The settings are all in degrees, but we need to interpret them as radians.
}

template<>
::curaengine::types::AngleDegrees Settings::get<::curaengine::types::AngleDegrees>(const std::string& key) const
{
    return get<double>(key);
}

template<>
::curaengine::types::Temperature Settings::get<::curaengine::types::Temperature>(const std::string& key) const
{
    return get<double>(key);
}

template<>
::curaengine::types::Velocity Settings::get<::curaengine::types::Velocity>(const std::string& key) const
{
    return get<double>(key);
}

template<>
::curaengine::types::Acceleration Settings::get<::curaengine::types::Acceleration>(const std::string& key) const
{
    return get<double>(key);
}

template<>
::curaengine::types::Ratio Settings::get<::curaengine::types::Ratio>(const std::string& key) const
{
    return get<double>(key) / 100.0; // The settings are all in percentages, but we need to interpret them as ratios.
}

template<>
::curaengine::types::Duration Settings::get<::curaengine::types::Duration>(const std::string& key) const
{
    return get<double>(key);
}

template<>
::curaengine::types::DraftShieldHeightLimitation Settings::get<::curaengine::types::DraftShieldHeightLimitation>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "full"_sw:
        return ::curaengine::types::DraftShieldHeightLimitation::FULL;
    case "limited"_sw:
        return ::curaengine::types::DraftShieldHeightLimitation::LIMITED;
    case "plugin"_sw:
        return ::curaengine::types::DraftShieldHeightLimitation::PLUGIN;
    default:
        return ::curaengine::types::DraftShieldHeightLimitation::FULL;
    }
}

template<>
::curaengine::geometry::Shape Settings::get<::curaengine::geometry::Shape>(const std::string& key) const
{
    std::string value_string = get<std::string>(key);

    ::curaengine::geometry::Shape result;
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

            ::curaengine::geometry::Polygon& poly = result.newLine();

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
::curaengine::geometry::Matrix4x3D Settings::get<::curaengine::geometry::Matrix4x3D>(const std::string& key) const
{
    const std::string value_string = get<std::string>(key);

    ::curaengine::geometry::Matrix4x3D result;
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
::curaengine::types::EGCodeFlavor Settings::get<::curaengine::types::EGCodeFlavor>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "Marlin"_sw:
        return ::curaengine::types::EGCodeFlavor::MARLIN;
    case "Griffin"_sw:
        return ::curaengine::types::EGCodeFlavor::GRIFFIN;
    case "Cheetah"_sw:
        return ::curaengine::types::EGCodeFlavor::CHEETAH;
    case "UltiGCode"_sw:
        return ::curaengine::types::EGCodeFlavor::ULTIGCODE;
    case "Makerbot"_sw:
        return ::curaengine::types::EGCodeFlavor::MAKERBOT;
    case "BFB"_sw:
        return ::curaengine::types::EGCodeFlavor::BFB;
    case "MACH3"_sw:
        return ::curaengine::types::EGCodeFlavor::MACH3;
    case "RepRap (Volumetric)"_sw:
        return ::curaengine::types::EGCodeFlavor::MARLIN_VOLUMATRIC;
    case "Repetier"_sw:
        return ::curaengine::types::EGCodeFlavor::REPETIER;
    case "RepRap (RepRap)"_sw:
        return ::curaengine::types::EGCodeFlavor::REPRAP;
    case "plugin"_sw:
        return ::curaengine::types::EGCodeFlavor::PLUGIN;
    default:
        return ::curaengine::types::EGCodeFlavor::MARLIN;
    }
}

template<>
::curaengine::types::EFillMethod Settings::get<::curaengine::types::EFillMethod>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "none"_sw:
        return ::curaengine::types::EFillMethod::NONE;
    case "lines"_sw:
        return ::curaengine::types::EFillMethod::LINES;
    case "grid"_sw:
        return ::curaengine::types::EFillMethod::GRID;
    case "cubic"_sw:
        return ::curaengine::types::EFillMethod::CUBIC;
    case "cubicsubdiv"_sw:
        return ::curaengine::types::EFillMethod::CUBICSUBDIV;
    case "tetrahedral"_sw:
        return ::curaengine::types::EFillMethod::TETRAHEDRAL;
    case "quarter_cubic"_sw:
        return ::curaengine::types::EFillMethod::QUARTER_CUBIC;
    case "triangles"_sw:
        return ::curaengine::types::EFillMethod::TRIANGLES;
    case "trihexagon"_sw:
        return ::curaengine::types::EFillMethod::TRIHEXAGON;
    case "concentric"_sw:
        return ::curaengine::types::EFillMethod::CONCENTRIC;
    case "zigzag"_sw:
        return ::curaengine::types::EFillMethod::ZIG_ZAG;
    case "cross"_sw:
        return ::curaengine::types::EFillMethod::CROSS;
    case "cross_3d"_sw:
        return ::curaengine::types::EFillMethod::CROSS_3D;
    case "gyroid"_sw:
        return ::curaengine::types::EFillMethod::GYROID;
    case "lightning"_sw:
        return ::curaengine::types::EFillMethod::LIGHTNING;
    case "honeycomb"_sw:
        return ::curaengine::types::EFillMethod::HONEYCOMB;
    case "octagon"_sw:
        return ::curaengine::types::EFillMethod::OCTAGON;
    case "plugin"_sw:
        return ::curaengine::types::EFillMethod::PLUGIN;
    default:
        return ::curaengine::types::EFillMethod::NONE;
    }
}

template<>
::curaengine::types::EPlatformAdhesion Settings::get<::curaengine::types::EPlatformAdhesion>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "skirt"_sw:
        return ::curaengine::types::EPlatformAdhesion::SKIRT;
    case "brim"_sw:
        return ::curaengine::types::EPlatformAdhesion::BRIM;
    case "raft"_sw:
        return ::curaengine::types::EPlatformAdhesion::RAFT;
    case "none"_sw:
        return ::curaengine::types::EPlatformAdhesion::NONE;
    case "plugin"_sw:
        return ::curaengine::types::EPlatformAdhesion::PLUGIN;
    default:
        return ::curaengine::types::EPlatformAdhesion::SKIRT;
    }
}

template<>
::curaengine::types::ESupportType Settings::get<::curaengine::types::ESupportType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "none"_sw:
        return ::curaengine::types::ESupportType::NONE;
    case "everywhere"_sw:
        return ::curaengine::types::ESupportType::EVERYWHERE;
    case "buildplate"_sw:
        return ::curaengine::types::ESupportType::PLATFORM_ONLY;
    case "plugin"_sw:
        return ::curaengine::types::ESupportType::PLUGIN;
    default:
        return ::curaengine::types::ESupportType::NONE;
    }
}

template<>
::curaengine::types::ESupportStructure Settings::get<::curaengine::types::ESupportStructure>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "normal"_sw:
        return ::curaengine::types::ESupportStructure::NORMAL;
    case "tree"_sw:
        return ::curaengine::types::ESupportStructure::TREE;
    case "plugin"_sw:
        return ::curaengine::types::ESupportStructure::PLUGIN;
    default:
        return ::curaengine::types::ESupportStructure::NORMAL;
    }
}


template<>
::curaengine::types::EZSeamType Settings::get<::curaengine::types::EZSeamType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "shortest"_sw:
        return ::curaengine::types::EZSeamType::SHORTEST;
    case "random"_sw:
        return ::curaengine::types::EZSeamType::RANDOM;
    case "back"_sw:
        return ::curaengine::types::EZSeamType::USER_SPECIFIED;
    case "sharpest_corner"_sw:
        return ::curaengine::types::EZSeamType::SHARPEST_CORNER;
    case "plugin"_sw:
        return ::curaengine::types::EZSeamType::PLUGIN;
    default:
        return ::curaengine::types::EZSeamType::SHORTEST;
    }
}

template<>
::curaengine::types::EZSeamCornerPrefType Settings::get<::curaengine::types::EZSeamCornerPrefType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "z_seam_corner_outer"_sw:
        return ::curaengine::types::EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER;
    case "z_seam_corner_any"_sw:
        return ::curaengine::types::EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY;
    case "z_seam_corner_weighted"_sw:
        return ::curaengine::types::EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED;
    case "plugin"_sw:
        return ::curaengine::types::EZSeamCornerPrefType::PLUGIN;
    case "z_seam_corner_inner"_sw:
    default:
        return ::curaengine::types::EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER;
    }
}

template<>
::curaengine::types::ESurfaceMode Settings::get<::curaengine::types::ESurfaceMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "normal"_sw:
        return ::curaengine::types::ESurfaceMode::NORMAL;
    case "surface"_sw:
        return ::curaengine::types::ESurfaceMode::SURFACE;
    case "both"_sw:
        return ::curaengine::types::ESurfaceMode::BOTH;
    case "plugin"_sw:
        return ::curaengine::types::ESurfaceMode::PLUGIN;
    default:
        return ::curaengine::types::ESurfaceMode::NORMAL;
    }
}

template<>
::curaengine::types::FillPerimeterGapMode Settings::get<::curaengine::types::FillPerimeterGapMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "nowhere"_sw:
        return ::curaengine::types::FillPerimeterGapMode::NOWHERE;
    case "everywhere"_sw:
        return ::curaengine::types::FillPerimeterGapMode::EVERYWHERE;
    case "plugin"_sw:
        return ::curaengine::types::FillPerimeterGapMode::PLUGIN;
    default:
        return ::curaengine::types::FillPerimeterGapMode::NOWHERE;
    }
}

template<>
::curaengine::types::BuildPlateShape Settings::get<::curaengine::types::BuildPlateShape>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "rectangular"_sw:
        return ::curaengine::types::BuildPlateShape::RECTANGULAR;
    case "elliptic"_sw:
        return ::curaengine::types::BuildPlateShape::ELLIPTIC;
    case "plugin"_sw:
        return ::curaengine::types::BuildPlateShape::PLUGIN;
    default:
        return ::curaengine::types::BuildPlateShape::RECTANGULAR;
    }
}

template<>
::curaengine::types::CombingMode Settings::get<::curaengine::types::CombingMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "all"_sw:
        return ::curaengine::types::CombingMode::ALL;
    case "off"_sw:
        return ::curaengine::types::CombingMode::OFF;
    case "noskin"_sw:
        return ::curaengine::types::CombingMode::NO_SKIN;
    case "no_outer_surfaces"_sw:
        return ::curaengine::types::CombingMode::NO_OUTER_SURFACES;
    case "infill"_sw:
        return ::curaengine::types::CombingMode::INFILL;
    case "plugin"_sw:
        return ::curaengine::types::CombingMode::PLUGIN;
    default:
        return ::curaengine::types::CombingMode::ALL;
    }
}

template<>
::curaengine::types::SupportDistPriority Settings::get<::curaengine::types::SupportDistPriority>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "xy_overrides_z"_sw:
        return ::curaengine::types::SupportDistPriority::XY_OVERRIDES_Z;
    case "z_overrides_xy"_sw:
        return ::curaengine::types::SupportDistPriority::Z_OVERRIDES_XY;
    case "plugin"_sw:
        return ::curaengine::types::SupportDistPriority::PLUGIN;
    default:
        return ::curaengine::types::SupportDistPriority::XY_OVERRIDES_Z;
    }
}

template<>
::curaengine::types::SlicingTolerance Settings::get<::curaengine::types::SlicingTolerance>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "middle"_sw:
        return ::curaengine::types::SlicingTolerance::MIDDLE;
    case "inclusive"_sw:
        return ::curaengine::types::SlicingTolerance::INCLUSIVE;
    case "exclusive"_sw:
        return ::curaengine::types::SlicingTolerance::EXCLUSIVE;
    case "plugin"_sw:
        return ::curaengine::types::SlicingTolerance::PLUGIN;
    default:
        return ::curaengine::types::SlicingTolerance::MIDDLE;
    }
}

template<>
::curaengine::types::InsetDirection Settings::get<::curaengine::types::InsetDirection>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using curaengine::types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "inside_out"_sw:
        return ::curaengine::types::InsetDirection::INSIDE_OUT;
    case "outside_in"_sw:
        return ::curaengine::types::InsetDirection::OUTSIDE_IN;
    case "plugin"_sw:
        return ::curaengine::types::InsetDirection::PLUGIN;
    default:
        return ::curaengine::types::InsetDirection::INSIDE_OUT;
    }
}

template<>
::curaengine::types::PrimeTowerMode Settings::get<::curaengine::types::PrimeTowerMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "interleaved")
    {
        return ::curaengine::types::PrimeTowerMode::INTERLEAVED;
    }

    return ::curaengine::types::PrimeTowerMode::NORMAL;
}

template<>
::curaengine::types::BrimLocation Settings::get<::curaengine::types::BrimLocation>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "everywhere")
    {
        return ::curaengine::types::BrimLocation::EVERYWHERE;
    }
    else if (value == "inside")
    {
        return ::curaengine::types::BrimLocation::INSIDE;
    }
    else // Default.
    {
        return ::curaengine::types::BrimLocation::OUTSIDE;
    }
}

template<>
::curaengine::types::CoolDuringExtruderSwitch Settings::get<::curaengine::types::CoolDuringExtruderSwitch>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "all_fans")
    {
        return ::curaengine::types::CoolDuringExtruderSwitch::ALL_FANS;
    }
    else if (value == "only_last_extruder")
    {
        return ::curaengine::types::CoolDuringExtruderSwitch::ONLY_LAST_EXTRUDER;
    }
    else // Default.
    {
        return ::curaengine::types::CoolDuringExtruderSwitch::UNCHANGED;
    }
}

template<>
::curaengine::types::InfillStartEndPreference Settings::get<::curaengine::types::InfillStartEndPreference>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "end_close_to_seam")
    {
        return ::curaengine::types::InfillStartEndPreference::END_CLOSE_TO_SEAM;
    }
    else if (value == "start_random")
    {
        return ::curaengine::types::InfillStartEndPreference::START_RANDOM;
    }
    else // Default.
    {
        return ::curaengine::types::InfillStartEndPreference::START_CLOSEST;
    }
}

template<>
::curaengine::types::RetractBeforeOuterWall Settings::get<::curaengine::types::RetractBeforeOuterWall>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "force_retracted")
    {
        return ::curaengine::types::RetractBeforeOuterWall::RETRACTED;
    }
    else if (value == "force_not_retracted")
    {
        return ::curaengine::types::RetractBeforeOuterWall::NOT_RETRACTED;
    }
    else if (value == "force_not_retracted_from_infill")
    {
        return ::curaengine::types::RetractBeforeOuterWall::NOT_RETRACTED_FROM_INFILL;
    }
    else // Default.
    {
        return ::curaengine::types::RetractBeforeOuterWall::AUTOMATIC;
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
std::vector<::curaengine::types::Ratio> Settings::get<std::vector<::curaengine::types::Ratio>>(const std::string& key) const
{
    auto values_double = get<std::vector<double>>(key);

    // The settings are all in percentages, but we need to interpret them as ratios.
    std::vector<::curaengine::types::Ratio> result;
    result.reserve(values_double.size());
    for (const auto& value_double : values_double)
    {
        result.emplace_back(value_double / 100.0);
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
std::vector<::curaengine::types::AngleDegrees> Settings::get<std::vector<::curaengine::types::AngleDegrees>>(const std::string& key) const
{
    std::vector<double> values_doubles = get<std::vector<double>>(key);
    return std::vector<::curaengine::types::AngleDegrees>(values_doubles.begin(), values_doubles.end()); // Cast them to AngleDegrees.
}

const std::string Settings::getAllSettingsString() const
{
    std::stringstream sstream;
    for (const std::pair<std::string, std::string> pair : settings)
    {
        char buffer[4096];
        snprintf(buffer, 4096, " -s %s=\"%s\"", pair.first.c_str(), ::curaengine::utils::Escaped{ pair.second.c_str() }.str);
        sstream << buffer;
    }
    return sstream.str();
}

bool Settings::has(const std::string& key, const bool parent_lookup) const
{
    const bool has_key = settings.contains(key);
    if (has_key || ! parent_lookup)
    {
        return has_key;
    }

    const std::unordered_map<std::string, ExtruderTrain*>& limit_to_extruder = Application::getInstance().current_slice_->scene.limit_to_extruder;
    if (limit_to_extruder.find(key) != limit_to_extruder.end())
    {
        return limit_to_extruder.at(key)->settings_.has(key, parent_lookup);
    }

    if (parent)
    {
        return parent->has(key, parent_lookup);
    }

    return false;
}

void Settings::setParent(const Settings* new_parent)
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

} // namespace curaengine
