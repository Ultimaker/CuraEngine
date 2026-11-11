// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "Settings.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <regex>
#include <sstream>
#include <string>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/map.hpp>
#include <spdlog/spdlog.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "types/Acceleration.h"
#include "types/AngleDegrees.h"
#include "types/AngleRadians.h"
#include "types/BrimLocation.h"
#include "types/BuildPlateShape.h"
#include "types/CombingMode.h"
#include "types/CoolDuringExtruderSwitch.h"
#include "types/DraftShieldHeightLimitation.h"
#include "types/Duration.h"
#include "types/EFillMethod.h"
#include "types/EGCodeFlavor.h"
#include "types/EPlatformAdhesion.h"
#include "types/ESupportStructure.h"
#include "types/ESupportType.h"
#include "types/ESurfaceMode.h"
#include "types/EZSeamCornerPrefType.h"
#include "types/EZSeamType.h"
#include "types/Escaped.h"
#include "types/FillPerimeterGapMode.h"
#include "types/InfillStartEndPreference.h"
#include "types/InsetDirection.h"
#include "types/LayerIndex.h"
#include "types/PrimeTowerMode.h"
#include "types/Ratio.h"
#include "types/RetractBeforeOuterWall.h"
#include "types/SlicingTolerance.h"
#include "types/SupportDistPriority.h"
#include "types/Temperature.h"
#include "types/Velocity.h"
#include "types/geometry/Matrix4x3D.h"
#include "types/geometry/Polygon.h"
#include "types/geometry/Shape.h"
#include "types/string_switch.h"
#include "utils/beading_strategy_factory.h"

namespace curaengine
{
using namespace types::geometry;
using namespace types;
using namespace utils;

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
LayerIndex Settings::get<LayerIndex>(const std::string& key) const
{
    // For the user we display layer numbers starting from 1, but we start counting from 0. Still it may be negative for Raft layers.
    return std::atoi(get<std::string>(key).c_str()) - 1;
}

template<>
coord_t Settings::get<coord_t>(const std::string& key) const
{
    return mmToInt(get<double>(key)); // The settings are all in millimetres, but we need to interpret them as microns.
}

template<>
AngleRadians Settings::get<AngleRadians>(const std::string& key) const
{
    return get<double>(key) * std::numbers::pi / 180; // The settings are all in degrees, but we need to interpret them as radians.
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
    return get<double>(key) / 100.0; // The settings are all in percentages, but we need to interpret them as ratios.
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
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "full"_sw:
        return DraftShieldHeightLimitation::Full;
    case "limited"_sw:
        return DraftShieldHeightLimitation::Limited;
    case "plugin"_sw:
        return DraftShieldHeightLimitation::Plugin;
    default:
        return DraftShieldHeightLimitation::Full;
    }
}

template<>
Shape Settings::get<Shape>(const std::string& key) const
{
    std::string value_string = get<std::string>(key);

    Shape result;
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

            Polygon& poly = result.newLine();

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
                    poly.emplace_back(mmToInt(first), mmToInt(second));
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
Matrix4x3D Settings::get<Matrix4x3D>(const std::string& key) const
{
    const std::string value_string = get<std::string>(key);

    Matrix4x3D result;
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
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "Marlin"_sw:
        return EGCodeFlavor::Marlin;
    case "Griffin"_sw:
        return EGCodeFlavor::Griffin;
    case "Cheetah"_sw:
        return EGCodeFlavor::Cheetah;
    case "UltiGCode"_sw:
        return EGCodeFlavor::UltiGCode;
    case "Makerbot"_sw:
        return EGCodeFlavor::MakerBot;
    case "BFB"_sw:
        return EGCodeFlavor::Bfb;
    case "MACH3"_sw:
        return EGCodeFlavor::Mach3;
    case "RepRap (Volumetric)"_sw:
        return EGCodeFlavor::MarlinVolumatric;
    case "Repetier"_sw:
        return EGCodeFlavor::Repetier;
    case "RepRap (RepRap)"_sw:
        return EGCodeFlavor::RepRap;
    case "plugin"_sw:
        return EGCodeFlavor::Plugin;
    default:
        return EGCodeFlavor::Marlin;
    }
}

template<>
EFillMethod Settings::get<EFillMethod>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "none"_sw:
        return EFillMethod::None;
    case "lines"_sw:
        return EFillMethod::Lines;
    case "grid"_sw:
        return EFillMethod::Grid;
    case "cubic"_sw:
        return EFillMethod::Cubic;
    case "cubicsubdiv"_sw:
        return EFillMethod::CubicSubdiv;
    case "tetrahedral"_sw:
        return EFillMethod::Tetrahedral;
    case "quarter_cubic"_sw:
        return EFillMethod::QuarterCubic;
    case "triangles"_sw:
        return EFillMethod::Triangles;
    case "trihexagon"_sw:
        return EFillMethod::TriHexagon;
    case "concentric"_sw:
        return EFillMethod::Concentric;
    case "zigzag"_sw:
        return EFillMethod::ZigZag;
    case "cross"_sw:
        return EFillMethod::Cross;
    case "cross_3d"_sw:
        return EFillMethod::Cross3D;
    case "gyroid"_sw:
        return EFillMethod::Gyroid;
    case "lightning"_sw:
        return EFillMethod::Lightning;
    case "honeycomb"_sw:
        return EFillMethod::Honeycomb;
    case "octagon"_sw:
        return EFillMethod::Octagon;
    case "plugin"_sw:
        return EFillMethod::Plugin;
    default:
        return EFillMethod::None;
    }
}

template<>
EPlatformAdhesion Settings::get<EPlatformAdhesion>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "skirt"_sw:
        return EPlatformAdhesion::Skirt;
    case "brim"_sw:
        return EPlatformAdhesion::Brim;
    case "raft"_sw:
        return EPlatformAdhesion::Raft;
    case "none"_sw:
        return EPlatformAdhesion::None;
    case "plugin"_sw:
        return EPlatformAdhesion::Plugin;
    default:
        return EPlatformAdhesion::Skirt;
    }
}

template<>
ESupportType Settings::get<ESupportType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "none"_sw:
        return ESupportType::None;
    case "everywhere"_sw:
        return ESupportType::Everywhere;
    case "buildplate"_sw:
        return ESupportType::PlatformOnly;
    case "plugin"_sw:
        return ESupportType::Plugin;
    default:
        return ESupportType::None;
    }
}

template<>
ESupportStructure Settings::get<ESupportStructure>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "normal"_sw:
        return ESupportStructure::Normal;
    case "tree"_sw:
        return ESupportStructure::Tree;
    case "plugin"_sw:
        return ESupportStructure::Plugin;
    default:
        return ESupportStructure::Normal;
    }
}


template<>
EZSeamType Settings::get<EZSeamType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "shortest"_sw:
        return EZSeamType::Shortest;
    case "random"_sw:
        return EZSeamType::Random;
    case "back"_sw:
        return EZSeamType::UserSpecified;
    case "sharpest_corner"_sw:
        return EZSeamType::SharpestCorner;
    case "plugin"_sw:
        return EZSeamType::Plugin;
    default:
        return EZSeamType::Shortest;
    }
}

template<>
EZSeamCornerPrefType Settings::get<EZSeamCornerPrefType>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "z_seam_corner_outer"_sw:
        return EZSeamCornerPrefType::ZSeamCornerPrefOuter;
    case "z_seam_corner_any"_sw:
        return EZSeamCornerPrefType::ZSeamCornerPrefAny;
    case "z_seam_corner_weighted"_sw:
        return EZSeamCornerPrefType::ZSeamCornerPrefWeighted;
    case "plugin"_sw:
        return EZSeamCornerPrefType::Plugin;
    case "z_seam_corner_inner"_sw:
    default:
        return EZSeamCornerPrefType::ZSeamCornerPrefInner;
    }
}

template<>
ESurfaceMode Settings::get<ESurfaceMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "normal"_sw:
        return ESurfaceMode::Normal;
    case "surface"_sw:
        return ESurfaceMode::Surface;
    case "both"_sw:
        return ESurfaceMode::Both;
    case "plugin"_sw:
        return ESurfaceMode::Plugin;
    default:
        return ESurfaceMode::Normal;
    }
}

template<>
FillPerimeterGapMode Settings::get<FillPerimeterGapMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "nowhere"_sw:
        return FillPerimeterGapMode::Nowhere;
    case "everywhere"_sw:
        return FillPerimeterGapMode::Everywhere;
    case "plugin"_sw:
        return FillPerimeterGapMode::Plugin;
    default:
        return FillPerimeterGapMode::Nowhere;
    }
}

template<>
BuildPlateShape Settings::get<BuildPlateShape>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "rectangular"_sw:
        return BuildPlateShape::Rectangular;
    case "elliptic"_sw:
        return BuildPlateShape::Elliptic;
    case "plugin"_sw:
        return BuildPlateShape::Plugin;
    default:
        return BuildPlateShape::Rectangular;
    }
}

template<>
CombingMode Settings::get<CombingMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "all"_sw:
        return CombingMode::All;
    case "off"_sw:
        return CombingMode::Off;
    case "noskin"_sw:
        return CombingMode::NoSkin;
    case "no_outer_surfaces"_sw:
        return CombingMode::NoOuterSurfaces;
    case "infill"_sw:
        return CombingMode::Infill;
    case "plugin"_sw:
        return CombingMode::Plugin;
    default:
        return CombingMode::All;
    }
}

template<>
SupportDistPriority Settings::get<SupportDistPriority>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "xy_overrides_z"_sw:
        return SupportDistPriority::XyOverridesZ;
    case "z_overrides_xy"_sw:
        return SupportDistPriority::ZOverridesXy;
    case "plugin"_sw:
        return SupportDistPriority::Plugin;
    default:
        return SupportDistPriority::XyOverridesZ;
    }
}

template<>
SlicingTolerance Settings::get<SlicingTolerance>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "middle"_sw:
        return SlicingTolerance::Middle;
    case "inclusive"_sw:
        return SlicingTolerance::Inclusive;
    case "exclusive"_sw:
        return SlicingTolerance::Exclusive;
    case "plugin"_sw:
        return SlicingTolerance::Plugin;
    default:
        return SlicingTolerance::Middle;
    }
}

template<>
InsetDirection Settings::get<InsetDirection>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    using types::operator""_sw;
    switch (types::hash_enum(value))
    {
    case "inside_out"_sw:
        return InsetDirection::InsideOut;
    case "outside_in"_sw:
        return InsetDirection::OutsideIn;
    case "plugin"_sw:
        return InsetDirection::Plugin;
    default:
        return InsetDirection::InsideOut;
    }
}

template<>
PrimeTowerMode Settings::get<PrimeTowerMode>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "interleaved")
    {
        return PrimeTowerMode::Interleaved;
    }

    return PrimeTowerMode::Normal;
}

template<>
BrimLocation Settings::get<BrimLocation>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "everywhere")
    {
        return BrimLocation::Everywhere;
    }
    else if (value == "inside")
    {
        return BrimLocation::Inside;
    }
    else // Default.
    {
        return BrimLocation::Outside;
    }
}

template<>
CoolDuringExtruderSwitch Settings::get<CoolDuringExtruderSwitch>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "all_fans")
    {
        return CoolDuringExtruderSwitch::AllFans;
    }
    else if (value == "only_last_extruder")
    {
        return CoolDuringExtruderSwitch::OnlyLastExtruder;
    }
    else // Default.
    {
        return CoolDuringExtruderSwitch::Unchanged;
    }
}

template<>
InfillStartEndPreference Settings::get<InfillStartEndPreference>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "end_close_to_seam")
    {
        return InfillStartEndPreference::EndCloseToSeam;
    }
    else if (value == "start_random")
    {
        return InfillStartEndPreference::StartRandom;
    }
    else // Default.
    {
        return InfillStartEndPreference::StartClosest;
    }
}

template<>
RetractBeforeOuterWall Settings::get<RetractBeforeOuterWall>(const std::string& key) const
{
    const std::string& value = get<std::string>(key);
    if (value == "force_retracted")
    {
        return RetractBeforeOuterWall::Retracted;
    }
    else if (value == "force_not_retracted")
    {
        return RetractBeforeOuterWall::NotRetracted;
    }
    else if (value == "force_not_retracted_from_infill")
    {
        return RetractBeforeOuterWall::NotRetractedFromInfill;
    }
    else // Default.
    {
        return RetractBeforeOuterWall::Automatic;
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
std::vector<Ratio> Settings::get<std::vector<Ratio>>(const std::string& key) const
{
    auto values_double = get<std::vector<double>>(key);

    // The settings are all in percentages, but we need to interpret them as ratios.
    std::vector<Ratio> result;
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
