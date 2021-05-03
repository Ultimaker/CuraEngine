// Copyright (c) 2019 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TreeSupport.h"
#include "Application.h" //To get settings.
#include "infill.h"
#include "infill/SierpinskiFillProvider.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h"
#include "settings/types/AngleRadians.h" //Creating the correct branch angles.
#include "support.h" //For precomputeCrossInfillTree
#include "utils/logoutput.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/polygonUtils.h" //For moveInside.
#include <chrono>
#include <fstream>
#include <omp.h>
#include <optional>
#include <stdio.h>
#include <string>

#define SUPPORT_TREE_CIRCLE_RESOLUTION 25 // The number of vertices in each circle.

// The various stages of the process can be weighted differently in the progress bar.
// These weights are obtained experimentally using a small sample size. Sensible weights can differ drastically based on the assumed default settings and model.
#define PROGRESS_TOTAL 10000
#define PROGRESS_PRECALC_COLL PROGRESS_TOTAL * 0.1
#define PROGRESS_PRECALC_AVO PROGRESS_TOTAL * 0.4
#define PROGRESS_GENERATE_NODES PROGRESS_TOTAL * 0.1
#define PROGRESS_AREA_CALC PROGRESS_TOTAL * 0.3
#define PROGRESS_DRAW_AREAS PROGRESS_TOTAL * 0.1


#define SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL false
#define SUPPORT_TREE_AVOID_SUPPORT_BLOCKER true
#define SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION true
#define SUPPORT_TREE_EXPONENTIAL_THRESHOLD 1000
#define SUPPORT_TREE_EXPONENTIAL_FACTOR 1.5
#define SUPPORT_TREE_COLLISION_RESOLUTION 500
#define SUPPORT_TREE_MAX_DEVIATION 0

namespace cura
{
bool TreeSupport::TreeSupportSettings::some_model_contains_thick_roof = false; // Explicit zero-initialization, because while a static variable should be always zero-initialized, the linker complains about "undefined reference" otherwise...

TreeSupport::TreeSupport(const SliceDataStorage& storage)
{
    // Group all meshes that can be processed together. NOTE this is different from mesh-groups! Only one setting object is needed per group, as different settings in the same group may only occur in the tip, which uses the original settings objects from the meshes.
    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];

        const bool non_supportable_mesh = mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh") || storage.meshes[mesh_idx].settings.get<bool>("support_mesh");
        if (storage.meshes[mesh_idx].settings.get<ESupportStructure>("support_structure") != ESupportStructure::TREE || non_supportable_mesh)
        {
            continue;
        }

        bool added = false;
        TreeSupportSettings next_settings(mesh.settings);

        if (mesh.settings.get<coord_t>("support_roof_height") >= 2 * next_settings.layer_height)
        {
            TreeSupportSettings::some_model_contains_thick_roof = true;
        }

        for (size_t idx = 0; idx < grouped_meshes.size(); idx++)
        {
            if (next_settings == grouped_meshes[idx].first)
            {
                added = true;
                grouped_meshes[idx].second.emplace_back(mesh_idx);
                // handle some settings that are only used for performance reasons. This ensures that a horrible set setting intended to improve performance can not reduce it drastically.
                grouped_meshes[idx].first.performance_interface_skip_layers = std::min(grouped_meshes[idx].first.performance_interface_skip_layers, next_settings.performance_interface_skip_layers);
            }
        }
        if (!added)
        {
            grouped_meshes.emplace_back(next_settings, std::vector<size_t>{ mesh_idx });
        }
    }
}


// todo remove as only for debugging relevant
std::string getPolygonAsString(const Polygons& poly)
{
    std::string ret = "";
    for (auto path : poly)
    {
        for (Point p : path)
        {
            if (ret != "")
                ret += ",";
            ret += "(" + std::to_string(p.X) + "," + std::to_string(p.Y) + ")";
        }
    }
    return ret;
}


void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    if (grouped_meshes.empty())
    {
        return;
    }

    if (storage.support.cross_fill_provider == nullptr)
    {
        AreaSupport::precomputeCrossInfillTree(storage);
    }

    size_t counter = 0;

    // Process every mesh group. These groups can not be processed parallel, as the processing in each group is parallelized, and nested parallelization is disables and slow.
    for (std::pair<TreeSupportSettings, std::vector<size_t>> processing : grouped_meshes)
    {
        // process each combination of meshes
        std::vector<std::set<SupportElement*>> move_bounds(storage.support.supportLayers.size()); // value is the area where support may be placed. As this is calculated in CreateLayerPathing it is saved and reused in drawAreas
        log("Processing support tree mesh group %lld of %lld containing %lld meshes.\n", counter + 1, grouped_meshes.size(), grouped_meshes[counter].second.size());
        std::vector<Polygons> exclude(storage.support.supportLayers.size());
        auto t_start = std::chrono::high_resolution_clock::now();
        // get all already existing support areas and exclude them
#pragma omp parallel for
        for (LayerIndex layer_idx = 0; layer_idx < coord_t(storage.support.supportLayers.size()); layer_idx++)
        {
            Polygons exlude_at_layer;
            exlude_at_layer.add(storage.support.supportLayers[layer_idx].support_bottom);
            exlude_at_layer.add(storage.support.supportLayers[layer_idx].support_roof);
            for (auto part : storage.support.supportLayers[layer_idx].support_infill_parts)
            {
                exlude_at_layer.add(part.outline);
            }
            exclude[layer_idx] = exlude_at_layer.unionPolygons();
        }
        config = processing.first; // this struct is used to easy retrieve setting. No other function except those in ModelVolumes and generateInitalAreas have knowledge of the existence of multiple meshes being processed.
        progress_multiplier = 1.0 / double(grouped_meshes.size());
        progress_offset = counter == 0 ? 0 : PROGRESS_TOTAL * (double(counter) * progress_multiplier);
        volumes_ = ModelVolumes(storage, config.maximum_move_distance, config.maximum_move_distance_slow, processing.second.front(), progress_multiplier, progress_offset, exclude);

        // ### Precalculate avoidances, collision etc.
        precalculate(storage, processing.second);
        auto t_precalc = std::chrono::high_resolution_clock::now();

        // ### Place tips of the support tree
        for (size_t mesh_idx : processing.second)
        {
            generateInitalAreas(storage.meshes[mesh_idx], move_bounds, storage);
        }
        auto t_gen = std::chrono::high_resolution_clock::now();

        // ### Propagate the influence areas downwards.
        createLayerPathing(move_bounds);
        auto t_path = std::chrono::high_resolution_clock::now();

        // ### Set a point in each influence area
        createNodesFromArea(move_bounds);
        auto t_place = std::chrono::high_resolution_clock::now();

        // ### draw these points as circles
        drawAreas(move_bounds, storage);

        auto t_draw = std::chrono::high_resolution_clock::now();
        auto dur_pre_gen = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_precalc - t_start).count();
        auto dur_gen = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_gen - t_precalc).count();
        auto dur_path = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_path - t_gen).count();
        auto dur_place = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_place - t_path).count();
        auto dur_draw = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_draw - t_place).count();
        auto dur_total = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_draw - t_start).count();
        log("Total time used creating Tree support for the currently grouped meshes: %.3lf ms. Different subtasks:\nCalculating Avoidance: %.3lf ms Creating inital influence areas: %.3lf ms Influence area creation: %.3lf ms Placement of Points in InfluenceAreas: %.3lf ms Drawing result as support %.3lf ms\n", dur_total, dur_pre_gen, dur_gen, dur_path, dur_place, dur_draw);

        for (auto& layer : move_bounds)
        {
            for (auto elem : layer)
            {
                delete elem->area;
                delete elem;
            }
        }

        counter++;
    }

    storage.support.generated = true;
}


void TreeSupport::precalculate(const SliceDataStorage& storage, std::vector<size_t> currently_processing_meshes)
{
    // calculate top most layer that is relevant for support
    LayerIndex max_layer = 0;
    for (size_t mesh_idx : currently_processing_meshes)
    {
        const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
        const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
        const size_t z_distance_top_layers = round_up_divide(z_distance_top,
                                                             layer_height) + 1; // Support must always be 1 layer below overhang.
        if (mesh.overhang_areas.size() <= z_distance_top_layers)
        {
            continue;
        }
        for (LayerIndex layer_idx = (mesh.overhang_areas.size() - z_distance_top_layers) - 1; layer_idx != 0; layer_idx--)
        {
            // look for max relevant layer
            const Polygons& overhang = mesh.overhang_areas[layer_idx + z_distance_top_layers];
            if (!overhang.empty())
            {
                if (layer_idx > max_layer) // iterates over multiple meshes
                {
                    max_layer = 1 + layer_idx; // plus one to avoid problems if something is of by one
                }
                break;
            }
        }
    }

    // ### The actual precalculation happens in ModelVolumes.
    volumes_.precalculate(max_layer);
}


std::vector<TreeSupport::LineInformation> TreeSupport::convertLinesToInternal(Polygons polylines, LayerIndex layer_idx)
{
    const bool xy_overrides = config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z;

    std::vector<LineInformation> result;
    // Also checks if the position is valid, if it is NOT, it deletes that point
    for (auto line : polylines)
    {
        LineInformation res_line;
        for (Point p : line)
        {
            if (!volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST_SAFE, false, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_BP_SAFE);
            }
            else if (!volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST, false, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_BP);
            }
            else if (config.support_rests_on_model && !volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST_SAFE, true, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS_SAFE);
            }
            else if (config.support_rests_on_model && !volumes_.getAvoidance(config.getRadius(0), layer_idx, AvoidanceType::FAST, true, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL_GRACIOUS);
            }
            else if (config.support_rests_on_model && !volumes_.getCollision(config.getRadius(0), layer_idx, !xy_overrides).inside(p, true))
            {
                res_line.emplace_back(p, LineStatus::TO_MODEL);
            }
            else
            {
                if (!res_line.empty())
                {
                    result.emplace_back(res_line);
                    res_line.clear();
                }
            }
        }
        if (!res_line.empty())
        {
            result.emplace_back(res_line);
            res_line.clear();
        }
    }

    return result;
}

Polygons TreeSupport::convertInternalToLines(std::vector<TreeSupport::LineInformation> lines)
{
    Polygons result;

    for (LineInformation line : lines)
    {
        Polygon path;
        for (auto point_data : line)
        {
            path.add(point_data.first);
        }
        result.add(path);
    }
    return result;
}


std::function<bool(std::pair<Point, TreeSupport::LineStatus>)> TreeSupport::getEvaluatePointForNextLayerFunction(size_t current_layer)
{
    const bool xy_overrides = config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z;
    std::function<bool(std::pair<Point, LineStatus>)> evaluatePoint = [=](std::pair<Point, LineStatus> p) {
        if (!volumes_.getAvoidance(config.getRadius(0), current_layer - 1, p.second == LineStatus::TO_BP_SAFE ? AvoidanceType::FAST_SAFE : AvoidanceType::FAST, false, !xy_overrides).inside(p.first, true))
        {
            return true;
        }
        if (config.support_rests_on_model && (p.second != LineStatus::TO_BP && p.second != LineStatus::TO_BP_SAFE))
        {
            if (p.second == LineStatus::TO_MODEL_GRACIOUS || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE)
            {
                return !volumes_.getAvoidance(config.getRadius(0), current_layer - 1, p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE ? AvoidanceType::FAST_SAFE : AvoidanceType::FAST, true, !xy_overrides).inside(p.first, true);
            }
            else
            {
                return !volumes_.getCollision(config.getRadius(0), current_layer - 1, !xy_overrides).inside(p.first, true);
            }
        }
        return false;
    };
    return evaluatePoint;
}


std::pair<std::vector<TreeSupport::LineInformation>, std::vector<TreeSupport::LineInformation>> TreeSupport::splitLines(std::vector<TreeSupport::LineInformation> lines, std::function<bool(std::pair<Point, TreeSupport::LineStatus>)> evaluatePoint)
{
    // assumes all Points on the current line are valid

    std::vector<LineInformation> keep(1);
    std::vector<LineInformation> set_free(1);
    enum STATE
    {
        keeping,
        freeing
    };
    for (std::vector<std::pair<Point, LineStatus>> line : lines)
    {
        STATE current = keeping;
        LineInformation resulting_line;
        for (std::pair<Point, LineStatus> me : line)
        {
            if (evaluatePoint(me))
            {
                if (keeping != current)
                {
                    if (!resulting_line.empty())
                    {
                        set_free.emplace_back(resulting_line);
                        resulting_line.clear();
                    }
                    current = keeping;
                }
                resulting_line.emplace_back(me);
            }
            else
            {
                if (freeing != current)
                {
                    if (!resulting_line.empty())
                    {
                        keep.emplace_back(resulting_line);
                        resulting_line.clear();
                    }
                    current = freeing;
                }
                resulting_line.emplace_back(me);
            }
        }
        if (!resulting_line.empty())
        {
            if (current == keeping)
            {
                keep.emplace_back(resulting_line);
            }
            else
            {
                set_free.emplace_back(resulting_line);
            }
        }
    }
    return std::pair<std::vector<std::vector<std::pair<Point, TreeSupport::LineStatus>>>, std::vector<std::vector<std::pair<Point, TreeSupport::LineStatus>>>>(keep, set_free);
}


void writePolylines(SVG& svg, Polygons polylines, SVG::Color color) // todo remove as only for debugging relevant
{
    for (auto path : polylines)
    {
        if (path.size() == 0)
        {
            continue;
        }
        Point before = path[0];
        for (size_t i = 1; i < path.size(); i++)
        {
            svg.writeLine(before, path[i], color, 2);
            before = path[i];
        }
    }
}


// ensures that every line segment is about distance in length. The resulting lines may differ from the original but all points are on the original
Polygons TreeSupport::ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points) const
{
    Polygons result;
    for (auto part : input)
    {
        if (part.size() == 0)
        {
            continue;
        }
        coord_t length = Polygon(part).offset(0).polyLineLength();
        size_t points = length / distance;
        if (points < min_points)
        {
            points = min_points;
        }
        coord_t target_distance = length / points;
        Polygon line;
        if (points == 1)
        {
            target_distance = length / 2 + 10; // we will only have the point in the middle
        }
        else
        {
            line.add(part[0]);
        }
        coord_t used_distance = 0;
        for (size_t pos = 0; pos + 1 < part.size(); pos++)
        {
            Point me = part[(part.size() + pos) % part.size()];
            Point next = part[(part.size() + pos + 1) % part.size()];
            coord_t next_step_length;
            do
            {
                next_step_length = vSize(me - next);
                if (next_step_length + used_distance >= target_distance)
                {
                    double scale = (1.0 * (target_distance - used_distance)) / (next_step_length);
                    me = me + (next - me) * scale;
                    used_distance = 0;
                    line.add(me);
                    continue;
                }
                used_distance += next_step_length;
            } while (next_step_length > target_distance);
        }
        if (points > 1)
        {
            line.add(part.back());
        }
        result.add(line);
    }
    return result;
}

// adds the implizit line from the last vertex to the first explicitly
Polygons TreeSupport::toPolylines(const Polygons& poly) const
{
    Polygons result;
    for (auto path : poly)
    {
        Polygon part;
        for (size_t i = 0; i < path.size(); i++)
        {
            part.add(path[i]);
        }
        part.add(path[0]);
        result.add(part);
    }
    return result;
}


Polygons TreeSupport::generateSupportInfillLines(const Polygons& area, bool roof, LayerIndex layer_idx, coord_t support_infill_distance, SierpinskiFillProvider* cross_fill_provider)
{
    Polygons gaps;
    // as we effectivly use lines to place our supportPoints we may use the Infill class for it, while not made for it it works perfect


    const EFillMethod pattern = roof ? config.roof_pattern : config.support_pattern;

    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = false;
    constexpr coord_t support_roof_overlap = 0; // the roofs should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    constexpr coord_t outline_offset = 0;
    constexpr coord_t extra_infill_shift = 0;
    constexpr size_t wall_line_count = 0;
    const Point infill_origin;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr size_t zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    std::vector<AngleDegrees> angles = roof ? config.support_roof_angles : config.support_infill_angles;

    const coord_t z = layer_idx;
    int divisor = static_cast<int>(angles.size());
    int index = ((z % divisor) + divisor) % divisor;
    const AngleDegrees fill_angle = angles[index];
    Infill roof_computation(pattern, zig_zaggify_infill, connect_polygons, area, outline_offset, roof ? config.support_roof_line_width : config.support_line_width, support_infill_distance, support_roof_overlap, infill_multiplier, fill_angle, z, extra_infill_shift, config.maximum_resolution, config.maximum_deviation, wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
    Polygons areas;
    Polygons lines;
    roof_computation.generate(areas, lines, cross_fill_provider);
    lines.add(toPolylines(areas));
    return lines;
}

Polygons TreeSupport::safeUnion(const Polygons first, const Polygons second) const
{
    // unionPolygons can slowly remove Polygons under certain circumstances, because of rounding issues (Polygons that have a thin area).
    // This does not cause a problem when actually using it on large areas, but as influence areas (representing centerpoints) can be very thin, this does occur so this ugly workaround is needed
    // Here is an example of a Polygons object that will loose vertices when unioning, and will be gone after a few times unionPolygons was called:
    /*
    Polygons example;
    Polygon exampleInner;
    exampleInner.add(Point(120410,83599));//A
    exampleInner.add(Point(120384,83643));//B
    exampleInner.add(Point(120399,83618));//C
    exampleInner.add(Point(120414,83591));//D
    exampleInner.add(Point(120423,83570));//E
    exampleInner.add(Point(120419,83580));//F
    example.add(exampleInner);
    for(int i=0;i<10;i++){
         log("Iteration %d Example area: %f\n",i,example.area());
         example=example.unionPolygons();
    }
*/


    bool was_empty = first.empty() && second.empty();

    Polygons result = first.unionPolygons(second);

    if (result.empty() && !was_empty) // error occurred
    {
        logDebug("Caught an area destroying union, enlarging areas a bit.\n");
        return toPolylines(first).offsetPolyLine(2).unionPolygons(toPolylines(second).offsetPolyLine(2)); // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
    }

    else
    {
        return result;
    }
}

Polygons TreeSupport::safeUnion(const Polygons first) const
{
    return safeUnion(first, Polygons());
}


SierpinskiFillProvider* TreeSupport::generateCrossFillProvider(const SliceMeshStorage& mesh, coord_t line_distance, coord_t line_width)
{
    const EFillMethod& support_pattern = mesh.settings.get<EFillMethod>("support_pattern");
    if (support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D)
    {
        AABB3D aabb;
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            logWarning("Tree support tried to generate a CrossFillProvider for a non model mesh.\n");
            return nullptr;
        }

        const coord_t aabb_expansion = mesh.settings.get<coord_t>("support_offset");
        AABB3D aabb_here(mesh.bounding_box);
        aabb_here.include(aabb_here.min - Point3(-aabb_expansion, -aabb_expansion, 0));
        aabb_here.include(aabb_here.max + Point3(-aabb_expansion, -aabb_expansion, 0));
        aabb.include(aabb_here);

        std::string cross_subdisivion_spec_image_file = mesh.settings.get<std::string>("cross_support_density_image");
        std::ifstream cross_fs(cross_subdisivion_spec_image_file.c_str());
        if (cross_subdisivion_spec_image_file != "" && cross_fs.good())
        {
            return new SierpinskiFillProvider(aabb, line_distance, line_width, cross_subdisivion_spec_image_file);
        }
        else
        {
            return new SierpinskiFillProvider(aabb, line_distance, line_width);
        }
    }
    return nullptr;
}


void TreeSupport::generateInitalAreas(const SliceMeshStorage& mesh, std::vector<std::set<SupportElement*>>& move_bounds, SliceDataStorage& storage)
{
    Polygon base_circle;

    const int baseRadius = 10;
    for (unsigned int i = 0; i < SUPPORT_TREE_CIRCLE_RESOLUTION; i++)
    {
        const AngleRadians angle = static_cast<double>(i) / SUPPORT_TREE_CIRCLE_RESOLUTION * TAU;
        base_circle.emplace_back(cos(angle) * baseRadius, sin(angle) * baseRadius);
    }

    TreeSupportSettings mesh_config(mesh.settings);

    const size_t z_distance_delta = mesh_config.z_distance_top_layers + 1; // To ensure z_distance_top_layers are left empty between the overhang (zeroth empty layer), the support has to be added z_distance_top_layers+1 layers below
    const coord_t support_line_width = mesh_config.support_line_width;

    const coord_t support_roof_line_distance = mesh.settings.get<coord_t>("support_roof_line_distance");
    const double minimum_roof_area = mesh.settings.get<double>("minimum_roof_area");
    const double minimum_support_area = mesh.settings.get<double>("minimum_support_area"); // todo did i interpret this setting correctly, as in should minimum_support_area also guarantee that every tip has at least this area?
    const size_t support_roof_layers = mesh.settings.get<bool>("support_roof_enable") ? round_divide(mesh.settings.get<coord_t>("support_roof_height"), mesh_config.layer_height) : 0;
    const bool roof_enabled = support_roof_layers != 0;
    const bool only_gracious = SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL;
    const EFillMethod support_pattern = mesh.settings.get<EFillMethod>("support_pattern");
    const coord_t support_tree_branch_distance = (support_pattern == EFillMethod::TRIANGLES ? 3 : (support_pattern == EFillMethod::GRID ? 2 : 1)) * support_line_width * 100 / mesh.settings.get<double>("support_tree_top_rate");
    const coord_t connect_length = support_tree_branch_distance;
    const coord_t support_outset = (mesh_config.performance_increased_xy_min ? 100 : 0) + mesh.settings.get<coord_t>("support_offset");
    const coord_t roof_outset = (mesh_config.performance_increased_xy_min ? 100 : 0) + mesh.settings.get<coord_t>("support_roof_offset");

    const double support_overhang_angle = mesh.settings.get<AngleRadians>("support_angle");
    const coord_t max_overhang_speed = (support_overhang_angle < TAU / 4) ? (coord_t)(tan(support_overhang_angle) * mesh_config.layer_height) : std::numeric_limits<coord_t>::max();
    const size_t max_overhang_insert_lag = std::max((size_t)round_up_divide(mesh_config.xy_distance, max_overhang_speed / 2), 2 * mesh_config.z_distance_top_layers); // cap for how much layer below the overhang a new support point may be added, as other than with regular support every new inserted point may cause extra material and time cost.  Could also be an user setting or differently calculated. Idea is that if an overhang does not turn valid in double the amount of layers a slope of support angle would take to travel xy_distance, nothing reasonable will come from it. The 2*z_distance_delta is only a catch for when the support angle is very high.

    const bool xy_overrides = mesh_config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z;
    SierpinskiFillProvider* cross_fill_provider = generateCrossFillProvider(mesh, support_tree_branch_distance, mesh_config.support_line_width);

    if (mesh.overhang_areas.size() <= z_distance_delta)
    {
        return;
    }
    logDebug("Distance between points of lines of inital influence areas %lld \n", connect_length);
    std::vector<std::unordered_set<Point>> already_inserted(mesh.overhang_areas.size() - z_distance_delta);

    // counting down as it is more likely to encounter areas that need much support at the top then at the bottom. This could improve performance because of the schedule(dynamic)
#pragma omp parallel for schedule(dynamic)
    for (LayerIndex layer_idx = mesh.overhang_areas.size() - z_distance_delta - 1; layer_idx >= 1; layer_idx--)
    {
        if (mesh.overhang_areas[layer_idx + z_distance_delta].empty())
        {
            continue;
        }

        Polygons relevant_forbidden = (mesh_config.support_rests_on_model ? (only_gracious ? volumes_.getAvoidance(mesh_config.getRadius(0), layer_idx, AvoidanceType::FAST, true, !xy_overrides) : volumes_.getCollision(mesh_config.getRadius(0), layer_idx, !xy_overrides)) : volumes_.getAvoidance(mesh_config.getRadius(0), layer_idx, AvoidanceType::FAST, false, !xy_overrides)); // take the least restrictive avoidance possible

        std::function<Polygons(const Polygons&, bool, size_t)> generateLines = [&](const Polygons& area, bool roof, LayerIndex layer_idx) {
            const coord_t support_infill_distance = roof ? support_roof_line_distance : support_tree_branch_distance;
            return generateSupportInfillLines(area, roof, layer_idx, support_infill_distance, cross_fill_provider);
        };

        std::function<void(std::pair<Point, LineStatus>, size_t, coord_t, size_t, bool)> addPointAsInfluenceArea = [&](std::pair<Point, LineStatus> p, size_t dtt, LayerIndex insert_layer, size_t dont_move_until, bool roof) {
            bool to_bp = p.second == LineStatus::TO_BP || p.second == LineStatus::TO_BP_SAFE;
            bool gracious = to_bp || p.second == LineStatus::TO_MODEL_GRACIOUS || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
            bool safe_radius = p.second == LineStatus::TO_BP_SAFE || p.second == LineStatus::TO_MODEL_GRACIOUS_SAFE;
            if (!mesh_config.support_rests_on_model && !to_bp)
            {
                logWarning("Tried to add an invalid support point\n");
                return;
            }
            Polygon circle;
            for (Point corner : base_circle)
            {
                circle.add(p.first + corner);
            }
            Polygons area = circle.offset(0);
#pragma omp critical(moveBounds)
            {
                if (!already_inserted[insert_layer].count(p.first / ((support_line_width + 1) / 10)))
                {
                    // normalize the point a bit to also catch points which are so close that inserting it would achieve nothing
                    already_inserted[insert_layer].emplace(p.first / ((support_line_width + 1) / 10));
                    SupportElement* elem = new SupportElement(dtt, insert_layer, p.first, to_bp, gracious, !xy_overrides, dont_move_until, roof, safe_radius);
                    elem->area = new Polygons(area);
                    move_bounds[insert_layer].emplace(elem);
                }
            }
        };


        std::vector<std::pair<Polygons, bool>> overhang_processing; // every overhang has saved if a roof should be generated for it. This can NOT be done in the for loop as an area may NOT have a roof even if it is larger than the minimum_roof_area when it is only larger because of the support horizontal expansion and it would not have a roof if the overhang is offset by support roof horizontal expansion instead. (At least this is the current behavior of the regular support)

        // If the xy distance overrides the z distance, some support needs to be inserted further down.
        //=> Analyze which support points do not fit on this layer and check if they will fit a few layers down (while adding them an infinite amount of layers down would technically be closer the the setting description, it would not produce reasonable results. )
        if (xy_overrides)
        {
            Polygons removed_overhang = mesh.overhang_areas[layer_idx + z_distance_delta].offset(support_outset).intersection(relevant_forbidden).offset(0.5 * support_line_width); // offset to ensure area already supported to not be supported below again.
            std::vector<LineInformation> overhang_lines;
            Polygons polylines = ensureMaximumDistancePolyline(generateLines(removed_overhang, false, layer_idx), support_line_width, 1); // support_line_width to form a line here as otherwise most will be unsupported. Technically this violates branch distance, but not only is this the only reasonable choice, but it ensures consistant behavior as some infill patterns generate each line segment as its own polyline part causing a similar line forming behavior. Also it is assumed that the area that is valid a layer below is to small for support roof.
            if (polylines.pointCount() <= 3)
            {
                // add the outer wall to ensure it is correct supported instead
                polylines = ensureMaximumDistancePolyline(toPolylines(removed_overhang), connect_length, 3);
            }

            for (auto line : polylines)
            {
                LineInformation res_line;
                for (Point p : line)
                {
                    res_line.emplace_back(p, LineStatus::INVALID);
                }
                overhang_lines.emplace_back(res_line);
            }
            for (size_t lag_ctr = 1; lag_ctr <= max_overhang_insert_lag && !overhang_lines.empty() && layer_idx - coord_t(lag_ctr) >= 1; lag_ctr++)
            {
                // get least restricted avoidance for layer_idx-lag_ctr
                Polygons relevant_forbidden_below = (mesh_config.support_rests_on_model ? (only_gracious ? volumes_.getAvoidance(mesh_config.getRadius(0), layer_idx - lag_ctr, AvoidanceType::FAST, true, !xy_overrides) : volumes_.getCollision(mesh_config.getRadius(0), layer_idx - lag_ctr, !xy_overrides)) : volumes_.getAvoidance(mesh_config.getRadius(0), layer_idx - lag_ctr, AvoidanceType::FAST, false, !xy_overrides));

                std::function<bool(std::pair<Point, LineStatus>)> evaluatePoint = [&](std::pair<Point, LineStatus> p) { return relevant_forbidden_below.inside(p.first, true); };

                std::pair<std::vector<TreeSupport::LineInformation>, std::vector<TreeSupport::LineInformation>> split = splitLines(overhang_lines, evaluatePoint); // keep all lines that are invalid
                overhang_lines = split.first;

                std::vector<LineInformation> fresh_valid_points = convertLinesToInternal(convertInternalToLines(split.second), layer_idx - lag_ctr); // set all now valid lines to their correct LineStatus. Easiest way is to just discard Avoidance information for each point and evaluate them again.
                for (LineInformation line : fresh_valid_points)
                {
                    for (auto point_data : line)
                    {
                        addPointAsInfluenceArea(point_data, 0, layer_idx - lag_ctr, 0, false);
                    }
                }
            }
        }

        Polygons overhang_regular = mesh.overhang_areas[layer_idx + z_distance_delta].offset(support_outset).difference(relevant_forbidden);
        Polygons overhang_roofs;
        if (roof_enabled)
        {
            overhang_roofs = mesh.overhang_areas[layer_idx + z_distance_delta].offset(roof_outset).difference(relevant_forbidden);
            overhang_roofs.removeSmallAreas(minimum_roof_area);
            overhang_regular = overhang_regular.difference(overhang_roofs);
            for (Polygons roof_part : overhang_roofs.splitIntoParts(true))
            {
                overhang_processing.emplace_back(roof_part, true);
            }
        }
        overhang_regular.removeSmallAreas(minimum_support_area);

        for (Polygons support_part : overhang_regular.splitIntoParts(true))
        {
            overhang_processing.emplace_back(support_part, false);
        }

        for (std::pair<Polygons, bool> overhang_pair : overhang_processing)
        {
            const bool roof_allowed_for_this_part = overhang_pair.second;
            Polygons overhang_outset = overhang_pair.first;

            std::vector<LineInformation> overhang_lines;
            Polygons last_overhang = overhang_outset;
            size_t dtt_roof = 0;
            std::vector<Polygons> added_roofs(support_roof_layers); // sometimes roofs could be empty as the pattern does not generate lines if the area is narrow enough (i am looking at you, concentric infill). To catch these cases the added roofs are saved to be evaluated later.

            if (roof_allowed_for_this_part)
            {
                for (dtt_roof = 0; dtt_roof < support_roof_layers && layer_idx - dtt_roof >= 1; dtt_roof++)
                {
                    // here the roof is handled. If roof can not be added the branches will try to not move instead
                    Polygons forbidden_next = (mesh_config.support_rests_on_model ? (only_gracious ? volumes_.getAvoidance(mesh_config.getRadius(0), layer_idx - (dtt_roof + 1), AvoidanceType::FAST, true, !xy_overrides) : volumes_.getCollision(mesh_config.getRadius(0), layer_idx - (dtt_roof + 1), !xy_overrides)) : volumes_.getAvoidance(mesh_config.getRadius(0), layer_idx - (dtt_roof + 1), AvoidanceType::FAST, false, !xy_overrides));
                    Polygons overhang_outset_next = overhang_outset.difference(forbidden_next);
                    if (overhang_outset_next.area() / (1000 * 1000) < minimum_roof_area) // next layer down the roof area would be to small so we have to insert our roof support here. Also convert squaremicrons to squaremilimeter
                    {
                        size_t dtt_before = dtt_roof > 0 ? dtt_roof - 1 : 0;
                        if (dtt_roof != 0)
                        {
                            overhang_lines = convertLinesToInternal(ensureMaximumDistancePolyline(generateLines(last_overhang, true, layer_idx - dtt_before), connect_length, 1), layer_idx - dtt_before);
                            overhang_lines = splitLines(overhang_lines, getEvaluatePointForNextLayerFunction(layer_idx - dtt_before)).first;
                        }

                        break;
                    }
                    added_roofs[dtt_roof] = overhang_outset;
                    last_overhang = overhang_outset;
                    overhang_outset = overhang_outset_next;
                }
            }

            size_t layer_generation_dtt = std::max(dtt_roof, size_t(1)) - 1; // 1 inside max and -1 outside to avoid underflow. layer_generation_dtt=dtt_roof-1 if dtt_roof!=0;
            if (overhang_lines.empty() && dtt_roof != 0 && generateLines(overhang_outset, true, layer_idx - layer_generation_dtt).empty()) // if the roof should be valid, check that the area does generate lines. This is NOT guaranteed.
            {
                for (size_t idx = 0; idx < dtt_roof; idx++)
                {
                    // check for every roof area that it has resulting lines. Remember idx 1 means the 2. layer of roof => higher idx == lower layer
                    if (generateLines(added_roofs[idx], true, layer_idx - idx).empty())
                    {
                        dtt_roof = idx;
                        layer_generation_dtt = std::max(dtt_roof, size_t(1)) - 1;
                        break;
                    }
                }
            }
#pragma omp critical(storage)
            {
                for (size_t idx = 0; idx < dtt_roof; idx++)
                {
                    storage.support.supportLayers[layer_idx - idx].support_roof.add(added_roofs[idx]);
                }
            }

            if (overhang_lines.empty())
            {
                Polygons polylines = ensureMaximumDistancePolyline(generateLines(overhang_outset, dtt_roof != 0, layer_idx - layer_generation_dtt), dtt_roof == 0 ? support_line_width : connect_length, 1); // support_line_width to form a line here as otherwise most will be unsupported. Technically this violates branch distance, but not only is this the only reasonable choice, but it ensures consistant behaviour as some infill patterns generate each line segment as its own polyline part causing a similar line forming behaviour. This is not doen when a roof is above as the roof will support the model and the trees only need to support the roof
                if (polylines.pointCount() <= 3)
                {
                    // add the outer wall to ensure it is correct supported instead
                    polylines = ensureMaximumDistancePolyline(toPolylines(overhang_outset), connect_length, 3);
                }
                LayerIndex last_insert_layer = layer_idx - dtt_roof;
                overhang_lines = convertLinesToInternal(polylines, last_insert_layer);
            }


            if (int(dtt_roof) > layer_idx && roof_allowed_for_this_part) // reached buildplate
            {
#pragma omp critical(storage)
                {
                    storage.support.supportLayers[0].support_roof.add(overhang_outset);
                }
            }
            else // normal trees have to be generated
            {
                for (LineInformation line : overhang_lines)
                {
                    for (auto point_data : line)
                    {
                        addPointAsInfluenceArea(point_data, 0, layer_idx - dtt_roof, support_roof_layers - dtt_roof, dtt_roof != 0);
                    }
                }
            }
        }
    }

    delete cross_fill_provider;
}

Polygons TreeSupport::safeOffsetInc(const Polygons& me, coord_t distance, const Polygons& collision, coord_t safe_step_size, coord_t last_safe_step_size, size_t min_amount_offset) const
{
    Polygons ret = safeUnion(me); // ensure sane input
    if (distance == 0)
    {
        return ret.difference(collision).unionPolygons();
    }
    if (safe_step_size < 0 || last_safe_step_size < 0)
    {
        logError("Offset increase got negative distance!\n");
        return ret.difference(collision).unionPolygons();
    }

    coord_t step_size = safe_step_size;
    size_t steps = distance > last_safe_step_size ? (distance - last_safe_step_size) / step_size : 0;
    if (steps + (distance < last_safe_step_size || distance % step_size != 0) < min_amount_offset) // yes one can add a bool as the standard specifies that a result from compare operators has to be 0 or 1
    {
        // we need to reduce the stepsize to ensure we offset the required amount (could be avoided if arcRadiance were exposed as we could just increase this when me has not enough vertices)
        step_size = distance / (min_amount_offset - 1);
        steps = distance / step_size;
    }

    // offset in steps
    for (size_t i = 0; i < steps; i++)
    {
        ret = ret.offset(step_size, ClipperLib::jtRound).difference(collision).unionPolygons();
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if (i % 10 == 7)
        {
            ret.simplify(15);
        }
    }

    ret = ret.offset(distance - steps * step_size, ClipperLib::jtRound); // offset the remainder

    return ret.difference(collision).unionPolygons(); // ensure sane output
}


void TreeSupport::mergeHelper(std::map<SupportElement, AABB>& reduced_aabb, std::map<SupportElement, AABB>& input_aabb, const std::unordered_map<SupportElement, Polygons>& to_bp_areas, const std::unordered_map<SupportElement, Polygons>& to_model_areas, const std::map<SupportElement, Polygons>& influence_areas, std::unordered_map<SupportElement, Polygons>& insert_bp_areas, std::unordered_map<SupportElement, Polygons>& insert_model_areas, std::unordered_map<SupportElement, Polygons>& insert_influence, std::vector<SupportElement>& erase, const LayerIndex layer_idx)
{
    const bool first_merge_iteration = reduced_aabb.empty(); // If this is the first iteration, all elements in input have to be merged with each other
    for (std::map<SupportElement, AABB>::iterator influence_iter = input_aabb.begin(); influence_iter != input_aabb.end(); influence_iter++)
    {
        bool merged = false;
        AABB influence_aabb = influence_iter->second;
        for (std::map<SupportElement, AABB>::iterator reduced_check_iter = reduced_aabb.begin(); reduced_check_iter != reduced_aabb.end(); reduced_check_iter++)
        {
            // As every area has to be checked for overlaps with other areas, some fast heuristic is needed to abort early if clearly possible
            // This is so performance critical that using a map lookup instead of the direct access of the cached AABBs can have a surprisingly large performance impact
            AABB aabb = reduced_check_iter->second;
            if (aabb.hit(influence_aabb))
            {
                if (!first_merge_iteration && input_aabb.count(reduced_check_iter->first))
                {
                    break; // Do not try to merge elements that already should have been merged. Done for potential performance improvement.
                }

                bool merging_gracious_and_non_gracious = reduced_check_iter->first.to_model_gracious != influence_iter->first.to_model_gracious; // we do not want to merge a gracious with a non gracious area as bad placement could negatively impact the dependability of the whole subtree
                bool merging_to_bp = reduced_check_iter->first.to_buildplate && influence_iter->first.to_buildplate;
                bool merging_min_and_regular_xy = reduced_check_iter->first.use_min_xy_dist != influence_iter->first.use_min_xy_dist; // could cause some errors with the increase of one area, as it is assumed that if the smaller is increased by the delta to the larger it is engulfed by it already. But because a different collision may be removed from the in draw area generated circles, this assumption could be wrong.
                coord_t increased_to_model_radius = 0;
                size_t larger_to_model_dtt = 0;

                if (!merging_to_bp)
                {
                    coord_t infl_radius = config.getRadius(influence_iter->first); // get the real radius increase as the user does not care for the collision model.
                    coord_t redu_radius = config.getRadius(reduced_check_iter->first);
                    if (reduced_check_iter->first.to_buildplate != influence_iter->first.to_buildplate)
                    {
                        if (reduced_check_iter->first.to_buildplate)
                        {
                            if (infl_radius < redu_radius)
                            {
                                increased_to_model_radius = influence_iter->first.increased_to_model_radius + redu_radius - infl_radius;
                            }
                        }
                        else
                        {
                            if (infl_radius > redu_radius)
                            {
                                increased_to_model_radius = reduced_check_iter->first.increased_to_model_radius + infl_radius - redu_radius;
                            }
                        }
                    }
                    larger_to_model_dtt = std::max(influence_iter->first.distance_to_top, reduced_check_iter->first.distance_to_top);
                }

                // if a merge could place a stable branch on unstable ground, would be increasing the radius further than allowed to when merging to model and to_bp trees or would merge to model before it is known they will even been drawn the merge is skipped
                if (merging_min_and_regular_xy || merging_gracious_and_non_gracious || increased_to_model_radius > config.max_to_model_radius_increase || (!merging_to_bp && larger_to_model_dtt < config.min_dtt_to_model && !reduced_check_iter->first.supports_roof && !influence_iter->first.supports_roof))
                {
                    continue;
                }

                Polygons relevant_infl;
                Polygons relevant_redu;

                if (merging_to_bp)
                {
                    relevant_infl = to_bp_areas.count(influence_iter->first) ? to_bp_areas.at(influence_iter->first) : Polygons(); // influence_iter->first is a new element => not required to check if it was changed
                    relevant_redu = insert_bp_areas.count(reduced_check_iter->first) ? insert_bp_areas[reduced_check_iter->first] : (to_bp_areas.count(reduced_check_iter->first) ? to_bp_areas.at(reduced_check_iter->first) : Polygons());
                }
                else
                {
                    relevant_infl = to_model_areas.count(influence_iter->first) ? to_model_areas.at(influence_iter->first) : Polygons();
                    relevant_redu = insert_model_areas.count(reduced_check_iter->first) ? insert_model_areas[reduced_check_iter->first] : (to_model_areas.count(reduced_check_iter->first) ? to_model_areas.at(reduced_check_iter->first) : Polygons());
                }

                const bool red_bigger = config.getCollisionRadius(reduced_check_iter->first) > config.getCollisionRadius(influence_iter->first);
                const bool equal_size = config.getCollisionRadius(reduced_check_iter->first) == config.getCollisionRadius(influence_iter->first);
                std::pair<SupportElement, Polygons> smaller_rad = red_bigger ? std::pair<SupportElement, Polygons>(influence_iter->first, relevant_infl) : std::pair<SupportElement, Polygons>(reduced_check_iter->first, relevant_redu);
                std::pair<SupportElement, Polygons> bigger_rad = red_bigger ? std::pair<SupportElement, Polygons>(reduced_check_iter->first, relevant_redu) : std::pair<SupportElement, Polygons>(influence_iter->first, relevant_infl);
                const coord_t real_radius_delta = std::abs(config.getRadius(bigger_rad.first) - config.getRadius(smaller_rad.first));
                const coord_t smaller_collision_radius = config.getCollisionRadius(smaller_rad.first);

                // the area of the bigger radius is used to ensure correct placement regarding the relevant avoidance, so if that would change an invalid area may be created
                if (!bigger_rad.first.can_use_safe_radius && smaller_rad.first.can_use_safe_radius && !equal_size)
                {
                    continue;
                }


                // the bigger radius is used to verify that the area is stil valid after the increase with the delta. If there were a point where the big influence area could be valid with can_use_safe_radius the element would already be can_use_safe_radius
                // the smaller radius, which gets increased by delta may reach into the area where use_min_xy_dist is no longer required.
                bool use_min_radius = bigger_rad.first.use_min_xy_dist && smaller_rad.first.use_min_xy_dist;

                // The idea is that the influence area with the smaller collision radius is increased by the radius difference.
                // If this area has any intersections with the influence area of the larger collision radius, a branch (of the larger collision radius) placed in this intersection, has already engulfed the branch of the smaller collision radius.
                // Because of this a merge may happen even if the influence areas (that represent possible center points of branches) do not intersect yet.
                // Remember that collision radius <= real radius as otherwise this assumption would be false.
                Polygons small_rad_increased_by_big_minus_small = safeOffsetInc(smaller_rad.second, real_radius_delta, volumes_.getCollision(smaller_collision_radius, layer_idx - 1, use_min_radius), 2 * (config.xy_distance + smaller_collision_radius - 3), 2 * (config.xy_distance + smaller_collision_radius - 3), 0); // -3 avoids possible rounding errors
                Polygons intersect = small_rad_increased_by_big_minus_small.intersection(bigger_rad.second);

                if (intersect.area() > 1) // dont use empty as a line is not empty, but for this use-case it very well may be (and would be one layer down as union does not keep lines)
                {
                    if (intersect.offset(-25).area() <= 1) // check if the overlap is large enough (Small ares tend to attract rounding errors in clipper). While 25 was guessed as enough, i did not have reason to change it.
                    {
                        continue;
                    }

                    // Do the actual merge now that the branches are confirmed to be able to intersect.

                    // calculate which point is closest to the point of the last merge (or tip center if no merge above it has happened)
                    // used at the end to estimate where to best place the branch on the bottom most layer
                    // could be replaced with a random point inside the new area
                    Point new_pos = reduced_check_iter->first.next_position;
                    if (!intersect.inside(new_pos, true))
                    {
                        PolygonUtils::moveInside(intersect, new_pos);
                    }

                    if (increased_to_model_radius == 0)
                    {
                        increased_to_model_radius = std::max(reduced_check_iter->first.increased_to_model_radius, influence_iter->first.increased_to_model_radius);
                    }

                    SupportElement key(reduced_check_iter->first, influence_iter->first, layer_idx - 1, new_pos, increased_to_model_radius, config);

                    Polygons intersect_influence;
                    Polygons infl_small = insert_influence.count(smaller_rad.first) ? insert_influence[smaller_rad.first] : (influence_areas.count(smaller_rad.first) ? influence_areas.at(smaller_rad.first) : Polygons());
                    Polygons infl_big = insert_influence.count(bigger_rad.first) ? insert_influence[bigger_rad.first] : (influence_areas.count(bigger_rad.first) ? influence_areas.at(bigger_rad.first) : Polygons());
                    Polygons small_rad_increased_by_big_minus_small_infl = safeOffsetInc(infl_small, real_radius_delta, volumes_.getCollision(smaller_collision_radius, layer_idx - 1, use_min_radius), 2 * (config.xy_distance + smaller_collision_radius - 3), 2 * (config.xy_distance + smaller_collision_radius - 3), 0);
                    intersect_influence = small_rad_increased_by_big_minus_small_infl.intersection(infl_big); // if the one with the bigger radius with the lower radius removed overlaps we can merge
                    intersect_influence = safeUnion(intersect_influence, intersect); // Rounding errors again. Do not ask me where or why.

                    Polygons intersect_sec;
                    if (merging_to_bp && config.support_rests_on_model)
                    {
                        if (key.to_model_gracious)
                        {
                            Polygons sec_small = insert_model_areas.count(smaller_rad.first) ? insert_model_areas[smaller_rad.first] : (to_model_areas.count(smaller_rad.first) ? to_model_areas.at(smaller_rad.first) : Polygons());
                            Polygons sec_big = insert_model_areas.count(bigger_rad.first) ? insert_model_areas[bigger_rad.first] : (to_model_areas.count(bigger_rad.first) ? to_model_areas.at(bigger_rad.first) : Polygons());
                            Polygons small_rad_increased_by_big_minus_small_sec = safeOffsetInc(sec_small, real_radius_delta, volumes_.getCollision(smaller_collision_radius, layer_idx - 1, use_min_radius), 2 * (config.xy_distance + smaller_collision_radius - 3), 2 * (config.xy_distance + smaller_collision_radius - 3), 0);
                            intersect_sec = small_rad_increased_by_big_minus_small_sec.intersection(sec_big); // if the one with the bigger radius with the lower radius removed overlaps we can merge
                            intersect_influence = safeUnion(intersect_influence, intersect_sec); // still rounding errors
                        }
                        else
                        {
                            intersect_sec = intersect_influence;
                        }
                    }

                    // remove the now merged elements from all buckets, as they do not exist anymore in their old form
                    insert_bp_areas.erase(reduced_check_iter->first);
                    insert_bp_areas.erase(influence_iter->first);
                    insert_model_areas.erase(reduced_check_iter->first);
                    insert_model_areas.erase(influence_iter->first);
                    insert_influence.erase(reduced_check_iter->first);
                    insert_influence.erase(influence_iter->first);

                    (merging_to_bp ? insert_bp_areas : insert_model_areas).emplace(key, intersect);
                    if (merging_to_bp && config.support_rests_on_model)
                    {
                        insert_model_areas.emplace(key, intersect_sec);
                    }
                    insert_influence.emplace(key, intersect_influence);

                    erase.emplace_back(reduced_check_iter->first);
                    erase.emplace_back(influence_iter->first);
                    Polygons merge = intersect.unionPolygons(intersect_sec).offset(config.getRadius(key), ClipperLib::jtRound).difference(volumes_.getCollision(0, layer_idx - 1)); // regular union should be preferable here as Polygons can only become smaller through rounding errors (smaller!=has smaller area as holes have a negative area.)

                    reduced_aabb.erase(reduced_check_iter->first); // this invalidates reduced_check_iter
                    reduced_aabb.emplace(key, AABB(merge));

                    merged = true;
                    break;
                }
            }
        }

        if (!merged)
        {
            reduced_aabb[influence_iter->first] = influence_aabb;
        }
    }
}


void TreeSupport::mergeInfluenceAreas(std::unordered_map<SupportElement, Polygons>& to_bp_areas, std::unordered_map<SupportElement, Polygons>& to_model_areas, std::map<SupportElement, Polygons>& influence_areas, LayerIndex layer_idx)
{
    /*
     * Idea behind this is that the calculation of merges can be accelerated a bit using divide and conquer:
     * If two groups of areas are already merged, only all elements in group 2 have to be merged into group one.
     * This can only accelerate by factor 2 (as half the work is merging the last two groups).
     * The actual merge logic is found in mergeHelper. This function only manages parallelization of different mergeHelper calls.
     */


    const size_t input_size = influence_areas.size();
    size_t num_threads = 1;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
#endif
    if (input_size == 0)
    {
        return;
    }
    constexpr int min_elements_per_bucket = 2;

    // max_bucket_count is input_size/min_elements_per_bucket round down to the next 2^n.
    // The rounding to 2^n is to ensure improved performance, as every iteration two buckets will be merged, halving the amount of buckets.
    // If halving would cause an uneven count, e.g. 3 Then bucket 0 and 1 would have to be merged, and in the next iteration the last remaining buckets. This is assumed to not be optimal performance-wise.
    const size_t max_bucket_count = std::pow(2, std::floor(std::log(round_up_divide(input_size, min_elements_per_bucket))));
    int bucket_count = std::min(max_bucket_count, num_threads); // do not use more buckets than available threads.

    // To achieve that every element in a bucket is already correctly merged with other elements in this bucket
    // an extra empty bucket is created for each bucket, and the elements are merged into the empty one.
    // Each thread will then process two buckets by merging all elements in the second bucket into the first one as mergeHelper will disable not trying to merge elements from the same bucket in this case.
    std::vector<std::map<SupportElement, Polygons>> buckets_area(2 * bucket_count);
    std::vector<std::map<SupportElement, AABB>> buckets_aabb(2 * bucket_count);


    size_t position = 0, counter = 0;
    const size_t over_elements = input_size % bucket_count;
    const size_t elements_per_step = input_size / bucket_count;

    // split the data in x parts to be able to divide and conquer
    // the first "over_elements" of buckets gets elements_per_step+1 elements
    for (std::map<SupportElement, Polygons>::iterator iter = influence_areas.begin(); iter != influence_areas.end(); iter++)
    {
        buckets_area[position * 2 + 1].emplace(iter->first, iter->second); // only use every second bucket beginning with 1 as this makes the parallel call later easier as we assume everything in a bucket i%2==0 is already processed
        counter++;
        if ((counter == elements_per_step && position >= over_elements) || counter > elements_per_step)
        {
            position++;
            counter = 0;
        }
    }

    // precalculate the AABBs from the influence areas.
#pragma omp parallel for
    for (size_t idx = 1; idx < buckets_area.size(); idx += 2) // +=2 as in the beginning only uneven buckets will be filled
    {
        for (const std::pair<SupportElement, Polygons>& input_pair : buckets_area[idx])
        {
            AABB outer_support_wall_aabb = AABB(input_pair.second);
            outer_support_wall_aabb.expand(config.getRadius(input_pair.first));
            buckets_aabb[idx].emplace(input_pair.first, outer_support_wall_aabb);
        }
    }

    while (buckets_area.size() > 1)
    {
        // Some temporary storage, of elements that have to be inserted or removed from the background storage. Only one per two buckets required
        std::vector<std::unordered_map<SupportElement, Polygons>> insert_main(buckets_area.size() / 2);
        std::vector<std::unordered_map<SupportElement, Polygons>> insert_secondary(buckets_area.size() / 2);
        std::vector<std::unordered_map<SupportElement, Polygons>> insert_influence(buckets_area.size() / 2);
        std::vector<std::vector<SupportElement>> erase(buckets_area.size() / 2);

#pragma omp parallel for schedule(dynamic)
        for (coord_t i = 0; i < (coord_t)buckets_area.size() - 1; i = i + 2)
        {
            // Merge bucket_count adjacent to each other, merging uneven bucket numbers into even buckets
            mergeHelper(buckets_aabb[i], buckets_aabb[i + 1], to_bp_areas, to_model_areas, influence_areas, insert_main[i / 2], insert_secondary[i / 2], insert_influence[i / 2], erase[i / 2], layer_idx);
            buckets_area[i + 1].clear(); // clear now irrelevant max_bucket_count, and delete them later
            buckets_aabb[i + 1].clear();
        }

        for (coord_t i = 0; i < (coord_t)buckets_area.size() - 1; i = i + 2)
        {
            for (SupportElement& del : erase[i / 2])
            {
                to_bp_areas.erase(del);
                to_model_areas.erase(del);
                influence_areas.erase(del);
            }

            for (const std::pair<SupportElement, Polygons>& tup : insert_main[i / 2])
            {
                to_bp_areas.emplace(tup);
            }

            for (const std::pair<SupportElement, Polygons>& tup : insert_secondary[i / 2])
            {
                to_model_areas.emplace(tup);
            }
            for (const std::pair<SupportElement, Polygons>& tup : insert_influence[i / 2])
            {
                influence_areas.emplace(tup);
            }
        }

        auto position = std::remove_if(buckets_area.begin(), buckets_area.end(), [&](const std::map<SupportElement, Polygons> x) mutable { return x.empty(); });
        buckets_area.erase(position, buckets_area.end());

        auto position_aabb = std::remove_if(buckets_aabb.begin(), buckets_aabb.end(), [&](const std::map<SupportElement, AABB> x) mutable { return x.empty(); });
        buckets_aabb.erase(position_aabb, buckets_aabb.end());
    }

    // input.clear(); // //insert parts back to input
    // input.insert(buckets_area[0].begin(),buckets_area[0].end());
}


std::optional<TreeSupport::SupportElement> TreeSupport::increaseSingleArea(AreaIncreaseSettings settings, LayerIndex layer_idx, SupportElement* parent, const Polygons& relevant_offset, Polygons& to_bp_data, Polygons& to_model_data, Polygons& increased, const coord_t overspeed, const bool mergelayer)
{
    SupportElement current_elem(parent); // also increases DTT by one
    Polygons check_layer_data;
    if (settings.increase_radius)
    {
        current_elem.effective_radius_height += 1;
    }
    coord_t radius = config.getCollisionRadius(current_elem);

    if (settings.move)
    {
        increased = relevant_offset;
        if (overspeed > 0)
        {
            increased = safeOffsetInc(increased, overspeed, volumes_.getWallRestiction(config.getCollisionRadius(*parent), layer_idx, parent->use_min_xy_dist), current_elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance, radius + (current_elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance), 1); // offsetting in 2 steps makes our offsetted area rounder preventing (rounding) errors created by to pointy areas. May not be a problem anymore though.
        }
        if (settings.no_error && settings.move)
        {
            increased.simplify(25); // as ClipperLib::jtRound has to be used for offsets this simplify is VERY important for performance.
        }
    }
    else // if no movement is done the areas keep parent area as no move == offset(0)
    {
        increased = *parent->area;
    }

    if (mergelayer || current_elem.to_buildplate)
    {
        to_bp_data = safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, false, settings.use_min_distance)));
        if (!current_elem.to_buildplate && to_bp_data.area() > 1) // mostly happening in the tip, but with merges one should check every time, just to be sure.
        {
            current_elem.to_buildplate = true; // sometimes nodes that can reach the buildplate are marked as cant reach, tainting subtrees. This corrects it.
            logDebug("Corrected taint leading to a wrong to model value on layer %lld targeting %lld with radius %lld\n", layer_idx - 1, current_elem.target_height, radius);
        }
    }
    if (config.support_rests_on_model)
    {
        if (mergelayer || current_elem.to_model_gracious)
        {
            to_model_data = safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, true, settings.use_min_distance)));
        }

        if (!current_elem.to_model_gracious)
        {
            if (mergelayer && to_model_data.area() >= 1)
            {
                current_elem.to_model_gracious = true;
                logDebug("Corrected taint leading to a wrong non gracious value on layer %lld targeting %lld with radius %lld\n", layer_idx - 1, current_elem.target_height, radius);
            }
            else
            {
                to_model_data = safeUnion(increased.difference(volumes_.getCollision(radius, layer_idx - 1, settings.use_min_distance)));
            }
        }
    }

    check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;

    if (settings.increase_radius && check_layer_data.area() > 1)
    {
        std::function<bool(coord_t)> validWithRadius = [&](coord_t next_radius) {
            if (volumes_.ceilRadius(next_radius, settings.use_min_distance) <= volumes_.ceilRadius(radius, settings.use_min_distance))
            {
                return true;
            }

            Polygons to_bp_data_2;
            if (current_elem.to_buildplate)
            {
                to_bp_data_2 = increased.difference(volumes_.getAvoidance(next_radius, layer_idx - 1, settings.type, false, settings.use_min_distance)).unionPolygons(); // regular union as output will not be used later => this area should always be a subset of the safeUnion one (i think)
            }
            Polygons to_model_data_2;
            if (config.support_rests_on_model && !current_elem.to_buildplate)
            {
                if (!current_elem.to_model_gracious)
                {
                    to_model_data_2 = increased.difference(volumes_.getCollision(next_radius, layer_idx - 1, settings.use_min_distance)).unionPolygons();
                }
                else
                {
                    to_model_data_2 = increased.difference(volumes_.getAvoidance(next_radius, layer_idx - 1, settings.type, true, settings.use_min_distance)).unionPolygons();
                }
            }
            Polygons check_layer_data_2 = current_elem.to_buildplate ? to_bp_data_2 : to_model_data_2;

            return check_layer_data_2.area() > 1;
        };
        bool increase_bp_foot = config.recommendedMinRadius(layer_idx - 1) > config.getRadius(current_elem) && current_elem.to_buildplate;

        coord_t ceil_radius_before = volumes_.ceilRadius(radius, settings.use_min_distance);

        if (increase_bp_foot && config.getRadius(current_elem) >= config.branch_radius)
        {
            if (validWithRadius(config.getRadius(current_elem.effective_radius_height, current_elem.elephant_foot_increases + 1)))
            {
                current_elem.elephant_foot_increases += 1;
                radius = config.getCollisionRadius(current_elem);
            }
        }

        if (config.getCollisionRadius(current_elem) < config.increase_radius_until_radius && config.getCollisionRadius(current_elem) < config.getRadius(current_elem))
        {
            coord_t target_radius = std::min(config.getRadius(current_elem), config.increase_radius_until_radius);
            coord_t current_ceil_radius = volumes_.getRadiusNextCeil(radius, settings.use_min_distance);

            while (current_ceil_radius < target_radius && validWithRadius(volumes_.getRadiusNextCeil(current_ceil_radius + 1, settings.use_min_distance)))
            {
                current_ceil_radius = volumes_.getRadiusNextCeil(current_ceil_radius + 1, settings.use_min_distance);
            }
            size_t resulting_eff_dtt = current_elem.effective_radius_height;
            while (resulting_eff_dtt + 1 < current_elem.distance_to_top && config.getRadius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases) <= current_ceil_radius && config.getRadius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases) <= config.getRadius(current_elem))
            {
                resulting_eff_dtt++;
            }
            current_elem.effective_radius_height = resulting_eff_dtt;
        }
        radius = config.getCollisionRadius(current_elem);
        if (ceil_radius_before != volumes_.ceilRadius(radius, settings.use_min_distance))
        {
            if (current_elem.to_buildplate)
            {
                to_bp_data = safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, false, settings.use_min_distance)));
            }
            if (config.support_rests_on_model && (!current_elem.to_buildplate || mergelayer))
            {
                if (!current_elem.to_model_gracious)
                {
                    to_model_data = safeUnion(increased.difference(volumes_.getCollision(radius, layer_idx - 1, settings.use_min_distance)));
                }
                else
                {
                    to_model_data = safeUnion(increased.difference(volumes_.getAvoidance(radius, layer_idx - 1, settings.type, true, settings.use_min_distance)));
                }
            }
            check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;
            if (check_layer_data.area() < 1)
            {
                logError("Lost area by doing catch up from %lld to radius %lld radius %d\n", ceil_radius_before, volumes_.ceilRadius(config.getCollisionRadius(current_elem), settings.use_min_distance));
            }
        }
    }

    return check_layer_data.area() > 1 ? std::optional<SupportElement>(current_elem) : std::optional<SupportElement>();
}


void TreeSupport::increaseAreas(std::unordered_map<SupportElement, Polygons>& to_bp_areas, std::unordered_map<SupportElement, Polygons>& to_model_areas, std::map<SupportElement, Polygons>& influence_areas, std::vector<SupportElement*>& bypass_merge_areas, const std::vector<SupportElement*>& last_layer, const LayerIndex layer_idx, const bool mergelayer)
{
#pragma omp parallel for schedule(dynamic)
    for (long long unsigned int i = 0; i < last_layer.size(); i++)
    {
        SupportElement* parent = last_layer[i];

        SupportElement elem(parent); // also increases dtt

        Polygons wall_restriction = volumes_.getWallRestiction(config.getCollisionRadius(*parent), layer_idx, parent->use_min_xy_dist); // Abstract representation of the model outline. If an influence area would move through it, it could teleport through a wall.

        Polygons to_bp_data, to_model_data;
        coord_t radius = config.getCollisionRadius(elem);

        // When the radius increases, the outer "support wall" of the branch will have been moved farther away from the center (as this is the definition of radius).
        // As it is not specified that the support_tree_angle has to be one of the center of the branch, it is here seen as the smaller angle of the outer wall of the branch, to the outer wall of the same branch one layer above.
        // As the branch may have become larger the distance between these 2 walls is smaller than the distance of the center points.
        // These extra distance is added to the movement distance possible for this layer.

        coord_t extra_speed = 5; // The extra speed is added to both movement distances. Also move 5 microns faster than allowed to avoid rounding errors, this may cause issues at VERY VERY small layer heights.
        coord_t extra_slow_speed = 0; // Only added to the slow movement distance.
        const coord_t ceiled_parent_radius = volumes_.ceilRadius(config.getCollisionRadius(*parent), parent->use_min_xy_dist);
        coord_t projected_radius_increased = config.getRadius(parent->effective_radius_height + 1, parent->elephant_foot_increases);
        coord_t projected_radius_delta = projected_radius_increased - config.getCollisionRadius(*parent);
        if (ceiled_parent_radius == volumes_.ceilRadius(projected_radius_increased, parent->use_min_xy_dist) || projected_radius_increased < config.increase_radius_until_radius)
        {
            // If it is guaranteed possible to increase the radius, the maximum movement speed can be increased, as it is assumed that the maximum movement speed is the one of the slower moving wall
            extra_speed += projected_radius_delta;
        }
        else
        {
            // if a guaranteed radius increase is not possible, only increase the slow speed
            extra_slow_speed += std::min(projected_radius_delta, (config.maximum_move_distance + extra_speed) - (config.maximum_move_distance_slow + extra_slow_speed)); // Ensure that the slow movement distance can not become larger than the fast one.
        }

        if (config.layer_start_bp_radius > layer_idx && config.recommendedMinRadius(layer_idx - 1) < config.getRadius(elem.effective_radius_height + 1, elem.elephant_foot_increases))
        {
            // can guarantee elephant foot radius increase
            if (ceiled_parent_radius == volumes_.ceilRadius(config.getRadius(parent->effective_radius_height + 1, parent->elephant_foot_increases + 1), parent->use_min_xy_dist))
            {
                extra_speed += config.branch_radius * config.diameter_scale_bp_radius;
            }
            else
            {
                extra_slow_speed += std::min(coord_t(config.branch_radius * config.diameter_scale_bp_radius), config.maximum_move_distance - (config.maximum_move_distance_slow + extra_slow_speed));
            }
        }

        const coord_t fast_speed = config.maximum_move_distance + extra_speed;
        const coord_t slow_speed = config.maximum_move_distance_slow + extra_speed + extra_slow_speed;

        Polygons offset_slow, offset_fast;

        bool add = false;
        bool bypass_merge = false;
        constexpr bool increase_radius = true, no_error = true, use_min_radius = true, move = true; // aliases for better readability

        // Determine in which order configurations are checked if they result in a valid influence area. Check will stop if a valid area is found
        std::deque<AreaIncreaseSettings> order;
        std::function<void(AreaIncreaseSettings, bool)> insertSetting = [&](AreaIncreaseSettings settings, bool back) {
            if (std::find(order.begin(), order.end(), settings) == order.end())
            {
                if (back)
                {
                    order.emplace_back(settings);
                }
                else
                {
                    order.emplace_front(settings);
                }
            }
        };

        const bool parent_moved_slow = elem.last_area_increase.increase_speed < config.maximum_move_distance;
        const bool avoidance_speed_mismatch = parent_moved_slow && elem.last_area_increase.type != AvoidanceType::SLOW;
        if (elem.last_area_increase.move && elem.last_area_increase.no_error && elem.can_use_safe_radius && !mergelayer && !avoidance_speed_mismatch && (elem.distance_to_top >= config.tip_layers || parent_moved_slow))
        {
            // assume that the avoidance type that was best for the parent is best for me. Makes this function about 7% faster.
            insertSetting(AreaIncreaseSettings(elem.last_area_increase.type, elem.last_area_increase.increase_speed < config.maximum_move_distance ? slow_speed : fast_speed, increase_radius, elem.last_area_increase.no_error, !use_min_radius, elem.last_area_increase.move), true);
            insertSetting(AreaIncreaseSettings(elem.last_area_increase.type, elem.last_area_increase.increase_speed < config.maximum_move_distance ? slow_speed : fast_speed, !increase_radius, elem.last_area_increase.no_error, !use_min_radius, elem.last_area_increase.move), true);
        }
        // branch may still go though a hole, so a check has to be done whether the hole was already passed, and the regular avoidance can be used.
        if (!elem.can_use_safe_radius)
        {
            // if the radius until which it is always increased can not be guaranteed, move fast. This is to avoid holes smaller than the real branch radius. This does not guarantee the avoidance of such holes, but ensures they are avoided if possible.
            // order.emplace_back(AvoidanceType::SLOW,!increase_radius,no_error,!use_min_radius,move);
            insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, slow_speed, increase_radius, no_error, !use_min_radius, !move), true); // did we go through the hole
            // in many cases the definition of hole is overly restrictive, so to avoid unnecessary fast movement in the tip, it is ignored there for a bit. This CAN cause a branch to go though a hole it otherwise may have avoided.
            if (elem.distance_to_top < round_up_divide(config.tip_layers, 2))
            {
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST, slow_speed, increase_radius, no_error, !use_min_radius, !move), true);
            }
            insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, increase_radius, no_error, !use_min_radius, !move), true); // did we manage to avoid the hole
            insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, !increase_radius, no_error, !use_min_radius, move), true);
            insertSetting(AreaIncreaseSettings(AvoidanceType::FAST, fast_speed, !increase_radius, no_error, !use_min_radius, move), true);
        }
        else
        {
            insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, slow_speed, increase_radius, no_error, !use_min_radius, move), true);
            // while moving fast to be able to increase the radius (b) may seems preferable (over a) this can cause the a sudden skip in movement, which looks similar to a layer shift and can reduce stability.
            // as such i have chosen to only use the user setting for radius increases as a friendly recommendation.
            insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, slow_speed, !increase_radius, no_error, !use_min_radius, move), true); // a
            if (elem.distance_to_top < config.tip_layers)
            {
                insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, slow_speed, increase_radius, no_error, !use_min_radius, move), true);
            }
            insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, increase_radius, no_error, !use_min_radius, move), true); // b
            insertSetting(AreaIncreaseSettings(AvoidanceType::FAST_SAFE, fast_speed, !increase_radius, no_error, !use_min_radius, move), true);
        }

        if (elem.use_min_xy_dist)
        {
            std::deque<AreaIncreaseSettings> new_order;
            // if the branch currently has to use min_xy_dist check if the configuration would also be valid with the regular xy_distance before checking with use_min_radius (Only happens when Support Distance priority is z overrides xy )
            for (AreaIncreaseSettings settings : order)
            {
                new_order.emplace_back(settings);
                new_order.emplace_back(settings.type, settings.increase_speed, settings.increase_radius, settings.no_error, use_min_radius, settings.move);
            }
            order = new_order;
        }
        if (elem.to_buildplate || (elem.to_model_gracious && (parent->area->intersection(volumes_.getPlaceableAreas(radius, layer_idx)).empty()))) // error case
        {
            // it is normal that we wont be able to find a new area at some point in time if we wont be able to reach layer 0 aka have to connect with the model
            insertSetting(AreaIncreaseSettings(AvoidanceType::FAST, fast_speed, !increase_radius, !no_error, elem.use_min_xy_dist, move), true);
        }
        if (elem.distance_to_top < elem.dont_move_until && elem.can_use_safe_radius) // only do not move when holes would be avoided in every case.
        {
            insertSetting(AreaIncreaseSettings(AvoidanceType::SLOW, 0, increase_radius, no_error, !use_min_radius, !move), false); // Only do not move when already in a no hole avoidance with the regular xy distance.
        }

        Polygons inc_wo_collision;

        for (AreaIncreaseSettings settings : order)
        {
            coord_t overspeed = settings.no_error ? 0 : config.maximum_move_distance / 2;
            if (offset_slow.empty())
            {
                offset_slow = safeOffsetInc(*parent->area, extra_speed + extra_slow_speed + config.maximum_move_distance_slow, wall_restriction, elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance, (elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance), 2).unionPolygons(); // offsetting in 2 steps makes our offsetted area rounder preventing (rounding) errors created by to pointy areas. At this point one can see that the Polygons class was never made for precision in the single digit micron range.
            }

            if ((settings.increase_speed != slow_speed) && offset_fast.empty())
            {
                const coord_t delta_slow_fast = config.maximum_move_distance - (config.maximum_move_distance_slow + extra_slow_speed);
                offset_fast = safeOffsetInc(offset_slow, delta_slow_fast, wall_restriction, elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance, (elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance), 1).unionPolygons();
            }
            std::optional<SupportElement> result;
            if (!settings.no_error) // ERROR CASE
            {
                // if the area becomes for whatever reason something that clipper sees as a line, offset would stop working, so ensure that even if if wrongly would be a line, it still actually has an area that can be increased
                Polygons lines_offset = safeOffsetInc(toPolylines(*parent->area).offsetPolyLine(5), extra_speed + config.maximum_move_distance, wall_restriction, elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance, (elem.use_min_xy_dist ? config.xy_min_distance : config.xy_distance), 2).unionPolygons();
                result = increaseSingleArea(settings, layer_idx, parent, offset_fast.unionPolygons(lines_offset), to_bp_data, to_model_data, inc_wo_collision, overspeed, mergelayer);
                logError("Influence area could not be increased! Data about the Influence area: Radius: %lld at layer: %lld NextTarget: %lld Distance to top: %lld Elephant foot increases %llf  use_min_xy_dist %d to buildplate %d gracious %d safe %d until move %d \n Parent %lld: Radius: %lld at layer: %lld NextTarget: %lld Distance to top: %lld Elephant foot increases %llf  use_min_xy_dist %d to buildplate %d gracious %d safe %d until move %d\n", radius, layer_idx - 1, elem.next_height, elem.distance_to_top, elem.elephant_foot_increases, elem.use_min_xy_dist, elem.to_buildplate, elem.to_model_gracious, elem.can_use_safe_radius, elem.dont_move_until, parent, config.getCollisionRadius(*parent), layer_idx, parent->next_height, parent->distance_to_top, parent->elephant_foot_increases, parent->use_min_xy_dist, parent->to_buildplate, parent->to_model_gracious, parent->can_use_safe_radius, parent->dont_move_until);
            }
            else
            {
                result = increaseSingleArea(settings, layer_idx, parent, settings.increase_speed == slow_speed ? offset_slow : offset_fast, to_bp_data, to_model_data, inc_wo_collision, overspeed, mergelayer);
            }

            if (result)
            {
                elem = result.value();
                radius = config.getCollisionRadius(elem);
                elem.last_area_increase = settings;
                add = true;
                bypass_merge = !settings.move || (settings.use_min_distance && elem.distance_to_top < config.tip_layers); // do not merge if the branch should not move or the priority has to be to get farther away from the model.
                if (settings.move)
                {
                    elem.dont_move_until = 0;
                }
                else
                {
                    elem.result_on_layer = parent->result_on_layer;
                }

                elem.can_use_safe_radius = settings.type != AvoidanceType::FAST;

                if (!settings.use_min_distance)
                {
                    elem.use_min_xy_dist = false;
                }
                if (!settings.no_error)
                {
                    logError("Trying to keep area by moving faster than intended: Success \n");
                }
                break;
            }
            else if (!settings.no_error)
            {
                logError("Trying to keep area by moving faster than intended: FAILURE! WRONG BRANCHES LIKLY! \n");
            }
        }

        if (add)
        {
            Polygons max_influence_area = safeUnion(inc_wo_collision.difference(volumes_.getCollision(radius, layer_idx - 1, elem.use_min_xy_dist)), safeUnion(to_bp_data, to_model_data)); // union seems useless, but some rounding errors somewhere can cause to_bp_data to be slightly bigger than it should be

#pragma omp critical(newLayer)
            {
                if (bypass_merge)
                {
                    Polygons* new_area = new Polygons(max_influence_area);
                    SupportElement* next = new SupportElement(elem, new_area);
                    bypass_merge_areas.emplace_back(next);
                }
                else
                {
                    influence_areas.emplace(elem, max_influence_area);
                    if (elem.to_buildplate)
                    {
                        to_bp_areas.emplace(elem, to_bp_data);
                    }
                    if (config.support_rests_on_model)
                    {
                        to_model_areas.emplace(elem, to_model_data);
                    }
                }
            }
        }
        else
        {
            parent->result_on_layer = Point(-1, -1); // If the bottom most point of a branch is set, later functions will assume that the position is valid, and ignore it. But as branches connecting with the model that are to small have to be culled, the bottom most point has to be not set. A point can be set on the top most tip layer (maybe more if it should not move for a few layers).
        }
    }
}


void TreeSupport::createLayerPathing(std::vector<std::set<SupportElement*>>& move_bounds)
{
    const double data_size_inverse = 1 / double(move_bounds.size());
    double progress_total = PROGRESS_PRECALC_AVO + PROGRESS_PRECALC_COLL + PROGRESS_GENERATE_NODES;

    auto dur_inc = std::chrono::duration_values<std::chrono::nanoseconds>::zero();
    auto dur_merge = std::chrono::duration_values<std::chrono::nanoseconds>::zero();


    const size_t merge_every_x_layers = std::min(std::min(5000 / (std::max(config.maximum_move_distance, coord_t(100))), 1000 / std::max(config.maximum_move_distance_slow, coord_t(20))), 3000 / config.layer_height); // Ensures at least one merge operation per 3mm height, 50 layers, 1 mm movement of slow speed or 5mm movement of fast speed (whatever is lowest). Values were guessed.

    // Calculate the influence areas for each layer below (Top down)
    // This is done by first increasing the influence area by the allowed movement distance, and merging them with other influence areas if possible
    for (LayerIndex layer_idx = move_bounds.size() - 1; layer_idx > 0; layer_idx--)
    {
        std::map<SupportElement, Polygons> influence_areas; // Over this map will be iterated when merging, as such it has to be ordered to ensure deterministic results.
        std::unordered_map<SupportElement, Polygons> to_bp_areas, to_model_areas; // The area of these SupportElement is not set, to avoid to much allocation and deallocation on the heap
        std::vector<SupportElement*> bypass_merge_areas; // Different to the other maps of SupportElements as these here have the area already set, as they are already to be inserted into move_bounds.

        auto ta = std::chrono::high_resolution_clock::now();

        std::vector<SupportElement*> last_layer;
        last_layer.insert(last_layer.begin(), move_bounds[layer_idx].begin(), move_bounds[layer_idx].end());

        // merging is expensive and only parallelized to a max speedup of 2. As such it may be useful to only merge every few layers to improve performance.
        bool merge_this_layer = layer_idx % merge_every_x_layers == 0;

        // ### Increase the influence areas by the allowed movement distance
        increaseAreas(to_bp_areas, to_model_areas, influence_areas, bypass_merge_areas, last_layer, layer_idx, merge_this_layer);

        auto tb = std::chrono::high_resolution_clock::now();

        if (merge_this_layer)
        {
            // ### Calculate which influence areas overlap, and merge them into a new influence area (simplified: an intersection of influence areas that have such an intersection)
            mergeInfluenceAreas(to_bp_areas, to_model_areas, influence_areas, layer_idx);
        }

        auto tc = std::chrono::high_resolution_clock::now();

        dur_inc += tb - ta;
        dur_merge += tc - tb;

        // Save calculated elements to output, and allocate Polygons on heap, as they will not be changed again.
        for (std::pair<SupportElement, Polygons> tup : influence_areas)
        {
            const SupportElement elem = tup.first;
            Polygons* new_area = new Polygons(safeUnion(tup.second));
            SupportElement* next = new SupportElement(elem, new_area);
            move_bounds[layer_idx - 1].emplace(next);

            if (new_area->area() < 1)
            {
                logError("Insert Error of Influence area on layer %lld. Origin of %lld areas. Was to bp %d\n", layer_idx - 1, elem.parents.size(), elem.to_buildplate);
            }
        }

        // Place already fully constructed elements in the output.
        for (SupportElement* elem : bypass_merge_areas)
        {
            if (elem->area->area() < 1)
            {
                logError("Insert Error of Influence area bypass on layer %lld.\n", layer_idx - 1);
            }
            move_bounds[layer_idx - 1].emplace(elem);
        }

        progress_total += data_size_inverse * PROGRESS_AREA_CALC;
        Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, PROGRESS_TOTAL);
    }

    log("Time spent with creating influence areas' subtasks: Increasing areas %lld ms merging areas: %lld ms\n", dur_inc.count() / 1000000, dur_merge.count() / 1000000);
}


void TreeSupport::setPointsOnAreas(const SupportElement* elem)
{
    // Based on the branch center point of the current layer, the point on the next (further up) layer is calculated.

    if (elem->result_on_layer == Point(-1, -1))
    {
        logError("Uninitialized support element\n");
        return;
    }

    for (SupportElement* next_elem : elem->parents)
    {
        if (next_elem->result_on_layer != Point(-1, -1)) // if the value was set somewhere else it it kept. This happens when a branch tries not to move after being unable to create a roof.
        {
            continue;
        }

        Point from = elem->result_on_layer;
        if (!(next_elem->area->inside(from, true)))
        {
            PolygonUtils::moveInside(*next_elem->area, from, 0); // Move inside has edgecases (see tests) so DONT use Polygons.inside to confirm correct move, Error with distance 0 is <= 1
            // it is not required to check if how far this move moved a point as is can be larger than maximum_movement_distance. While this seems like a problem it may for example occur after merges.
        }
        next_elem->result_on_layer = from;
        // do not call recursive because then amount of layers would be restricted by the stack size
    }
}

bool TreeSupport::setToModelContact(std::vector<std::set<SupportElement*>>& move_bounds, SupportElement* first_elem, const LayerIndex layer_idx)
{
    if (first_elem->to_model_gracious)
    {
        SupportElement* check = first_elem;

        std::vector<SupportElement*> checked;
        LayerIndex last_successfull_layer = layer_idx;
        bool set = false;

        // check for every layer upwards, up to the point where this influence area was created (either by inital insert or merge) if the branch could be placed on it, and highest up layer index.

        for (LayerIndex layer_check = layer_idx; check->next_height >= layer_check; layer_check++)
        {
            if (!check->area->intersection(volumes_.getPlaceableAreas(config.getCollisionRadius(*check), layer_check)).empty())
            {
                set = true;
                last_successfull_layer = layer_check;
            }
            checked.emplace_back(check);
            if (check->parents.size() == 1)
            {
                check = check->parents[0];
            }
            else
            {
                break; // reached merge point
            }
        }

        // Could not find valid placement, even though it should exist => error handling
        if (!set)
        {
            if (SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL)
            {
                logWarning("No valid placement found for to model gracious element on layer %lld: REMOVING BRANCH\n", layer_idx);

                for (LayerIndex layer = layer_idx; layer <= first_elem->next_height; layer++)
                {
                    move_bounds[layer].erase(checked[layer - layer_idx]);
                    delete checked[layer - layer_idx]->area;
                    delete checked[layer - layer_idx];
                }
            }
            else
            {
                logWarning("No valid placement found for to model gracious element on layer %lld\n", layer_idx);
                first_elem->to_model_gracious = false;
                return setToModelContact(move_bounds, first_elem, layer_idx);
            }
        }

        for (LayerIndex layer = layer_idx + 1; layer < last_successfull_layer - 1; layer++)
        {
            move_bounds[layer].erase(checked[layer - layer_idx]);
            delete checked[layer - layer_idx]->area;
            delete checked[layer - layer_idx];
        }

        // Guess a point inside the influence area, in which the branch will be placed in.
        Point best = checked[last_successfull_layer - layer_idx]->next_position;
        if (!checked[last_successfull_layer - layer_idx]->area->inside(best, true))
        {
            PolygonUtils::moveInside(*checked[last_successfull_layer - layer_idx]->area, best);
        }
        checked[last_successfull_layer - layer_idx]->result_on_layer = best;

        logDebug("Added gracious Support On Model Point (%lld,%lld). The current layer is %lld\n", best.X, best.Y, last_successfull_layer);

        return last_successfull_layer != layer_idx;
    }
    else // can not add graceful => just place it here and hope for the best
    {
        Point best = first_elem->next_position;
        if (!first_elem->area->inside(best, true))
        {
            PolygonUtils::moveInside(*first_elem->area, best);
        }
        first_elem->result_on_layer = best;
        first_elem->to_model_gracious = false;
        logDebug("Added NON gracious Support On Model Point (%lld,%lld). The current layer is %lld\n", best.X, best.Y, layer_idx);
        return false;
    }
}


void TreeSupport::createNodesFromArea(std::vector<std::set<SupportElement*>>& move_bounds)
{
    // Initialize points on layer 0, with a "random" point in the influence area. Point is chosen based on an inaccurate estimate where the branches will split into two, but every point inside the influence area would produce a valid result.
    for (SupportElement* init : move_bounds[0])
    {
        Point p = init->next_position;
        if (!(init->area->inside(p, true)))
        {
            PolygonUtils::moveInside(*init->area, p, 0);
        }
        init->result_on_layer = p;

        setPointsOnAreas(init); // also set the parent nodes, as these will be required for the first iteration of the loop below
    }


    for (LayerIndex layer_idx = 1; layer_idx < LayerIndex(move_bounds.size()); layer_idx++)
    {
        std::unordered_set<SupportElement*> remove;
        for (SupportElement* elem : move_bounds[layer_idx])
        {
            bool removed = false;
            if (elem->result_on_layer == Point(-1, -1)) // check if the resulting center point is not yet set
            {
                if (elem->to_buildplate || (!elem->to_buildplate && elem->distance_to_top < config.min_dtt_to_model && !elem->supports_roof))
                {
                    if (elem->to_buildplate)
                    {
                        logError("Uninitialized Influence area targeting (%lld,%lld) at target_height: %lld layer: %lld\n", elem->target_position.X, elem->target_position.Y, elem->target_height, layer_idx);
                    }
                    remove.emplace(elem); // we dont need to remove yet the parents as they will have a lower dtt and also no result_on_layer set
                    removed = true;
                    for (SupportElement* parent : elem->parents)
                    {
                        parent->result_on_layer = Point(-1, -1); // When the roof was not able to generate downwards enough, the top elements may have not moved, and have result_on_layer already set. As this branch needs to be removed => all parents result_on_layer have to be invalidated.
                    }
                    continue;
                }
                else
                {
                    // set the point where the branch will be placed on the model
                    removed = setToModelContact(move_bounds, elem, layer_idx);
                    if (removed)
                    {
                        remove.emplace(elem);
                    }
                }
            }

            if (!removed)
            {
                setPointsOnAreas(elem); // element is valid now setting points in the layer above
            }
        }

        // delete all not needed support elements
        for (SupportElement* del : remove)
        {
            move_bounds[layer_idx].erase(del);
            delete del->area;
            delete del;
        }
        remove.clear();
    }
}


void TreeSupport::drawAreas(std::vector<std::set<SupportElement*>>& move_bounds, SliceDataStorage& storage)
{
    double progress_total = PROGRESS_PRECALC_AVO + PROGRESS_PRECALC_COLL + PROGRESS_GENERATE_NODES + PROGRESS_AREA_CALC;

    Polygon branch_circle; // Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    for (unsigned int i = 0; i < SUPPORT_TREE_CIRCLE_RESOLUTION; i++)
    {
        const AngleRadians angle = static_cast<double>(i) / SUPPORT_TREE_CIRCLE_RESOLUTION * TAU;
        branch_circle.emplace_back(cos(angle) * config.branch_radius, sin(angle) * config.branch_radius);
    }
    const size_t z_distance_bottom_layers = config.z_distance_bottom_layers;
    std::vector<Polygons> support_layer_storage(move_bounds.size());
    const coord_t max_radius_change_per_layer = 1 + config.support_line_width / 2; // this is the upper limit a radius may change per layer. +1 to avoid rounding errors
    std::map<SupportElement*, SupportElement*> inverese_tree_order; // in the tree structure only the parents can be accessed. Inverse this to be able to access the children.
    std::vector<std::pair<LayerIndex, SupportElement*>> linear_data; // All SupportElements are put into a layer independent storage to improve parallelization. Was added at a point in time where this function had performance issues.
                                                                     // These were fixed by creating less initial points, but i do not see a good reason to remove a working performance optimization.
    for (LayerIndex layer_idx = 0; layer_idx < LayerIndex(move_bounds.size()); layer_idx++)
    {
        for (SupportElement* elem : move_bounds[layer_idx])
        {
            if ((layer_idx > 0 && ((!inverese_tree_order.count(elem) && elem->target_height == layer_idx) || (inverese_tree_order.count(elem) && inverese_tree_order[elem]->result_on_layer == Point(-1, -1))))) // we either come from nowhere at the final layer or we had invalid parents 2. should never happen but just to be sure
            {
                continue;
            }

            for (SupportElement* par : elem->parents)
            {
                if (par->result_on_layer == Point(-1, -1))
                {
                    continue;
                }
                inverese_tree_order.emplace(par, elem);
            }
            linear_data.emplace_back(layer_idx, elem);
        }
    }
    std::vector<Polygons> linear_inserts(linear_data.size());
    const size_t progress_inserts_check_interval = linear_data.size() / 10;

    // parallel iterating over all elements
#pragma omp parallel for
    for (coord_t i = 0; i < static_cast<coord_t>(linear_data.size()); i++)
    {
        SupportElement* elem = linear_data[i].second;
        coord_t radius = config.getRadius(*elem);
        bool parent_uses_min = false;
        SupportElement* child_elem = inverese_tree_order.count(elem) ? inverese_tree_order.at(elem) : nullptr;

        // Calculate multiple ovalized circles, to connect with every parent and child. Also generate regular circle for the current layer. Merge all these into one area.
        std::vector<std::pair<Point, coord_t>> movement_directions{ std::pair<Point, coord_t>(Point(0, 0), radius) };
        if (child_elem != nullptr)
        {
            Point movement = (child_elem->result_on_layer - elem->result_on_layer);
            movement_directions.emplace_back(movement, radius);
        }
        for (SupportElement* parent : elem->parents)
        {
            Point movement = (parent->result_on_layer - elem->result_on_layer);
            movement_directions.emplace_back(movement, std::max(config.getRadius(parent), config.support_line_width));
            parent_uses_min |= parent->use_min_xy_dist;
        }

        coord_t max_speed = 0;
        std::function<Polygons(coord_t)> generateArea = [&](coord_t offset) {
            Polygons poly;

            for (std::pair<Point, coord_t> movement : movement_directions)
            {
                max_speed = std::max(max_speed, vSize(movement.first));

                // Visualization: https://jsfiddle.net/0zvcq39L/2/
                // Ovalizes the circle to an ellipse, that contains both old center and new target position.
                double used_scale = (movement.second + offset) / (1.0 * config.branch_radius);
                Point center_position = elem->result_on_layer + movement.first / 2;
                const double moveX = movement.first.X / (used_scale * config.branch_radius);
                const double moveY = movement.first.Y / (used_scale * config.branch_radius);
                const double vsize_inv = 0.5 / (0.01 + std::sqrt(moveX * moveX + moveY * moveY));

                double matrix[] = {
                    used_scale * (1 + moveX * moveX * vsize_inv),
                    used_scale * (0 + moveX * moveY * vsize_inv),
                    used_scale * (0 + moveX * moveY * vsize_inv),
                    used_scale * (1 + moveY * moveY * vsize_inv),
                };
                Polygon circle;
                for (Point vertex : branch_circle)
                {
                    vertex = Point(matrix[0] * vertex.X + matrix[1] * vertex.Y, matrix[2] * vertex.X + matrix[3] * vertex.Y);
                    circle.add(center_position + vertex);
                }
                poly.add(circle.offset(0));
            }
            poly = poly.unionPolygons().difference(volumes_.getCollision(0, linear_data[i].first, parent_uses_min || elem->use_min_xy_dist)).unionPolygons();
            return poly;
        };


        bool fast_relative_movement = max_speed > radius * 0.75;

        // ensure branch area will not overlap with model/collision. This can happen because of e.g. ovalization or increase_until_radius.
        linear_inserts[i] = generateArea(0);

        if (fast_relative_movement || config.getRadius(*elem) - config.getCollisionRadius(*elem) > config.support_line_width)
        {
            // simulate the path the nozzle will take on the outermost wall
            // if multiple parts exist, the outer line will not go all around the support part potentially causing support material to be printed mid air
            Polygons nozzle_path = linear_inserts[i].offset(-config.support_line_width / 2);
            if (nozzle_path.splitIntoParts(false).size() > 1)
            {
                // Just try to make the area a tiny bit larger.
                linear_inserts[i] = generateArea(config.support_line_width / 2);
                nozzle_path = linear_inserts[i].offset(-config.support_line_width / 2);

                // if larger area did not fix the problem, all parts off the nozzle path that do not contain the center point are removed, hoping for the best
                if (nozzle_path.splitIntoParts(false).size() > 1)
                {
                    Polygons polygons_with_correct_center;
                    for (PolygonsPart part : nozzle_path.splitIntoParts(false))
                    {
                        if (part.inside(elem->result_on_layer, true))
                        {
                            polygons_with_correct_center = polygons_with_correct_center.unionPolygons(part);
                        }
                        else
                        {
                            // try a fuzzy inside as sometimes the point should be on the border, but is not because of rounding errors...
                            Point from = elem->result_on_layer;
                            PolygonUtils::moveInside(part, from, 0);
                            if (vSize(elem->result_on_layer - from) < 25)
                            {
                                polygons_with_correct_center = polygons_with_correct_center.unionPolygons(part);
                            }
                        }
                    }
                    linear_inserts[i] = polygons_with_correct_center.offset(config.support_line_width / 2).unionPolygons(); // Increase the area again, to ensure the nozzle path when calculated later is very similar to the one assumed above.
                    linear_inserts[i] = linear_inserts[i].difference(volumes_.getCollision(0, linear_data[i].first, parent_uses_min || elem->use_min_xy_dist)).unionPolygons();
                }
            }
        }

        if (i % progress_inserts_check_interval == 0)
        {
#pragma omp critical(progress)
            {
                progress_total += PROGRESS_DRAW_AREAS / 30; // one third of the amount of progress in done in this loop and only 10 samples are reported
                Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, PROGRESS_TOTAL);
            }
        }
    }

    std::vector<std::unordered_map<SupportElement*, Polygons>> layer_tree_polygons(move_bounds.size()); // reorder the processed data by layers again. The map also could be a vector<pair<SupportElement*,Polygons>>.

    // single threaded combining all elements to the right layers. ONLY COPYS DATA!
    for (coord_t i = 0; i < static_cast<coord_t>(linear_data.size()); i++)
    {
        layer_tree_polygons[linear_data[i].first].emplace(linear_data[i].second, linear_inserts[i]);
    }

    // in some edgecases a branch may go though a hole, where the regular radius does not fit. This can result in an apparent jump in branch radius. As such this cases need to be caught and smoothed out.

    // smooth upwards
    for (LayerIndex layer_idx = 0; layer_idx < LayerIndex(move_bounds.size()) - 1; layer_idx++)
    {
        std::vector<std::pair<SupportElement*, Polygons>> processing;
        processing.insert(processing.end(), layer_tree_polygons[layer_idx].begin(), layer_tree_polygons[layer_idx].end());
        std::vector<std::pair<SupportElement*, Polygons>> update_next(processing.size(), std::pair<SupportElement*, Polygons>(nullptr, Polygons())); // with this a lock can be avoided
#pragma omp parallel for schedule(static, 1)
        for (coord_t processing_idx = 0; processing_idx < coord_t(processing.size()); processing_idx++)
        {
            std::pair<SupportElement*, Polygons> data_pair = processing[processing_idx];

            coord_t max_outer_wall_distance = 0;
            bool do_something = false;
            for (SupportElement* parent : data_pair.first->parents)
            {
                if (config.getRadius(*parent) != config.getCollisionRadius(*parent))
                {
                    do_something = true;
                    max_outer_wall_distance = std::max(max_outer_wall_distance, vSize(data_pair.first->result_on_layer - parent->result_on_layer) - (config.getRadius(*data_pair.first) - config.getRadius(*parent)));
                }
            }
            max_outer_wall_distance += max_radius_change_per_layer; // As this change is a bit larger than what usually appears, lost radius can be slowly reclaimed over the layers.
            if (do_something)
            {
                Polygons max_allowed_area = data_pair.second.offset(max_outer_wall_distance);
                for (SupportElement* parent : data_pair.first->parents)
                {
                    if (config.getRadius(*parent) != config.getCollisionRadius(*parent))
                    {
                        update_next[processing_idx] = std::pair<SupportElement*, Polygons>(parent, layer_tree_polygons[layer_idx + 1][parent].intersection(max_allowed_area));
                    }
                }
            }
        }
        for (std::pair<SupportElement*, Polygons> data_pair : update_next)
        {
            if (data_pair.first != nullptr)
            {
                layer_tree_polygons[layer_idx + 1][data_pair.first] = data_pair.second;
            }
        }
    }

    progress_total += PROGRESS_DRAW_AREAS / 6;
    Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, PROGRESS_TOTAL); // It is just assumed that both smoothing loops together are one third of the time spent in this function. This was guessed. As the whole function is only 10%, and the smoothing is hard to predict a progress report in the loop may be not useful.

    // smooth downwards
    std::unordered_set<SupportElement*> updated_last_iteration;
    for (LayerIndex layer_idx = move_bounds.size() - 2; layer_idx >= 0; layer_idx--)
    {
        std::vector<std::pair<SupportElement*, Polygons>> processing;
        processing.insert(processing.end(), layer_tree_polygons[layer_idx].begin(), layer_tree_polygons[layer_idx].end());
        std::vector<std::pair<SupportElement*, Polygons>> update_next(processing.size(), std::pair<SupportElement*, Polygons>(nullptr, Polygons())); // with this a lock can be avoided
#pragma omp parallel for schedule(static, 1)
        for (int processing_idx = 0; processing_idx < int(processing.size()); processing_idx++)
        {
            std::pair<SupportElement*, Polygons> data_pair = processing[processing_idx];
            bool do_something = false;
            Polygons max_allowed_area;
            for (size_t idx = 0; idx < data_pair.first->parents.size(); idx++)
            {
                SupportElement* parent = data_pair.first->parents[idx];
                coord_t max_outer_line_increase = max_radius_change_per_layer;
                Polygons result = layer_tree_polygons[layer_idx + 1][parent].offset(max_outer_line_increase);
                Point direction = data_pair.first->result_on_layer - parent->result_on_layer;
                // move the polygons object
                for (auto& outer : result)
                {
                    for (Point& p : outer)
                    {
                        p += direction;
                    }
                }
                max_allowed_area.add(result);
                do_something = do_something || updated_last_iteration.count(parent) || config.getCollisionRadius(*parent) != config.getRadius(*parent);
            }

            if (do_something)
            {
                Polygons result = max_allowed_area.unionPolygons().intersection(data_pair.second);
                if (result.area() < data_pair.second.area())
                {
                    update_next[processing_idx] = std::pair<SupportElement*, Polygons>(data_pair.first, result);
                }
            }
        }
        updated_last_iteration.clear();
        for (std::pair<cura::TreeSupport::SupportElement*, Polygons> data_pair : update_next)
        {
            if (data_pair.first != nullptr)
            {
                updated_last_iteration.emplace(data_pair.first);
                layer_tree_polygons[layer_idx][data_pair.first] = data_pair.second;
            }
        }
    }

    progress_total += PROGRESS_DRAW_AREAS / 6;
    Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, PROGRESS_TOTAL);


    // drop down all trees that connect non gracefully with the model
    std::vector<std::vector<std::pair<LayerIndex, Polygons>>> dropped_down_areas(linear_data.size());
#pragma omp parallel for schedule(static, 1)
    for (coord_t idx = 0; idx < coord_t(linear_data.size()); idx++)
    {
        SupportElement* elem = linear_data[idx].second;
        bool non_gracious_model_contact = !elem->to_model_gracious && !inverese_tree_order.count(elem); // if a element has no child, it connects to whatever is below as no support further down for it will exist.

        if (non_gracious_model_contact)
        {
            Polygons rest_support = layer_tree_polygons[linear_data[idx].first][elem];
            LayerIndex counter = 1;
            while (rest_support.area() > 1 && counter < linear_data[idx].first)
            {
                rest_support = rest_support.difference(volumes_.getCollision(0, linear_data[idx].first - counter));
                dropped_down_areas[idx].emplace_back(linear_data[idx].first - counter, rest_support);
                counter++;
            }
        }
    }

    // single threaded combining all dropped down support areas to the right layers. ONLY COPYS DATA!
    for (coord_t i = 0; i < static_cast<coord_t>(dropped_down_areas.size()); i++)
    {
        for (std::pair<LayerIndex, Polygons> pair : dropped_down_areas[i])
        {
            support_layer_storage[pair.first].add(pair.second);
        }
    }

    // single threaded combining all support areas to the right layers. ONLY COPYS DATA!
    for (LayerIndex layer_idx = 0; layer_idx < LayerIndex(move_bounds.size()); layer_idx++)
    {
        for (std::pair<SupportElement*, Polygons> data_pair : layer_tree_polygons[layer_idx])
        {
            support_layer_storage[layer_idx].add(data_pair.second);
        }
    }


    progress_total = PROGRESS_PRECALC_AVO + PROGRESS_PRECALC_COLL + PROGRESS_GENERATE_NODES + PROGRESS_AREA_CALC + 2 * PROGRESS_DRAW_AREAS / 3;
    linear_inserts.clear();
    linear_data.clear();


    // Iterate over the generated circles in parallel and clean them up. Also add support floor.
#pragma omp parallel for shared(support_layer_storage, storage) schedule(dynamic)
    for (LayerIndex layer_idx = 0; layer_idx < static_cast<coord_t>(support_layer_storage.size()); layer_idx++)
    {
        support_layer_storage[layer_idx] = support_layer_storage[layer_idx].unionPolygons().smooth(50);
        if (!storage.support.supportLayers[layer_idx].support_roof.empty())
        {
            Polygons tree_lines = toPolylines(support_layer_storage[layer_idx].offset(-config.support_line_width / 2)).offsetPolyLine(config.support_line_width / 2);
            tree_lines = tree_lines.unionPolygons(toPolylines(generateSupportInfillLines(support_layer_storage[layer_idx], false, layer_idx, config.support_line_distance, storage.support.cross_fill_provider)).offsetPolyLine(config.support_line_width / 2));
            storage.support.supportLayers[layer_idx].support_roof = storage.support.supportLayers[layer_idx].support_roof.unionPolygons().difference(tree_lines); // do not draw roof where the tree is. I prefer it this way as otherwise the roof may cut of a branch from its support below.
        }

        support_layer_storage[layer_idx].simplify(std::min(coord_t(30), config.maximum_resolution), std::min(coord_t(10), config.maximum_deviation)); // simplify a bit, to ensure the output does not contain outrageous amounts of vertices. Should not be necessary, just a precaution.

        // Subtract support floors from the support area and add them to the support floor instead.
        if (config.support_bottom_layers > 0 && !support_layer_storage[layer_idx].empty())
        {
            Polygons floor_layer = storage.support.supportLayers[layer_idx].support_bottom;
            Polygons layer_outset = support_layer_storage[layer_idx].offset(config.support_bottom_offset).difference(volumes_.getCollision(0, layer_idx, false));
            size_t layers_below = 0;
            while (layers_below <= config.support_bottom_layers)
            {
                // one sample at 0 layers below, another at config.support_bottom_layers. In-between samples at config.performance_interface_skip_layers distance from each other.
                const size_t sample_layer = static_cast<size_t>(std::max(0, (static_cast<int>(layer_idx) - static_cast<int>(layers_below)) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool no_support = false;
                constexpr bool no_prime_tower = false;
                floor_layer.add(layer_outset.intersection(storage.getLayerOutlines(sample_layer, no_support, no_prime_tower)));
                if (layers_below < config.support_bottom_layers)
                {
                    layers_below = std::min(layers_below + config.performance_interface_skip_layers, config.support_bottom_layers);
                }
                else
                {
                    break;
                }
            }
            floor_layer = floor_layer.unionPolygons();
            floor_layer.removeSmallAreas(config.minimum_bottom_area);
            storage.support.supportLayers[layer_idx].support_bottom = storage.support.supportLayers[layer_idx].support_bottom.unionPolygons(floor_layer);
            support_layer_storage[layer_idx] = support_layer_storage[layer_idx].difference(floor_layer.offset(10)); // Subtract the support floor from the normal support.
        }

        for (PolygonsPart part : support_layer_storage[layer_idx].splitIntoParts(true)) // Convert every part into a PolygonsPart for the support.
        {
            PolygonsPart outline;
            outline.add(part);
            storage.support.supportLayers[layer_idx].support_infill_parts.emplace_back(outline, config.support_line_width, config.support_wall_count);
        }

#pragma omp critical(progress)
        {
            progress_total += PROGRESS_DRAW_AREAS / (3 * support_layer_storage.size());
            Progress::messageProgress(Progress::Stage::SUPPORT, progress_total * progress_multiplier + progress_offset, PROGRESS_TOTAL);
        }

#pragma omp critical(storage)
        {
            if (!storage.support.supportLayers[layer_idx].support_infill_parts.empty() || !storage.support.supportLayers[layer_idx].support_roof.empty())
            {
                storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, static_cast<int>(layer_idx));
            }
        }
    }
}



ModelVolumes::ModelVolumes(const SliceDataStorage& storage, const coord_t max_move, const coord_t max_move_slow, size_t current_mesh_idx, double progress_multiplier, double progress_offset, const std::vector<Polygons>& additional_excluded_areas) : max_move_{ std::max(max_move - 2, coord_t(0)) }, max_move_slow_{ std::max(max_move_slow - 2, coord_t(0)) }, progress_multiplier{ progress_multiplier }, progress_offset{ progress_offset }, machine_border_{ calculateMachineBorderCollision(storage.getMachineBorder()) } // -2 to avoid rounding errors
{
    anti_overhang_ = std::vector<Polygons>(storage.support.supportLayers.size(), Polygons());
    std::unordered_map<size_t, size_t> mesh_to_layeroutline_idx;
    min_maximum_deviation_ = std::numeric_limits<coord_t>::max();
    min_maximum_resolution_ = std::numeric_limits<coord_t>::max();
    support_rests_on_model = false;
    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];
        bool added = false;
        for (size_t idx = 0; idx < layer_outlines_.size(); idx++)
        {
            if (checkSettingsEquality(layer_outlines_[idx].first, mesh.settings))
            {
                added = true;
                mesh_to_layeroutline_idx[mesh_idx] = idx;
            }
        }
        if (!added)
        {
            mesh_to_layeroutline_idx[mesh_idx] = layer_outlines_.size();
            layer_outlines_.emplace_back(mesh.settings, std::vector<Polygons>(storage.support.supportLayers.size(), Polygons()));
        }
    }

    for (auto data_pair : layer_outlines_)
    {
        support_rests_on_model |= data_pair.first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
        min_maximum_deviation_ = std::min(min_maximum_deviation_, data_pair.first.get<coord_t>("meshfix_maximum_deviation"));
        min_maximum_resolution_ = std::min(min_maximum_resolution_, data_pair.first.get<coord_t>("meshfix_maximum_resolution"));
    }

    min_maximum_deviation_ = std::min(coord_t(SUPPORT_TREE_MAX_DEVIATION), min_maximum_deviation_);
    current_outline_idx = mesh_to_layeroutline_idx[current_mesh_idx];
    TreeSupport::TreeSupportSettings config(layer_outlines_[current_outline_idx].first);

    if (config.support_overrides == SupportDistPriority::Z_OVERRIDES_XY)
    {
        current_min_xy_dist = std::max(config.xy_min_distance, coord_t(100));
        current_min_xy_dist_delta = std::max(config.xy_distance, coord_t(100)) - current_min_xy_dist;
    }
    else
    {
        current_min_xy_dist = config.xy_distance;
        current_min_xy_dist_delta = 0;
    }
    increase_until_radius = config.increase_radius_until_radius;

    for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage mesh = storage.meshes[mesh_idx];
#pragma omp parallel for
        for (LayerIndex layer_idx = 0; layer_idx < coord_t(layer_outlines_[mesh_to_layeroutline_idx[mesh_idx]].second.size()); layer_idx++)
        {
            if (mesh.layer_nr_max_filled_layer < layer_idx)
            {
                continue; // cant break as omp for loop wont allow it
            }
            Polygons outline = extractOutlineFromMesh(mesh, layer_idx);
            layer_outlines_[mesh_to_layeroutline_idx[mesh_idx]].second[layer_idx].add(outline);
        }
    }
#pragma omp parallel for
    for (LayerIndex layer_idx = 0; layer_idx < coord_t(anti_overhang_.size()); layer_idx++)
    {
        if (layer_idx < coord_t(additional_excluded_areas.size()))
        {
            anti_overhang_[layer_idx].add(additional_excluded_areas[layer_idx]);
        }

        if (SUPPORT_TREE_AVOID_SUPPORT_BLOCKER)
        {
            anti_overhang_[layer_idx].add(storage.support.supportLayers[layer_idx].anti_overhang);
        }

        if (storage.primeTower.enabled)
        {
            anti_overhang_[layer_idx].add(layer_idx == 0 ? storage.primeTower.outer_poly_first_layer : storage.primeTower.outer_poly);
        }
        anti_overhang_[layer_idx] = anti_overhang_[layer_idx].unionPolygons();
    }
    for (size_t idx = 0; idx < layer_outlines_.size(); idx++)
    {
#pragma omp parallel for
        for (LayerIndex layer_idx = 0; layer_idx < coord_t(anti_overhang_.size()); layer_idx++)
        {
            layer_outlines_[idx].second[layer_idx] = layer_outlines_[idx].second[layer_idx].unionPolygons();
        }
    }
}


void ModelVolumes::precalculate(coord_t max_layer)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    precalculated = true;

    // Get the config corresponding to one mesh that is in the current group. Which one has to be irrelevant. Not the prettiest way to do this, but it ensures some calculations that may be a bit more complex like inital layer diameter are only done in once.
    TreeSupport::TreeSupportSettings config(layer_outlines_[current_outline_idx].first);

    // calculate which radius each layer in the tip may have.
    std::unordered_set<coord_t> possible_tip_radiis;
    for (size_t dtt = 0; dtt <= config.tip_layers; dtt++)
    {
        possible_tip_radiis.emplace(ceilRadius(config.getRadius(dtt)));
        possible_tip_radiis.emplace(ceilRadius(config.getRadius(dtt) + current_min_xy_dist_delta));
    }
    // It theoretically may happen in the tip, that the radius can change so much in-between 2 layers, that a ceil step is skipped (as in there is a radius r so that ceilRadius(radius(dtt))<ceilRadius(r)<ceilRadius(radius(dtt+1))). As such a radius will not reasonable happen in the tree and it will most likely not be requested, there is no need to calculate them. So just skip these.
    for (coord_t radius_eval = ceilRadius(1); radius_eval <= config.branch_radius; radius_eval = ceilRadius(radius_eval + 1))
    {
        if (!possible_tip_radiis.count(radius_eval))
        {
            ignorable_radii_.emplace(radius_eval);
        }
    }

    // it may seem that the required avoidance can be of a smaller radius when going to model (no initial layer diameter for to model branches)
    // but as for every branch going towards the bp, the to model avoidance is required to check for possible merges with to model branches, this assumption is in-fact wrong.
    std::unordered_map<coord_t, LayerIndex> radius_until_layer;
    // while it is possible to calculate,up to which layer the avoidance should be calculated, this simulation is easier to understand, and does not need to be adjusted if something of the radius calculation is changed.
    // Overhead with an assumed worst case of 6600 layers was about 2ms
    for (LayerIndex simulated_dtt = 0; simulated_dtt <= max_layer; simulated_dtt++)
    {
        const LayerIndex current_layer = max_layer - simulated_dtt;
        const coord_t max_regular_radius = ceilRadius(config.getRadius(simulated_dtt, 0) + current_min_xy_dist_delta);
        const coord_t max_min_radius = ceilRadius(config.getRadius(simulated_dtt, 0)); // the maximal radius that the radius with the min_xy_dist can achieve
        const coord_t max_initial_layer_diameter_radius = ceilRadius(config.recommendedMinRadius(current_layer) + current_min_xy_dist_delta);
        if (!radius_until_layer.count(max_regular_radius))
        {
            radius_until_layer[max_regular_radius] = current_layer;
        }
        if (!radius_until_layer.count(max_min_radius))
        {
            radius_until_layer[max_min_radius] = current_layer;
        }
        if (!radius_until_layer.count(max_initial_layer_diameter_radius))
        {
            radius_until_layer[max_initial_layer_diameter_radius] = current_layer;
        }
    }

    // Copy to deque to use in parallel for later.
    std::deque<RadiusLayerPair> relevant_avoidance_radiis;
    std::deque<RadiusLayerPair> relevant_avoidance_radiis_to_model;
    relevant_avoidance_radiis.insert(relevant_avoidance_radiis.end(), radius_until_layer.begin(), radius_until_layer.end());
    relevant_avoidance_radiis_to_model.insert(relevant_avoidance_radiis_to_model.end(), radius_until_layer.begin(), radius_until_layer.end());

    // Append additional radiis needed for collision.

    radius_until_layer[ceilRadius(increase_until_radius, false)] = max_layer; // To calculate collision holefree for every radius, the collision of radius increase_until_radius will be required.
    // Collision for radius 0 needs to be calculated everywhere, as it will be used to ensure valid xy_distance in drawAreas.
    radius_until_layer[0] = max_layer;
    if (current_min_xy_dist_delta != 0)
    {
        radius_until_layer[current_min_xy_dist_delta] = max_layer;
    }

    std::deque<RadiusLayerPair> relevant_collision_radiis;
    relevant_collision_radiis.insert(relevant_collision_radiis.end(), radius_until_layer.begin(), radius_until_layer.end()); // Now that required_avoidance_limit contains the maximum of ild and regular required radius just copy.


    // ### Calculate the relevant collisions
    calculateCollision(relevant_collision_radiis);

    // calculate a separate Collisions with all holes removed. These are relevant for some avoidances that try to avoid holes (called safe)
    std::deque<RadiusLayerPair> relevant_hole_collision_radiis;
    for (RadiusLayerPair key : relevant_avoidance_radiis)
    {
        if (key.first < increase_until_radius + current_min_xy_dist_delta)
        {
            relevant_hole_collision_radiis.emplace_back(key);
        }
    }

    // ### Calculate collisions without holes, build from regular collision
    calculateCollisionHolefree(relevant_hole_collision_radiis);

    auto t_coll = std::chrono::high_resolution_clock::now();

    // ### Calculate the relevant avoidances

#pragma omp parallel // calculateAvoidance does NOT include a parallel block to enable working on multiple calculations at the same time
    {
        calculateAvoidance(relevant_avoidance_radiis);
        calculateWallRestictions(relevant_avoidance_radiis);

        if (support_rests_on_model)
        {
            calculatePlaceables(relevant_avoidance_radiis_to_model);
            calculateAvoidanceToModel(relevant_avoidance_radiis_to_model);
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto dur_col = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_coll - t_start).count();
    auto dur_avo = 0.001 * std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_coll).count();

    log("Precalculating collision took %.3lf ms. Precalculating avoidance took %.3lf ms.\n", dur_col, dur_avo);
}

const Polygons& ModelVolumes::getCollision(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    coord_t orig_radius = radius;
    std::optional<std::reference_wrapper<const Polygons>> result;
    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }

    // special case as if a radius 0 is requested it could be to ensure correct xy distance. As such it is beneficial if the collision is as close to the configured values as possible.
    if (orig_radius != 0)
    {
        radius = ceilRadius(radius);
    }
    RadiusLayerPair key{ radius, layer_idx };

#pragma omp critical(collision_cache_)
    {
        result = getArea(collision_cache_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated)
    {
        logWarning("Had to calculate collision at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", key.first, key.second);
    }
    calculateCollision(key);
    return getCollision(orig_radius, layer_idx, min_xy_dist);
}

const Polygons& ModelVolumes::getCollisionHolefree(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    coord_t orig_radius = radius;
    std::optional<std::reference_wrapper<const Polygons>> result;
    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }
    if (radius >= increase_until_radius + current_min_xy_dist_delta)
    {
        return getCollision(orig_radius, layer_idx, min_xy_dist);
    }
    RadiusLayerPair key{ radius, layer_idx };

#pragma omp critical(collision_cache_holefree_)
    {
        result = getArea(collision_cache_holefree_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated)
    {
        logWarning("Had to calculate collision holefree at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", key.first, key.second);
    }
    calculateCollisionHolefree(key);
    return getCollisionHolefree(orig_radius, layer_idx, min_xy_dist);
}

const Polygons& ModelVolumes::getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist)
{
    if (layer_idx == 0) // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
    {
        return getCollision(radius, layer_idx, min_xy_dist);
    }

    coord_t orig_radius = radius;

    std::optional<std::reference_wrapper<const Polygons>> result;

    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }
    radius = ceilRadius(radius);

    if (radius >= increase_until_radius + current_min_xy_dist_delta && type == AvoidanceType::FAST_SAFE) // no holes anymore by definition at this request
    {
        type = AvoidanceType::FAST;
    }

    const RadiusLayerPair key{ radius, layer_idx };

    std::unordered_map<RadiusLayerPair, Polygons>* cache_ptr = nullptr;
    if (!to_model && type == AvoidanceType::FAST)
    {
        cache_ptr = &avoidance_cache_;
    }
    else if (!to_model && type == AvoidanceType::SLOW)
    {
        cache_ptr = &avoidance_cache_slow_;
    }
    else if (!to_model && type == AvoidanceType::FAST_SAFE)
    {
        cache_ptr = &avoidance_cache_hole_;
    }
    else if (to_model && type == AvoidanceType::FAST)
    {
        cache_ptr = &avoidance_cache_to_model_;
    }
    else if (to_model && type == AvoidanceType::SLOW)
    {
        cache_ptr = &avoidance_cache_to_model_slow_;
    }
    else if (to_model && type == AvoidanceType::FAST_SAFE)
    {
        cache_ptr = &avoidance_cache_hole_to_model;
    }
    else
    {
        logError("Invalid Avoidance Request\n");
    }


    if (to_model)
    {
#pragma omp critical(avoidance_cache_to_model_)
        {
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            logWarning("Had to calculate Avoidance to model at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", key.first, key.second);
        }
        calculateAvoidanceToModel(key);
    }
    else
    {
#pragma omp critical(avoidance_cache_)
        {
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            logWarning("Had to calculate Avoidance at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", key.first, key.second);
        }
        calculateAvoidance(key);
    }
    return getAvoidance(orig_radius, layer_idx, type, to_model, min_xy_dist); // retrive failed and correct result was calculated. Now it has to be retrived.
}

const Polygons& ModelVolumes::getPlaceableAreas(coord_t radius, LayerIndex layer_idx)
{
    std::optional<std::reference_wrapper<const Polygons>> result;
    const coord_t orig_radius = radius;
    radius = ceilRadius(radius);
    RadiusLayerPair key{ radius, layer_idx };

#pragma omp critical(placeable_areas_cache_)
    {
        result = getArea(placeable_areas_cache_, key);
    }
    if (result)
    {
        return result.value().get();
    }
    if (precalculated)
    {
        logWarning("Had to calculate Placeable Areas at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", radius, layer_idx);
    }
    if (radius != 0)
    {
        calculatePlaceables(key);
    }
    else
    {
        getCollision(0, layer_idx, true);
    }
    return getPlaceableAreas(orig_radius, layer_idx);
}


const Polygons& ModelVolumes::getWallRestiction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist)
{
    if (layer_idx == 0) // Should never be requested as there will be no going below layer 0 ..., but just to be sure some semi-sane catch. Alternative would be empty Polygon.
    {
        return getCollision(radius, layer_idx, min_xy_dist);
    }

    coord_t orig_radius = radius;
    min_xy_dist = min_xy_dist && current_min_xy_dist_delta > 0;

    std::optional<std::reference_wrapper<const Polygons>> result;

    radius = ceilRadius(radius);
    const RadiusLayerPair key{ radius, layer_idx };

    std::unordered_map<RadiusLayerPair, Polygons>* cache_ptr;
    if (min_xy_dist)
    {
        cache_ptr = &wall_restictions_cache_min_;
    }
    else
    {
        cache_ptr = &wall_restictions_cache_;
    }


    if (min_xy_dist)
    {
#pragma omp critical(wall_restictions_cache_min_)
        {
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            logWarning("Had to calculate Wall restricions at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", key.first, key.second);
        }
    }
    else
    {
#pragma omp critical(wall_restictions_cache_)
        {
            result = getArea(*cache_ptr, key);
        }
        if (result)
        {
            return result.value().get();
        }
        if (precalculated)
        {
            logWarning("Had to calculate Wall restricions at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n", key.first, key.second);
        }
    }
    calculateWallRestictions(key);
    return getWallRestiction(orig_radius, layer_idx, min_xy_dist); // Retrieve failed and correct result was calculated. Now it has to be retrieved.
}

coord_t ModelVolumes::ceilRadius(coord_t radius, bool min_xy_dist) const
{
    if (!min_xy_dist)
    {
        radius += current_min_xy_dist_delta;
    }
    return ceilRadius(radius);
}
coord_t ModelVolumes::getRadiusNextCeil(coord_t radius, bool min_xy_dist) const
{
    coord_t ceiled_radius = ceilRadius(radius, min_xy_dist);

    if (!min_xy_dist)
        ceiled_radius -= current_min_xy_dist_delta;
    return ceiled_radius;
}

bool ModelVolumes::checkSettingsEquality(const Settings& me, const Settings& other) const
{
    return TreeSupport::TreeSupportSettings(me) == TreeSupport::TreeSupportSettings(other);
}


Polygons ModelVolumes::extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx) const
{
    constexpr bool external_polys_only = false;
    Polygons total;

    // similar to SliceDataStorage.getLayerOutlines but only for one mesh instead of for everyone

    if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
    {
        return Polygons();
    }
    const SliceLayer& layer = mesh.layers[layer_idx];

    layer.getOutlines(total, external_polys_only);
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
    {
        total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
    }
    coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
    coord_t maximum_deviation = mesh.settings.get<coord_t>("meshfix_maximum_deviation");
    total.simplify(maximum_resolution, maximum_deviation);
    return total;
}

LayerIndex ModelVolumes::getMaxCalculatedLayer(coord_t radius, const std::unordered_map<RadiusLayerPair, Polygons>& map) const
{
    coord_t max_layer = -1;

    // the placeable on model areas do not exist on layer 0, as there can not be model below it. As such it may be possible that layer 1 is available, but layer 0 does not exist.
    const RadiusLayerPair key_layer_1(radius, 1);
    if (getArea(map, key_layer_1))
    {
        max_layer = 1;
    }

    while (map.count(RadiusLayerPair(radius, max_layer + 1)))
    {
        max_layer++;
    }

    return max_layer;
}


void ModelVolumes::calculateCollision(std::deque<RadiusLayerPair> keys)
{
#pragma omp parallel for schedule(static, 1)
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
        coord_t radius = keys[i].first;
        RadiusLayerPair key(radius, 0);
        std::unordered_map<RadiusLayerPair, Polygons> data_outer;
        std::unordered_map<RadiusLayerPair, Polygons> data_placeable_outer;
        for (size_t outline_idx = 0; outline_idx < layer_outlines_.size(); outline_idx++)
        {
            std::unordered_map<RadiusLayerPair, Polygons> data;
            std::unordered_map<RadiusLayerPair, Polygons> data_placeable;

            const coord_t layer_height = layer_outlines_[outline_idx].first.get<coord_t>("layer_height");
            const bool support_rests_on_this_model = layer_outlines_[outline_idx].first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
            const coord_t z_distance_bottom = layer_outlines_[outline_idx].first.get<coord_t>("support_bottom_distance");
            const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom, layer_height);
            const coord_t z_distance_top_layers = round_up_divide(layer_outlines_[outline_idx].first.get<coord_t>("support_top_distance"), layer_height);
            const LayerIndex max_anti_overhang_layer = anti_overhang_.size() - 1;
            const LayerIndex max_required_layer = keys[i].second + std::max(coord_t(1), z_distance_top_layers);
            const coord_t xy_distance = outline_idx == current_outline_idx ? current_min_xy_dist : layer_outlines_[outline_idx].first.get<coord_t>("support_xy_distance");
            // technically this causes collision for the normal xy_distance to be larger by current_min_xy_dist_delta for all not currently processing meshes as this delta will be added at request time.
            // avoiding this would require saving each collision for each outline_idx separately.
            // and later for each avoidance... But avoidance calculation has to be for the whole scene and can NOT be done for each outline_idx separately and combined later.
            // so avoiding this inaccuracy seems infeasible as it would require 2x the avoidance calculations => 0.5x the performance.
            coord_t min_layer_bottom;
#pragma omp critical(collision_cache_)
            {
                min_layer_bottom = getMaxCalculatedLayer(radius, collision_cache_) - z_distance_bottom_layers;
            }

            if (min_layer_bottom < 0)
            {
                min_layer_bottom = 0;
            }
            for (LayerIndex layer_idx = min_layer_bottom; layer_idx <= max_required_layer; layer_idx++)
            {
                key.second = layer_idx;
                Polygons collision_areas = machine_border_;
                if (size_t(layer_idx) < layer_outlines_[outline_idx].second.size())
                {
                    collision_areas.add(layer_outlines_[outline_idx].second[layer_idx]);
                }
                collision_areas = collision_areas.offset(radius + xy_distance); // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                data[key].add(collision_areas); // if a key does not exist when it is accessed it is added!
            }


            // Add layers below, to ensure correct support_bottom_distance. Also save placeable areas of radius 0, if required for this mesh.
            for (LayerIndex layer_idx = max_required_layer; layer_idx >= min_layer_bottom; layer_idx--)
            {
                key.second = layer_idx;
                for (size_t layer_offset = 1; layer_offset <= z_distance_bottom_layers && layer_idx - coord_t(layer_offset) > min_layer_bottom; layer_offset++)
                {
                    data[key].add(data[RadiusLayerPair(radius, layer_idx - layer_offset)]);
                }
                if (support_rests_on_this_model && radius == 0 && layer_idx < coord_t(1 + keys[i].second))
                {
                    data[key] = data[key].unionPolygons();
                    Polygons above = data[RadiusLayerPair(radius, layer_idx + 1)];
                    if (max_anti_overhang_layer >= layer_idx + 1)
                    {
                        above = above.unionPolygons(anti_overhang_[layer_idx]);
                    }
                    else
                    {
                        above = above.unionPolygons(); // just to be sure the area is correctly unioned as otherwise difference may behave unexpectedly.
                    }
                    Polygons placeable = data[key].difference(above);
                    data_placeable[RadiusLayerPair(radius, layer_idx + 1)] = data_placeable[RadiusLayerPair(radius, layer_idx + 1)].unionPolygons(placeable);
                }
            }

            // Add collision layers above to ensure correct support_top_distance.
            for (LayerIndex layer_idx = min_layer_bottom; layer_idx <= max_required_layer; layer_idx++)
            {
                key.second = layer_idx;
                for (coord_t layer_offset = 1; layer_offset <= z_distance_top_layers && layer_offset + layer_idx <= max_required_layer; layer_offset++)
                {
                    data[key].add(data[RadiusLayerPair(radius, layer_idx + layer_offset)]);
                }
                if (max_anti_overhang_layer >= layer_idx)
                {
                    data[key] = data[key].unionPolygons(anti_overhang_[layer_idx].offset(radius));
                }
                else
                {
                    data[key] = data[key].unionPolygons();
                }
            }

            for (LayerIndex layer_idx = max_required_layer; layer_idx > keys[i].second; layer_idx--)
            {
                data.erase(RadiusLayerPair(radius, layer_idx)); // all these dont have the correct z_distance_top_layers as they can still have areas above them
            }

            for (auto pair : data)
            {
                pair.second.simplify(min_maximum_resolution_, min_maximum_deviation_);
                data_outer[pair.first] = data_outer[pair.first].unionPolygons(pair.second);
            }
            if (radius == 0)
            {
                for (auto pair : data_placeable)
                {
                    pair.second.simplify(min_maximum_resolution_, min_maximum_deviation_);
                    data_placeable_outer[pair.first] = data_placeable_outer[pair.first].unionPolygons(pair.second);
                }
            }
        }
#pragma omp critical(progress)
        {
            if (precalculated && precalculation_progress < PROGRESS_PRECALC_COLL)
            {
                precalculation_progress += PROGRESS_PRECALC_COLL / keys.size();
                Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, PROGRESS_TOTAL);
            }
        }
#pragma omp critical(collision_cache_)
        {
            collision_cache_.insert(data_outer.begin(), data_outer.end());
        }
        if (radius == 0)
        {
#pragma omp critical(placeable_areas_cache_)
            {
                placeable_areas_cache_.insert(data_placeable_outer.begin(), data_placeable_outer.end());
            }
        }
    }
}
void ModelVolumes::calculateCollisionHolefree(std::deque<RadiusLayerPair> keys)
{
    LayerIndex max_layer = 0;
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
        max_layer = std::max(max_layer, keys[i].second);
    }

#pragma omp parallel
    {
        std::unordered_map<RadiusLayerPair, Polygons> data;
#pragma omp for schedule(guided)
        for (coord_t layer_idx = 0; layer_idx <= max_layer; layer_idx++)
        {
            for (RadiusLayerPair key : keys)
            {
                // Logically increase the collision by increase_until_radius
                coord_t radius = key.first;
                coord_t increase_radius_ceil = ceilRadius(increase_until_radius, false) - ceilRadius(radius, true);
                Polygons col = getCollision(increase_until_radius, layer_idx, false).offset(5 - increase_radius_ceil, ClipperLib::jtRound).unionPolygons(); // this union is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
                col.simplify(min_maximum_resolution_, min_maximum_deviation_);
                data[RadiusLayerPair(radius, layer_idx)] = col;
            }
        }
#pragma omp critical(collision_cache_holefree_)
        {
            collision_cache_holefree_.insert(data.begin(), data.end());
        }
    }
}


// ensures offsets are only done in sizes with a max step size per offset while adding the collision offset after each step, this ensures that areas cannot glitch through walls defined by the collision when offsetting to fast
Polygons ModelVolumes::safeOffset(const Polygons& me, coord_t distance, ClipperLib::JoinType jt, coord_t max_safe_step_distance, const Polygons& collision) const
{
    const size_t steps = std::abs(distance / max_safe_step_distance);
    assert(distance * max_safe_step_distance >= 0);
    Polygons ret = me;

    for (size_t i = 0; i < steps; i++)
    {
        ret = ret.offset(max_safe_step_distance, jt).unionPolygons(collision);
    }
    ret = ret.offset(distance % max_safe_step_distance, jt);

    return ret.unionPolygons(collision);
}

void ModelVolumes::calculateAvoidance(std::deque<RadiusLayerPair> keys)
{
    const std::vector<AvoidanceType> all_types = { AvoidanceType::SLOW, AvoidanceType::FAST_SAFE, AvoidanceType::FAST };
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
#pragma omp for schedule(dynamic) nowait
        for (size_t type_idx = 0; type_idx < all_types.size(); type_idx++)
        {
            AvoidanceType type = all_types[type_idx];
            const bool slow = type == AvoidanceType::SLOW;
            const bool hole = type == AvoidanceType::FAST_SAFE;

            coord_t radius = keys[i].first;
            LayerIndex max_required_layer = keys[i].second;

            // do not calculate not needed safe avoidances
            if (hole && radius >= increase_until_radius + current_min_xy_dist_delta)
            {
                continue;
            }

            const coord_t offset_speed = slow ? max_move_slow_ : max_move_;
            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist * 1.9);
            RadiusLayerPair key(radius, 0);
            Polygons latest_avoidance;
            LayerIndex start_layer;
#pragma omp critical(avoidance_cache_)
            {
                start_layer = 1 + getMaxCalculatedLayer(radius, slow ? avoidance_cache_slow_ : hole ? avoidance_cache_hole_ : avoidance_cache_);
            }
            if (start_layer > max_required_layer)
            {
                logDebug("Requested calculation for value already calculated ?\n");
                continue;
            }
            start_layer = std::max(start_layer, LayerIndex(1)); // Ensure StartLayer is at least 1 as if no avoidance was calculated getMaxCalculatedLayer returns -1
            std::vector<std::pair<RadiusLayerPair, Polygons>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Polygons>(RadiusLayerPair(radius, -1), Polygons()));


            latest_avoidance = getAvoidance(radius, start_layer - 1, type, false, true); // minDist as the delta was already added, also avoidance for layer 0 will return the collision.

            // ### main loop doing the calculation
            for (LayerIndex layer = start_layer; layer <= max_required_layer; layer++)
            {
                key.second = layer;
                Polygons col;
                if ((slow && radius < increase_until_radius + current_min_xy_dist_delta) || hole)
                {
                    col = getCollisionHolefree(radius, layer, true);
                }
                else
                {
                    col = getCollision(radius, layer, true);
                }
                latest_avoidance = safeOffset(latest_avoidance, -offset_speed, ClipperLib::jtRound, -max_step_move, col);
                latest_avoidance.simplify(min_maximum_resolution_, min_maximum_deviation_);
                data[layer] = std::pair<RadiusLayerPair, Polygons>(key, latest_avoidance);
            }
#pragma omp critical(progress)
            {
                if (precalculated && precalculation_progress < PROGRESS_PRECALC_COLL + PROGRESS_PRECALC_AVO)
                {
                    precalculation_progress += support_rests_on_model ? 0.4 : 1 * PROGRESS_PRECALC_AVO / (keys.size() * 3);
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, PROGRESS_TOTAL);
                }
            }
#pragma omp critical(avoidance_cache_)
            {
                if (slow)
                {
                    avoidance_cache_slow_.insert(data.begin(), data.end());
                }
                else
                {
                    if (hole)
                    {
                        avoidance_cache_hole_.insert(data.begin(), data.end());
                    }
                    else
                    {
                        avoidance_cache_.insert(data.begin(), data.end());
                    }
                }
            }
        }
    }
}

void ModelVolumes::calculatePlaceables(std::deque<RadiusLayerPair> keys)
{
#pragma omp for schedule(static, 1)
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
        const coord_t radius = keys[i].first;
        const LayerIndex max_required_layer = keys[i].second;
        std::vector<std::pair<RadiusLayerPair, Polygons>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Polygons>(RadiusLayerPair(radius, -1), Polygons()));
        RadiusLayerPair key(radius, 0);

        LayerIndex start_layer;
#pragma omp critical(placeable_areas_cache_)
        {
            start_layer = 1 + getMaxCalculatedLayer(radius, placeable_areas_cache_);
        }
        if (start_layer > max_required_layer)
        {
            logDebug("Requested calculation for value already calculated ?\n");
            continue;
        }

        if (start_layer == 0)
        {
            data[0] = std::pair<RadiusLayerPair, Polygons>(key, machine_border_.difference(getCollision(radius, 0, true)));
            start_layer = 1;
        }

        for (LayerIndex layer = start_layer; layer <= max_required_layer; layer++)
        {
            key.second = layer;
            Polygons placeable = getPlaceableAreas(0, layer);
            placeable.simplify(min_maximum_resolution_, min_maximum_deviation_); // it is faster to do this here in each thread than once in calculateCollision.
            placeable = placeable.offset(-radius);

            data[layer] = std::pair<RadiusLayerPair, Polygons>(key, placeable);
        }
#pragma omp critical(progress)
        {
            if (precalculated && precalculation_progress < PROGRESS_PRECALC_COLL + PROGRESS_PRECALC_AVO)
            {
                precalculation_progress += 0.2 * PROGRESS_PRECALC_AVO / (keys.size());
                Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, PROGRESS_TOTAL);
            }
        }


#pragma omp critical(placeable_areas_cache_)
        {
            placeable_areas_cache_.insert(data.begin(), data.end());
        }
    }
}


void ModelVolumes::calculateAvoidanceToModel(std::deque<RadiusLayerPair> keys)
{
    const std::vector<AvoidanceType> all_types = { AvoidanceType::SLOW, AvoidanceType::FAST_SAFE, AvoidanceType::FAST };

    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
#pragma omp for schedule(dynamic) nowait
        for (size_t type_idx = 0; type_idx < all_types.size(); type_idx++)
        {
            AvoidanceType type = all_types[type_idx];
            bool slow = type == AvoidanceType::SLOW;
            bool hole = type == AvoidanceType::FAST_SAFE;
            coord_t radius = keys[i].first;
            LayerIndex max_required_layer = keys[i].second;

            // do not calculate not needed safe avoidances
            if (hole && radius >= increase_until_radius + current_min_xy_dist_delta)
            {
                continue;
            }
            getPlaceableAreas(radius, max_required_layer); // ensuring Placeableareas are calculated todo No Request layer 0
            const coord_t offset_speed = slow ? max_move_slow_ : max_move_;
            const coord_t max_step_move = std::max(1.9 * radius, current_min_xy_dist * 1.9);
            Polygons latest_avoidance;
            std::vector<std::pair<RadiusLayerPair, Polygons>> data(max_required_layer + 1, std::pair<RadiusLayerPair, Polygons>(RadiusLayerPair(radius, -1), Polygons()));
            RadiusLayerPair key(radius, 0);

            LayerIndex start_layer;
#pragma omp critical(avoidance_cache_to_model_)
            {
                start_layer = 1 + getMaxCalculatedLayer(radius, slow ? avoidance_cache_to_model_slow_ : hole ? avoidance_cache_hole_to_model : avoidance_cache_to_model_);
            }
            if (start_layer > max_required_layer)
            {
                logDebug("Requested calculation for value already calculated ?\n");
                continue;
            }
            start_layer = std::max(start_layer, LayerIndex(1));
            latest_avoidance = getAvoidance(radius, start_layer - 1, type, true, true); // minDist as the delta was already added, also avoidance for layer 0 will return the collision.

            // ### main loop doing the calculation
            for (LayerIndex layer = start_layer; layer <= max_required_layer; layer++)
            {
                key.second = layer;
                Polygons col = getCollision(radius, layer, true);

                if ((slow && radius < increase_until_radius + current_min_xy_dist_delta) || hole)
                {
                    col = getCollisionHolefree(radius, layer, true);
                }
                else
                {
                    col = getCollision(radius, layer, true);
                }

                latest_avoidance = safeOffset(latest_avoidance, -offset_speed, ClipperLib::jtRound, -max_step_move, col).difference(getPlaceableAreas(radius, layer));

                latest_avoidance.simplify(min_maximum_resolution_, min_maximum_deviation_);
                data[layer] = std::pair<RadiusLayerPair, Polygons>(key, latest_avoidance);
            }
#pragma omp critical(progress)
            {
                if (precalculated && precalculation_progress < PROGRESS_PRECALC_COLL + PROGRESS_PRECALC_AVO)
                {
                    precalculation_progress += 0.4 * PROGRESS_PRECALC_AVO / (keys.size() * 3);
                    Progress::messageProgress(Progress::Stage::SUPPORT, precalculation_progress * progress_multiplier + progress_offset, PROGRESS_TOTAL);
                }
            }
#pragma omp critical(avoidance_cache_to_model_)
            {
                if (slow)
                {
                    avoidance_cache_to_model_slow_.insert(data.begin(), data.end());
                }
                else
                {
                    if (hole)
                    {
                        avoidance_cache_hole_to_model.insert(data.begin(), data.end());
                    }
                    else
                    {
                        avoidance_cache_to_model_.insert(data.begin(), data.end());
                    }
                }
            }
        }
    }
}


void ModelVolumes::calculateWallRestictions(std::deque<RadiusLayerPair> keys)
{
#pragma omp for nowait schedule(dynamic)
    for (long long unsigned int i = 0; i < keys.size(); i++)
    {
        coord_t radius = keys[i].first;
        RadiusLayerPair key(radius, 0);
        coord_t min_layer_bottom;
        std::unordered_map<RadiusLayerPair, Polygons> data;
        std::unordered_map<RadiusLayerPair, Polygons> data_min;

#pragma omp critical(wall_restictions_cache_)
        {
            min_layer_bottom = getMaxCalculatedLayer(radius, wall_restictions_cache_);
        }

        if (min_layer_bottom < 1)
        {
            min_layer_bottom = 1;
        }
        for (LayerIndex layer_idx = min_layer_bottom; layer_idx <= keys[i].second; layer_idx++)
        {
            key.second = layer_idx;
            coord_t layer_idx_below = layer_idx - 1;
            Polygons wall_restriction = getCollision(0, layer_idx, false).intersection(getCollision(radius, layer_idx_below, true)); // radius contains current_min_xy_dist_delta already if required
            wall_restriction.simplify(min_maximum_resolution_, min_maximum_deviation_);
            data.emplace(key, wall_restriction);
            if (current_min_xy_dist_delta > 0)
            {
                Polygons wall_restriction_min = getCollision(0, layer_idx, true).intersection(getCollision(radius, layer_idx_below, true));
                wall_restriction_min.simplify(min_maximum_resolution_, min_maximum_deviation_);
                data_min.emplace(key, wall_restriction_min);
            }
        }

#pragma omp critical(wall_restictions_cache_)
        {
            wall_restictions_cache_.insert(data.begin(), data.end());
        }
#pragma omp critical(wall_restictions_cache_min_)
        {
            wall_restictions_cache_min_.insert(data_min.begin(), data_min.end());
        }
    }
}

coord_t ModelVolumes::ceilRadius(coord_t radius) const
{
    coord_t exponential_result = SUPPORT_TREE_EXPONENTIAL_THRESHOLD;
    if (radius >= SUPPORT_TREE_EXPONENTIAL_THRESHOLD && SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION)
    {
        while (exponential_result < radius || ignorable_radii_.count(exponential_result))
        {
            exponential_result = std::max(coord_t(exponential_result * SUPPORT_TREE_EXPONENTIAL_FACTOR), exponential_result + SUPPORT_TREE_COLLISION_RESOLUTION);
        }
        return exponential_result;
    }
    const auto remainder = radius % SUPPORT_TREE_COLLISION_RESOLUTION;
    const auto delta = remainder != 0 ? SUPPORT_TREE_COLLISION_RESOLUTION - remainder : 0;

    if (ignorable_radii_.count(radius + delta))
    {
        return ceilRadius(radius + delta + 1);
    }

    return radius + delta;
}

template <typename KEY>
const std::optional<std::reference_wrapper<const Polygons>> ModelVolumes::getArea(const std::unordered_map<KEY, Polygons>& cache, const KEY key) const
{
    const auto it = cache.find(key);
    if (it != cache.end())
    {
        return std::optional<std::reference_wrapper<const Polygons>>{ it->second };
    }
    else
    {
        return std::optional<std::reference_wrapper<const Polygons>>();
    }
}


Polygons ModelVolumes::calculateMachineBorderCollision(Polygon machine_border)
{
    Polygons machine_volume_border;
    machine_volume_border.add(machine_border.offset(1000000)); // Put a border of 1m around the print volume so that we don't collide.
    machine_border.reverse(); // Makes the polygon negative so that we subtract the actual volume from the collision area.
    machine_volume_border.add(machine_border);
    return machine_volume_border;
}

} // namespace cura
