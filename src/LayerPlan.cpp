//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cstring>
#include <optional>

#include "Application.h" //To communicate layer view data.
#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "PathOrderMonotonic.h" //Monotonic ordering of skin lines.
#include "raft.h" // getTotalExtraLayers
#include "Slice.h"
#include "sliceDataStorage.h"
#include "communication/Communication.h"
#include "pathPlanning/Comb.h"
#include "pathPlanning/CombPaths.h"
#include "settings/types/Ratio.h"
#include "utils/logoutput.h"
#include "utils/polygonUtils.h"
#include "utils/linearAlg2D.h"
#include "WipeScriptConfig.h"

namespace cura {

constexpr int MINIMUM_LINE_LENGTH = 5; // in uM. Generated lines shorter than this may be discarded
constexpr int MINIMUM_SQUARED_LINE_LENGTH = MINIMUM_LINE_LENGTH * MINIMUM_LINE_LENGTH;

ExtruderPlan::ExtruderPlan(const size_t extruder, const LayerIndex layer_nr, const bool is_initial_layer, const bool is_raft_layer, const coord_t layer_thickness, const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config)
: heated_pre_travel_time(0)
, required_start_temperature(-1)
, extruder_nr(extruder)
, layer_nr(layer_nr)
, is_initial_layer(is_initial_layer)
, is_raft_layer(is_raft_layer)
, layer_thickness(layer_thickness)
, fan_speed_layer_time_settings(fan_speed_layer_time_settings)
, retraction_config(retraction_config)
, extrudeSpeedFactor(1.0)
, extraTime(0.0)
, totalPrintTime(0)
{
}

void ExtruderPlan::handleInserts(unsigned int& path_idx, GCodeExport& gcode)
{
    while ( ! inserts.empty() && path_idx >= inserts.front().path_idx)
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        inserts.front().write(gcode);
        inserts.pop_front();
    }
}

void ExtruderPlan::handleAllRemainingInserts(GCodeExport& gcode)
{
    while ( ! inserts.empty() )
    { // handle the Insert to be inserted before this path_idx (and all inserts not handled yet)
        NozzleTempInsert& insert = inserts.front();
        assert(insert.path_idx == paths.size());
        insert.write(gcode);
        inserts.pop_front();
    }
}

void ExtruderPlan::setExtrudeSpeedFactor(const Ratio speed_factor)
{
    extrudeSpeedFactor = speed_factor;
}

double ExtruderPlan::getExtrudeSpeedFactor()
{
    return extrudeSpeedFactor;
}

void ExtruderPlan::setFanSpeed(double _fan_speed)
{
    fan_speed = _fan_speed;
}
double ExtruderPlan::getFanSpeed()
{
    return fan_speed;
}

void ExtruderPlan::applyBackPressureCompensation(const double back_pressure_compensation)
{
    constexpr double epsilon_speed_factor = 0.001; // Don't put on actual 'limit double minimum', because we don't want printers to stall.
    for (auto& path : paths)
    {
        const Ratio nominal_flow_for_path = path.config->getFlowRatio();
        const double nominal_width_for_path = static_cast<double>(path.config->getLineWidth());
        if (path.flow <= 0.0 || nominal_flow_for_path <= 0.0 || nominal_width_for_path <= 0.0 || path.config->isTravelPath() || path.config->isBridgePath())
        {
            continue;
        }
        const double line_width_for_path = path.flow * nominal_flow_for_path * nominal_width_for_path;
        path.speed_back_pressure_factor = std::max(epsilon_speed_factor, 1.0 + (nominal_width_for_path / line_width_for_path - 1.0) * back_pressure_compensation);
    }
}

GCodePath* LayerPlan::getLatestPathWithConfig(const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio flow, bool spiralize, const Ratio speed_factor)
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0 && paths.back().config == &config && !paths.back().done && paths.back().flow == flow && paths.back().speed_factor == speed_factor && paths.back().mesh_id == current_mesh) // spiralize can only change when a travel path is in between
    {
        return &paths.back();
    }
    paths.emplace_back(config, current_mesh, space_fill_type, flow, spiralize, speed_factor);
    GCodePath* ret = &paths.back();
    ret->skip_agressive_merge_hint = mode_skip_agressive_merge;
    return ret;
}

const Polygons* LayerPlan::getCombBoundaryInside() const
{
    return &comb_boundary_preferred;
}

void LayerPlan::forceNewPathStart()
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0)
        paths[paths.size()-1].done = true;
}

LayerPlan::LayerPlan(const SliceDataStorage& storage, LayerIndex layer_nr, coord_t z, coord_t layer_thickness, size_t start_extruder, const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, coord_t comb_boundary_offset, coord_t comb_move_inside_distance, coord_t travel_avoid_distance)
: storage(storage)
, configs_storage(storage, layer_nr, layer_thickness)
, z(z)
, final_travel_z(z)
, mode_skip_agressive_merge(false)
, layer_nr(layer_nr)
, is_initial_layer(layer_nr == 0 - static_cast<LayerIndex>(Raft::getTotalExtraLayers()))
, is_raft_layer(layer_nr < 0 - static_cast<LayerIndex>(Raft::getFillerLayerCount()))
, layer_thickness(layer_thickness)
, has_prime_tower_planned_per_extruder(Application::getInstance().current_slice->scene.extruders.size(), false)
, current_mesh("NONMESH")
, last_extruder_previous_layer(start_extruder)
, last_planned_extruder(&Application::getInstance().current_slice->scene.extruders[start_extruder])
, first_travel_destination_is_inside(false) // set properly when addTravel is called for the first time (otherwise not set properly)
, comb_boundary_minimum(computeCombBoundary(CombBoundary::MINIMUM))
, comb_boundary_preferred(computeCombBoundary(CombBoundary::PREFERRED))
, comb_move_inside_distance(comb_move_inside_distance)
, fan_speed_layer_time_settings_per_extruder(fan_speed_layer_time_settings_per_extruder)
{
    size_t current_extruder = start_extruder;
    was_inside = true; // not used, because the first travel move is bogus
    is_inside = false; // assumes the next move will not be to inside a layer part (overwritten just before going into a layer part)
    if (Application::getInstance().current_slice->scene.current_mesh_group->settings.get<CombingMode>("retraction_combing") != CombingMode::OFF)
    {
        comb = new Comb(storage, layer_nr, comb_boundary_minimum, comb_boundary_preferred, comb_boundary_offset, travel_avoid_distance, comb_move_inside_distance);
    }
    else
    {
        comb = nullptr;
    }
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice->scene.extruders)
    {
        layer_start_pos_per_extruder.emplace_back(extruder.settings.get<coord_t>("layer_start_x"), extruder.settings.get<coord_t>("layer_start_y"));
    }
    extruder_plans.reserve(Application::getInstance().current_slice->scene.extruders.size());
    extruder_plans.emplace_back(current_extruder, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[current_extruder], storage.retraction_config_per_extruder[current_extruder]);

    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
    { //Skirt and brim.
        skirt_brim_is_processed[extruder_nr] = false;
    }
}

LayerPlan::~LayerPlan()
{
    if (comb)
        delete comb;
}

ExtruderTrain* LayerPlan::getLastPlannedExtruderTrain()
{
    return last_planned_extruder;
}

Polygons LayerPlan::computeCombBoundary(const CombBoundary boundary_type)
{
    Polygons comb_boundary;
    const CombingMode mesh_combing_mode = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<CombingMode>("retraction_combing");
    if (mesh_combing_mode != CombingMode::OFF && (layer_nr >=0 || mesh_combing_mode != CombingMode::NO_SKIN))
    {
        if (layer_nr < 0)
        {
            comb_boundary = storage.raftOutline.offset(MM2INT(0.1));
        }
        else
        {
            for (const SliceMeshStorage& mesh : storage.meshes)
            {
                const SliceLayer& layer = mesh.layers[static_cast<size_t>(layer_nr)];
                // don't process infill_mesh or anti_overhang_mesh
                if (mesh.settings.get<bool>("infill_mesh") && mesh.settings.get<bool>("anti_overhang_mesh"))
                {
                    continue;
                }
                const CombingMode combing_mode = mesh.settings.get<CombingMode>("retraction_combing");
                const coord_t inner_walls_offset = boundary_type == CombBoundary::MINIMUM
                                                       ? 0
                                                       : mesh.settings.get<coord_t>("wall_line_width_x")
                                                             * (mesh.settings.get<size_t>("wall_line_count") - 1)
                                                             / 4;
                const coord_t offset = -10 - mesh.settings.get<coord_t>("wall_line_width_0") - inner_walls_offset;

                for (const SliceLayerPart& part : layer.parts)
                {
                    if (combing_mode == CombingMode::ALL) // Add the increased outline offset (skin, infill and part of the inner walls)
                    {
                        comb_boundary.add(part.outline.offset(offset));
                    }
                    else if (combing_mode == CombingMode::NO_SKIN) // Add the increased outline offset, subtract skin (infill and part of the inner walls)
                    {
                        comb_boundary.add(part.outline.offset(offset).difference(part.inner_area.difference(part.infill_area)));
                    }
                    else if (combing_mode == CombingMode::INFILL) // Add the infill (infill only)
                    {
                        comb_boundary.add(part.infill_area);
                    }
                }
            }
        }
    }
    return comb_boundary;
}

void LayerPlan::setIsInside(bool _is_inside)
{
    is_inside = _is_inside;
}

bool LayerPlan::setExtruder(const size_t extruder_nr)
{
    if (extruder_nr == getExtruder())
    {
        return false;
    }
    setIsInside(false);
    { // handle end position of the prev extruder
        ExtruderTrain* extruder = getLastPlannedExtruderTrain();
        const bool end_pos_absolute = extruder->settings.get<bool>("machine_extruder_end_pos_abs");
        Point end_pos(extruder->settings.get<coord_t>("machine_extruder_end_pos_x"), extruder->settings.get<coord_t>("machine_extruder_end_pos_y"));
        if (!end_pos_absolute)
        {
            end_pos += getLastPlannedPositionOrStartingPosition();
        }
        else 
        {
            const Point extruder_offset(extruder->settings.get<coord_t>("machine_nozzle_offset_x"), extruder->settings.get<coord_t>("machine_nozzle_offset_y"));
            end_pos += extruder_offset; // absolute end pos is given as a head position
        }
        if (end_pos_absolute || last_planned_position)
        {
            addTravel(end_pos); //  + extruder_offset cause it
        }
    }
    if (extruder_plans.back().paths.empty() && extruder_plans.back().inserts.empty())
    { // first extruder plan in a layer might be empty, cause it is made with the last extruder planned in the previous layer
        extruder_plans.back().extruder_nr = extruder_nr;
    }
    extruder_plans.emplace_back(extruder_nr, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[extruder_nr], storage.retraction_config_per_extruder[extruder_nr]);
    assert(extruder_plans.size() <= Application::getInstance().current_slice->scene.extruders.size() && "Never use the same extruder twice on one layer!");
    last_planned_extruder = &Application::getInstance().current_slice->scene.extruders[extruder_nr];

    { // handle starting pos of the new extruder
        ExtruderTrain* extruder = getLastPlannedExtruderTrain();
        const bool start_pos_absolute = extruder->settings.get<bool>("machine_extruder_start_pos_abs");
        Point start_pos(extruder->settings.get<coord_t>("machine_extruder_start_pos_x"), extruder->settings.get<coord_t>("machine_extruder_start_pos_y"));
        if (!start_pos_absolute)
        {
            start_pos += getLastPlannedPositionOrStartingPosition();
        }
        else 
        {
            Point extruder_offset(extruder->settings.get<coord_t>("machine_nozzle_offset_x"), extruder->settings.get<coord_t>("machine_nozzle_offset_y"));
            start_pos += extruder_offset; // absolute start pos is given as a head position
        }
        if (start_pos_absolute || last_planned_position)
        {
            last_planned_position = start_pos;
        }
    }
    return true;
}
void LayerPlan::setMesh(const std::string mesh_id)
{
    current_mesh = mesh_id;
}

void LayerPlan::moveInsideCombBoundary(const coord_t distance)
{
    constexpr coord_t max_dist2 = MM2INT(2.0) * MM2INT(2.0); // if we are further than this distance, we conclude we are not inside even though we thought we were.
    // this function is to be used to move from the boudary of a part to inside the part
    Point p = getLastPlannedPositionOrStartingPosition(); // copy, since we are going to move p
    if (PolygonUtils::moveInside(comb_boundary_preferred, p, distance, max_dist2) != NO_INDEX)
    {
        //Move inside again, so we move out of tight 90deg corners
        PolygonUtils::moveInside(comb_boundary_preferred, p, distance, max_dist2);
        if (comb_boundary_preferred.inside(p))
        {
            addTravel_simple(p);
            //Make sure the that any retraction happens after this move, not before it by starting a new move path.
            forceNewPathStart();
        }
    }
}

bool LayerPlan::getPrimeTowerIsPlanned(unsigned int extruder_nr) const
{
    return has_prime_tower_planned_per_extruder[extruder_nr];
}

void LayerPlan::setPrimeTowerIsPlanned(unsigned int extruder_nr)
{
    has_prime_tower_planned_per_extruder[extruder_nr] = true;
}

std::optional<std::pair<Point, bool>> LayerPlan::getFirstTravelDestinationState() const
{
    std::optional<std::pair<Point, bool>> ret;
    if (first_travel_destination)
    {
        ret = std::make_pair(*first_travel_destination, first_travel_destination_is_inside);
    }
    return ret;
}

GCodePath& LayerPlan::addTravel(const Point p, const bool force_retract)
{
    const GCodePathConfig& travel_config = configs_storage.travel_config_per_extruder[getExtruder()];
    const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[getExtruder()];

    GCodePath* path = getLatestPathWithConfig(travel_config, SpaceFillType::None);

    bool combed = false;

    const ExtruderTrain* extruder = getLastPlannedExtruderTrain();

    const bool is_first_travel_of_extruder_after_switch = extruder_plans.back().paths.size() == 1 && (extruder_plans.size() > 1 || last_extruder_previous_layer != getExtruder());
    bool bypass_combing = is_first_travel_of_extruder_after_switch && extruder->settings.get<bool>("retraction_hop_after_extruder_switch");

    const bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
    const bool retraction_enable = extruder->settings.get<bool>("retraction_enable");
    if (is_first_travel_of_layer)
    {
        bypass_combing = true; // first travel move is bogus; it is added after this and the previous layer have been planned in LayerPlanBuffer::addConnectingTravelMove
        first_travel_destination = p;
        first_travel_destination_is_inside = is_inside;
        if (layer_nr == 0 && retraction_enable && extruder->settings.get<bool>("retraction_hop_enabled"))
        {
            path->retract = true;
            path->perform_z_hop = true;
        }
        forceNewPathStart(); // force a new travel path after this first bogus move
    }
    else if (force_retract && last_planned_position && !shorterThen(*last_planned_position - p, retraction_config.retraction_min_travel_distance))
    {
        // path is not shorter than min travel distance, force a retraction
        path->retract = true;
        if (comb == nullptr)
        {
            path->perform_z_hop = extruder->settings.get<bool>("retraction_hop_enabled");
        }
    }

    if (comb != nullptr && !bypass_combing)
    {
        CombPaths combPaths;

        // Divide by 2 to get the radius
        // Multiply by 2 because if two lines start and end points places very close then will be applied combing with retractions. (Ex: for brim)
        const coord_t max_distance_ignored = extruder->settings.get<coord_t>("machine_nozzle_tip_outer_diameter") / 2 * 2;

        bool unretract_before_last_travel_move = false; // Decided when calculating the combing
        combed = comb->calc(*extruder, *last_planned_position, p, combPaths, was_inside, is_inside, max_distance_ignored, unretract_before_last_travel_move);
        if (combed)
        {
            bool retract = path->retract || (combPaths.size() > 1 && retraction_enable);
            if (!retract)
            { // check whether we want to retract
                if (combPaths.throughAir)
                {
                    retract = retraction_enable;
                }
                else
                {
                    for (CombPath& combPath : combPaths)
                    { // retract when path moves through a boundary
                        if (combPath.cross_boundary)
                        {
                            retract = retraction_enable;
                            break;
                        }
                    }
                }
                if (combPaths.size() == 1)
                {
                    CombPath comb_path = combPaths[0];
                    if (extruder->settings.get<bool>("limit_support_retractions") &&
                        combPaths.throughAir && !comb_path.cross_boundary && comb_path.size() == 2 && comb_path[0] == *last_planned_position && comb_path[1] == p)
                    { // limit the retractions from support to support, which didn't cross anything
                        retract = false;
                    }
                }
            }

            const coord_t maximum_travel_resolution = extruder->settings.get<coord_t>("meshfix_maximum_travel_resolution");
            coord_t distance = 0;
            Point last_point((last_planned_position) ? *last_planned_position : Point(0, 0));
            for (CombPath& combPath : combPaths)
            { // add all comb paths (don't do anything special for paths which are moving through air)
                if (combPath.empty())
                {
                    continue;
                }
                for (Point& comb_point : combPath)
                {
                    if (path->points.empty() || vSize2(path->points.back() - comb_point) > maximum_travel_resolution * maximum_travel_resolution)
                    {
                        path->points.push_back(comb_point);
                        distance += vSize(last_point - comb_point);
                        last_point = comb_point;
                    }
                }
                distance += vSize(last_point - p);
                const coord_t retract_threshold = extruder->settings.get<coord_t>("retraction_combing_max_distance");
                path->retract = retract || (retract_threshold > 0 && distance > retract_threshold && retraction_enable);
                // don't perform a z-hop
            }
            // Whether to unretract before the last travel move of the travel path, which comes before the wall to be printed.
            // This should be true when traveling towards an outer wall to make sure that the unretraction will happen before the
            // last travel move BEFORE going to that wall. This way, the nozzle doesn't sit still on top of the outer wall's
            // path while it is unretracting, avoiding possible blips.
            path->unretract_before_last_travel_move = path->retract && unretract_before_last_travel_move;
        }
    }

    // CURA-6675:
    // Retraction Minimal Travel Distance should work for all travel moves. If the travel move is shorter than the
    // Retraction Minimal Travel Distance, retraction should be disabled.
    if (!is_first_travel_of_layer && last_planned_position && shorterThen(*last_planned_position - p, retraction_config.retraction_min_travel_distance))
    {
        path->retract = false;
        path->perform_z_hop = false;
    }

    // no combing? retract only when path is not shorter than minimum travel distance
    if (!combed && !is_first_travel_of_layer && last_planned_position && !shorterThen(*last_planned_position - p, retraction_config.retraction_min_travel_distance))
    {
        if (was_inside) // when the previous location was from printing something which is considered inside (not support or prime tower etc)
        {               // then move inside the printed part, so that we don't ooze on the outer wall while retraction, but on the inside of the print.
            assert (extruder != nullptr);
            coord_t innermost_wall_line_width = extruder->settings.get<coord_t>((extruder->settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
            if (layer_nr == 0)
            {
                innermost_wall_line_width *= extruder->settings.get<Ratio>("initial_layer_line_width_factor");
            }
            moveInsideCombBoundary(innermost_wall_line_width);
        }
        path->retract = retraction_enable;
        path->perform_z_hop = retraction_enable && extruder->settings.get<bool>("retraction_hop_enabled");
    }

    // must start new travel path as retraction can be enabled or not depending on path length, etc.
    forceNewPathStart();

    GCodePath& ret = addTravel_simple(p, path);
    was_inside = is_inside;
    return ret;
}

GCodePath& LayerPlan::addTravel_simple(Point p, GCodePath* path)
{
    bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
    if (is_first_travel_of_layer)
    { // spiralize calls addTravel_simple directly as the first travel move in a layer
        first_travel_destination = p;
        first_travel_destination_is_inside = is_inside;
    }
    if (path == nullptr)
    {
        path = getLatestPathWithConfig(configs_storage.travel_config_per_extruder[getExtruder()], SpaceFillType::None);
    }
    path->points.push_back(p);
    last_planned_position = p;
    return *path;
}

void LayerPlan::planPrime(const float& prime_blob_wipe_length)
{
    forceNewPathStart();
    GCodePath& prime_travel = addTravel_simple(getLastPlannedPositionOrStartingPosition() + Point(0, MM2INT(prime_blob_wipe_length)));
    prime_travel.retract = false;
    prime_travel.perform_z_hop = false;
    prime_travel.perform_prime = true;
    forceNewPathStart();
}

void LayerPlan::addExtrusionMove(Point p, const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio& flow, bool spiralize, Ratio speed_factor, double fan_speed)
{
    GCodePath* path = getLatestPathWithConfig(config, space_fill_type, flow, spiralize, speed_factor);
    path->points.push_back(p);
    path->setFanSpeed(fan_speed);
    last_planned_position = p;
}

void LayerPlan::addPolygon(ConstPolygonRef polygon, int start_idx, const bool backwards, const GCodePathConfig& config, coord_t wall_0_wipe_dist, bool spiralize, const Ratio& flow_ratio, bool always_retract)
{
    Point p0 = polygon[start_idx];
    addTravel(p0, always_retract);
    const int direction = backwards ? -1 : 1;
    for(size_t point_idx = 1; point_idx < polygon.size(); point_idx++)
    {
        Point p1 = polygon[(start_idx + point_idx * direction + polygon.size()) % polygon.size()];
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow_ratio, spiralize);
        p0 = p1;
    }
    if(polygon.size() > 2)
    {
        const Point& p1 = polygon[start_idx];
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow_ratio, spiralize);

        if(wall_0_wipe_dist > 0)
        { // apply outer wall wipe
            p0 = polygon[start_idx];
            int distance_traversed = 0;
            for(size_t point_idx = 1; ; point_idx++)
            {
                Point p1 = polygon[(start_idx + point_idx * direction + polygon.size()) % polygon.size()];
                int p0p1_dist = vSize(p1 - p0);
                if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
                {
                    Point vector = p1 - p0;
                    Point half_way = p0 + normal(vector, wall_0_wipe_dist - distance_traversed);
                    addTravel_simple(half_way);
                    break;
                }
                else
                {
                    addTravel_simple(p1);
                    distance_traversed += p0p1_dist;
                }
                p0 = p1;
            }
            forceNewPathStart();
        }
    }
    else 
    {
        logWarning("WARNING: line added as polygon! (LayerPlan)\n");
    }
}

void LayerPlan::addPolygonsByOptimizer(const Polygons& polygons, const GCodePathConfig& config, const ZSeamConfig& z_seam_config, coord_t wall_0_wipe_dist, bool spiralize, const Ratio flow_ratio, bool always_retract, bool reverse_order, const std::optional<Point> start_near_location)
{
    if(polygons.empty())
    {
        return;
    }
    PathOrderOptimizer<ConstPolygonRef> orderOptimizer(start_near_location ? start_near_location.value() : getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for(size_t poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(polygons[poly_idx]);
    }
    orderOptimizer.optimize();

    if(!reverse_order)
    {
        for(const PathOrderOptimizer<ConstPolygonRef>::Path& path : orderOptimizer.paths)
        {
            addPolygon(*path.vertices, path.start_vertex, path.backwards, config, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
        }
    }
    else
    {
        for(int index = orderOptimizer.paths.size() - 1; index >= 0; --index)
        {
            const PathOrderOptimizer<ConstPolygonRef>::Path& path = orderOptimizer.paths[index];
            addPolygon(*path.vertices, path.start_vertex, path.backwards, config, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
        }
    }
}

static constexpr float max_non_bridge_line_volume = MM2INT(100); // limit to accumulated "volume" of non-bridge lines which is proportional to distance x extrusion rate

void LayerPlan::addWallLine(const Point& p0, const Point& p1, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, float flow, float& non_bridge_line_volume, Ratio speed_factor, double distance_to_bridge_start)
{
    const coord_t min_line_len = 5; // we ignore lines less than 5um long
    const double acceleration_segment_len = MM2INT(1); // accelerate using segments of this length
    const double acceleration_factor = 0.75; // must be < 1, the larger the value, the slower the acceleration
    const bool spiralize = false;

    const coord_t min_bridge_line_len = settings.get<coord_t>("bridge_wall_min_length");
    const Ratio bridge_wall_coast = settings.get<Ratio>("bridge_wall_coast");
    const Ratio overhang_speed_factor = settings.get<Ratio>("wall_overhang_speed_factor");

    Point cur_point = p0;

    // helper function to add a single non-bridge line

    // If the line precedes a bridge line, it may be coasted to reduce the nozzle pressure before the bridge is reached

    // alternatively, if the line follows a bridge line, it may be segmented and the print speed gradually increased to reduce under-extrusion

    auto addNonBridgeLine = [&](const Point& line_end)
    {
        coord_t distance_to_line_end = vSize(cur_point - line_end);

        while (distance_to_line_end > min_line_len)
        {
            // if we are accelerating after a bridge line, the segment length is less than the whole line length
            Point segment_end = (speed_factor == 1 || distance_to_line_end < acceleration_segment_len) ? line_end : cur_point + (line_end - cur_point) * acceleration_segment_len / distance_to_line_end;

            // flow required for the next line segment - when accelerating after a bridge segment, the flow is increased in inverse proportion to the speed_factor
            // so the slower the feedrate, the greater the flow - the idea is to get the extruder back to normal pressure as quickly as possible
            const float segment_flow = (speed_factor < 1) ? flow * (1 / speed_factor) : flow;

            // if a bridge is present in this wall, this particular segment may need to be partially or wholely coasted
            if (distance_to_bridge_start > 0)
            {
                // speed_flow_factor approximates how the extrusion rate alters between the non-bridge wall line and the following bridge wall line
                // if the extrusion rates are the same, its value will be 1, if the bridge config extrusion rate is < the non-bridge config extrusion rate, the value is < 1

                const Ratio speed_flow_factor((bridge_config.getSpeed() * bridge_config.getFlowRatio()) / (non_bridge_config.getSpeed() * non_bridge_config.getFlowRatio()));

                // coast distance is proportional to distance, speed and flow of non-bridge segments just printed and is throttled by speed_flow_factor
                const double coast_dist = std::min(non_bridge_line_volume, max_non_bridge_line_volume) * (1 - speed_flow_factor) * bridge_wall_coast / 40;

                if ((distance_to_bridge_start - distance_to_line_end) <= coast_dist)
                {
                    // coast takes precedence over acceleration
                    segment_end = line_end;
                }

                const coord_t len = vSize(cur_point - segment_end);
                if (coast_dist > 0 && ((distance_to_bridge_start - len) <= coast_dist))
                {
                    if ((len - coast_dist) > min_line_len)
                    {
                        // segment is longer than coast distance so extrude using non-bridge config to start of coast
                        addExtrusionMove(segment_end + coast_dist * (cur_point - segment_end) / len, non_bridge_config, SpaceFillType::Polygons, segment_flow, spiralize, speed_factor);
                    }
                    // then coast to start of bridge segment
                    addExtrusionMove(segment_end, non_bridge_config, SpaceFillType::Polygons, 0, spiralize, speed_factor);
                }
                else
                {
                    // no coasting required, just normal segment using non-bridge config
                    addExtrusionMove(segment_end, non_bridge_config, SpaceFillType::Polygons, segment_flow, spiralize,
                        (overhang_mask.empty() || (!overhang_mask.inside(p0, true) && !overhang_mask.inside(p1, true))) ? speed_factor : overhang_speed_factor);
                }

                distance_to_bridge_start -= len;
            }
            else
            {
                // no coasting required, just normal segment using non-bridge config
                addExtrusionMove(segment_end, non_bridge_config, SpaceFillType::Polygons, segment_flow, spiralize,
                    (overhang_mask.empty() || (!overhang_mask.inside(p0, true) && !overhang_mask.inside(p1, true))) ? speed_factor : overhang_speed_factor);
            }
            non_bridge_line_volume += vSize(cur_point - segment_end) * segment_flow * speed_factor * non_bridge_config.getSpeed();
            cur_point = segment_end;
            speed_factor = 1 - (1 - speed_factor) * acceleration_factor;
            if (speed_factor >= 0.9)
            {
                speed_factor = 1;
            }
            distance_to_line_end = vSize(cur_point - line_end);
        }
    };

    if (bridge_wall_mask.empty())
    {
        // no bridges required
        addExtrusionMove(p1, non_bridge_config, SpaceFillType::Polygons, flow, spiralize,
            (overhang_mask.empty() || (!overhang_mask.inside(p0, true) && !overhang_mask.inside(p1, true))) ? 1.0_r : overhang_speed_factor);
    }
    else
    {
        // bridges may be required
        if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask, p0, p1))
        {
            // the line crosses the boundary between supported and non-supported regions so one or more bridges are required

            // determine which segments of the line are bridges

            Polygon line_poly;
            line_poly.add(p0);
            line_poly.add(p1);
            Polygons line_polys;
            line_polys.add(line_poly);
            line_polys = bridge_wall_mask.intersectionPolyLines(line_polys);

            // line_polys now contains the wall lines that need to be printed using bridge_config

            while (line_polys.size() > 0)
            {
                // find the bridge line segment that's nearest to the current point
                int nearest = 0;
                float smallest_dist2 = vSize2f(cur_point - line_polys[0][0]);
                for(unsigned i = 1; i < line_polys.size(); ++i)
                {
                    float dist2 = vSize2f(cur_point - line_polys[i][0]);
                    if (dist2 < smallest_dist2)
                    {
                        nearest = i;
                        smallest_dist2 = dist2;
                    }
                }
                ConstPolygonRef bridge = line_polys[nearest];

                // set b0 to the nearest vertex and b1 the furthest
                Point b0 = bridge[0];
                Point b1 = bridge[1];

                if (vSize2f(cur_point - b1) < vSize2f(cur_point - b0))
                {
                    // swap vertex order
                    b0 = bridge[1];
                    b1 = bridge[0];
                }

                // extrude using non_bridge_config to the start of the next bridge segment

                addNonBridgeLine(b0);

                const double bridge_line_len = vSize(b1 - cur_point);

                if (bridge_line_len >= min_bridge_line_len)
                {
                    // extrude using bridge_config to the end of the next bridge segment

                    if (bridge_line_len > min_line_len)
                    {
                        addExtrusionMove(b1, bridge_config, SpaceFillType::Polygons, flow);
                        non_bridge_line_volume = 0;
                        cur_point = b1;
                        // after a bridge segment, start slow and accelerate to avoid under-extrusion due to extruder lag
                        speed_factor = std::max(std::min(Ratio(bridge_config.getSpeed() / non_bridge_config.getSpeed()), 1.0_r), 0.5_r);
                    }
                }
                else
                {
                    // treat the short bridge line just like a normal line

                    addNonBridgeLine(b1);
                }

                // finished with this segment
                line_polys.remove(nearest);
            }

            // if we haven't yet reached p1, fill the gap with non_bridge_config line
            addNonBridgeLine(p1);
        }
        else if (bridge_wall_mask.inside(p0, true) && vSize(p0 - p1) >= min_bridge_line_len)
        {
            // both p0 and p1 must be above air (the result will be ugly!)
            addExtrusionMove(p1, bridge_config, SpaceFillType::Polygons, flow);
            non_bridge_line_volume = 0;
        }
        else
        {
            // no part of the line is above air or the line is too short to print as a bridge line
            addNonBridgeLine(p1);
        }
    }
}

void LayerPlan::addWall(ConstPolygonRef wall, int start_idx, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
    //TODO: Deprecated in favor of ExtrusionJunction version below.
    if (wall.size() < 3)
    {
        logWarning("WARNING: point, or line added as (polygon) wall sequence! (LayerPlan)\n");
    }

    constexpr size_t dummy_perimeter_id = 0;  // <-- Here, don't care about which perimeter any more.
    const coord_t nominal_line_width = non_bridge_config.getLineWidth();  // <-- The line width which it's 'supposed to' be will be used to adjust the flow ratio each time, this'll give a flow-ratio-multiplier of 1.

    LineJunctions ewall;
    std::for_each(wall.begin(), wall.end(), [&dummy_perimeter_id, &nominal_line_width, &ewall](const Point& p)
    {
        ewall.emplace_back(p, nominal_line_width, dummy_perimeter_id);
    });
    ewall.emplace_back(*wall.begin(), nominal_line_width, dummy_perimeter_id);
    constexpr bool is_closed = true;
    constexpr bool is_reversed = false;
    constexpr bool is_linked_path = false;
    addWall(ewall, start_idx, settings, non_bridge_config, bridge_config, wall_0_wipe_dist, flow_ratio, always_retract, is_closed, is_reversed, is_linked_path);
}

void LayerPlan::addWall(const LineJunctions& wall, int start_idx, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract, const bool is_closed, const bool is_reversed, const bool is_linked_path)
{
    if (wall.empty())
    {
        return;
    }
    if(is_closed)
    {
        // make sure wall start point is not above air!
        start_idx = locateFirstSupportedVertex(wall, start_idx);
    }

    float non_bridge_line_volume = max_non_bridge_line_volume; // assume extruder is fully pressurised before first non-bridge line is output
    double speed_factor = 1.0; // start first line at normal speed
    coord_t distance_to_bridge_start = 0; // will be updated before each line is processed

    const coord_t min_bridge_line_len = settings.get<coord_t>("bridge_wall_min_length");
    const coord_t small_feature_max_length = settings.get<coord_t>("small_feature_max_length");
    const bool is_small_feature = (small_feature_max_length > 0) && cura::shorterThan(wall, small_feature_max_length);
    Ratio small_feature_speed_factor = settings.get<Ratio>((layer_nr == 0) ? "small_feature_speed_factor_0" : "small_feature_speed_factor");
    const Velocity min_speed = fan_speed_layer_time_settings_per_extruder[getLastPlannedExtruderTrain()->extruder_nr].cool_min_speed;
    small_feature_speed_factor = std::max((double)small_feature_speed_factor, (double)(min_speed / non_bridge_config.getSpeed()));

    const Ratio nominal_line_width_multiplier = 1.0 / Ratio(non_bridge_config.getLineWidth()); // we multiply the flow with the actual wanted line width (for that junction), and then multiply with this

    // helper function to calculate the distance from the start of the current wall line to the first bridge segment

    auto computeDistanceToBridgeStart = [&](unsigned current_index)
    {
        distance_to_bridge_start = 0;

        if (!bridge_wall_mask.empty())
        {
            // there is air below the part so iterate through the lines that have not yet been output accumulating the total distance to the first bridge segment
            for (unsigned point_idx = current_index; point_idx < wall.size(); ++point_idx)
            {
                const ExtrusionJunction& p0 = wall[point_idx];
                const ExtrusionJunction& p1 = wall[(point_idx + 1) % wall.size()];

                if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask, p0.p, p1.p))
                {
                    // the line crosses the boundary between supported and non-supported regions so it will contain one or more bridge segments

                    // determine which segments of the line are bridges

                    Polygon line_poly;
                    line_poly.add(p0.p);
                    line_poly.add(p1.p);
                    Polygons line_polys;
                    line_polys.add(line_poly);
                    line_polys = bridge_wall_mask.intersectionPolyLines(line_polys);

                    while (line_polys.size() > 0)
                    {
                        // find the bridge line segment that's nearest to p0
                        int nearest = 0;
                        float smallest_dist2 = vSize2f(p0.p - line_polys[0][0]);
                        for(unsigned i = 1; i < line_polys.size(); ++i)
                        {
                            float dist2 = vSize2f(p0.p - line_polys[i][0]);
                            if (dist2 < smallest_dist2)
                            {
                                nearest = i;
                                smallest_dist2 = dist2;
                            }
                        }
                        ConstPolygonRef bridge = line_polys[nearest];

                        // set b0 to the nearest vertex and b1 the furthest
                        Point b0 = bridge[0];
                        Point b1 = bridge[1];

                        if (vSize2f(p0.p - b1) < vSize2f(p0.p - b0))
                        {
                            // swap vertex order
                            b0 = bridge[1];
                            b1 = bridge[0];
                        }

                        distance_to_bridge_start += vSize(b0 - p0.p);

                        const double bridge_line_len = vSize(b1 - b0);

                        if (bridge_line_len >= min_bridge_line_len)
                        {
                            // job done, we have found the first bridge line
                            return;
                        }

                        distance_to_bridge_start += bridge_line_len;

                        // finished with this segment
                        line_polys.remove(nearest);
                    }
                }
                else if (!bridge_wall_mask.inside(p0.p, true))
                {
                    // none of the line is over air
                    distance_to_bridge_start += vSize(p1.p - p0.p);
                }
            }

            // we have got all the way to the end of the wall without finding a bridge segment so disable coasting by setting distance_to_bridge_start back to 0

            distance_to_bridge_start = 0;
        }
    };

    bool first_line = true;

    ExtrusionJunction p0 = wall[start_idx];

    const int direction = is_reversed ? -1 : 1;
    const size_t max_index = is_closed ? wall.size() + 1 : wall.size();
    for(size_t point_idx = 1; point_idx < max_index; point_idx++)
    {
        const ExtrusionJunction& p1 = wall[(wall.size() + start_idx + point_idx * direction) % wall.size()];
        const coord_t line_width = (p0.w + p1.w) / 2; //For lines which in itself vary in width, use the average width of the variable line.

        if (!bridge_wall_mask.empty())
        {
            computeDistanceToBridgeStart((wall.size() + start_idx + point_idx * direction - 1) % wall.size());
        }

        if (first_line)
        {
            addTravel(p0.p, always_retract);
            first_line = false;
        }
        if (is_small_feature)
        {
            constexpr bool spiralize = false;
            addExtrusionMove(p1.p, non_bridge_config, SpaceFillType::Polygons, flow_ratio * (line_width * nominal_line_width_multiplier), spiralize, small_feature_speed_factor);
        }
        else
        {
            addWallLine(p0.p, p1.p, settings, non_bridge_config, bridge_config, flow_ratio * (line_width * nominal_line_width_multiplier), non_bridge_line_volume, speed_factor, distance_to_bridge_start);
        }

        p0 = p1;
    }

    if (wall.size() >= 2)
    {
        if (!bridge_wall_mask.empty())
        {
            computeDistanceToBridgeStart((start_idx + wall.size() - 1) % wall.size());
        }

        if (wall_0_wipe_dist > 0 && !is_linked_path)
        { // apply outer wall wipe
            p0 = wall[start_idx];
            int distance_traversed = 0;
            for (unsigned int point_idx = 1; ; point_idx++)
            {
                if(point_idx > wall.size() && distance_traversed == 0) //Wall has a total circumference of 0. This loop would never end.
                {
                    break; //No wipe if the wall has no circumference.
                }
                ExtrusionJunction p1 = wall[(start_idx + point_idx) % wall.size()];
                int p0p1_dist = vSize(p1 - p0);
                if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
                {
                    Point vector = p1.p - p0.p;
                    Point half_way = p0.p + normal(vector, wall_0_wipe_dist - distance_traversed);
                    addTravel_simple(half_way);
                    break;
                }
                else
                {
                    addTravel_simple(p1.p);
                    distance_traversed += p0p1_dist;
                }
                p0 = p1;
            }
            forceNewPathStart();
        }
    }
    else
    {
        logWarning("WARNING: point added as wall sequence! (LayerPlan)\n");
    }
}

void LayerPlan::addInfillWall(const LineJunctions& wall, const GCodePathConfig& path_config,
                        bool force_retract)
{
    assert(("All empty walls should have been filtered at this stage", !wall.empty()));
    ExtrusionJunction junction{*wall.begin()};
    addTravel(junction.p, force_retract);

    for (const auto &junction_n : wall)
    {
        const double flow = junction_n.w / Ratio(path_config.getLineWidth());
        constexpr SpaceFillType space_fill_type = SpaceFillType::Polygons;
        addExtrusionMove(junction_n.p, path_config, space_fill_type, flow);
        junction = junction_n;
    }
}

void LayerPlan::addWalls(const Polygons& walls, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, const ZSeamConfig& z_seam_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
    //TODO: Deprecated in favor of ExtrusionJunction version below.
    PathOrderOptimizer<ConstPolygonRef> orderOptimizer(getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for(size_t poly_idx = 0; poly_idx < walls.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(walls[poly_idx]);
    }
    orderOptimizer.optimize();
    for(const PathOrderOptimizer<ConstPolygonRef>::Path& path : orderOptimizer.paths)
    {
        addWall(*path.vertices, path.start_vertex, settings, non_bridge_config, bridge_config, wall_0_wipe_dist, flow_ratio, always_retract);
    }
}

void LayerPlan::addWalls(const PathJunctions& walls, const Settings& settings, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, const ZSeamConfig& z_seam_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
    constexpr bool detect_loops = true;
    PathOrderOptimizer<const LineJunctions*> order_optimizer(getLastPlannedPositionOrStartingPosition(), z_seam_config, detect_loops);
    for(const LineJunctions& wall : walls)
    {
        order_optimizer.addPolyline(&wall);
    }

    cura::Point p_end {0, 0};
    order_optimizer.optimize();
    for(const PathOrderOptimizer<const LineJunctions*>::Path& path : order_optimizer.paths)
    {
        p_end = path.backwards ? path.vertices->back().p : path.vertices->front().p;
        const cura::Point p_start = path.backwards ? path.vertices->front().p : path.vertices->back().p;
        const bool linked_path = p_start != p_end;
        addWall(*path.vertices, path.start_vertex, settings, non_bridge_config, bridge_config, wall_0_wipe_dist, flow_ratio, always_retract, path.is_closed, path.backwards, linked_path);
    }
}

void LayerPlan::addLinesByOptimizer(const Polygons& polygons, const GCodePathConfig& config, SpaceFillType space_fill_type, bool enable_travel_optimization, int wipe_dist, float flow_ratio, std::optional<Point> near_start_location, double fan_speed)
{
    Polygons boundary;
    if (enable_travel_optimization && !comb_boundary_minimum.empty())
    {
        // use the combing boundary inflated so that all infill lines are inside the boundary
        int dist = 0;
        if (layer_nr >= 0)
        {
            // determine how much the skin/infill lines overlap the combing boundary
            for (const SliceMeshStorage& mesh : storage.meshes)
            {
                const coord_t overlap = std::max(mesh.settings.get<coord_t>("skin_overlap_mm"), mesh.settings.get<coord_t>("infill_overlap_mm"));
                if (overlap > dist)
                {
                    dist = overlap;
                }
            }
            dist += 100; // ensure boundary is slightly outside all skin/infill lines
        }
        boundary.add(comb_boundary_minimum.offset(dist));
        // simplify boundary to cut down processing time
        boundary.simplify(MM2INT(0.1), MM2INT(0.1));
    }
    constexpr bool detect_loops = true;
    PathOrderOptimizer<ConstPolygonRef> order_optimizer(near_start_location.value_or(getLastPlannedPositionOrStartingPosition()), ZSeamConfig(), detect_loops, &boundary);
    for(size_t line_idx = 0; line_idx < polygons.size(); line_idx++)
    {
        order_optimizer.addPolyline(polygons[line_idx]);
    }
    order_optimizer.optimize();

    for(size_t order_idx = 0; order_idx < order_optimizer.paths.size(); order_idx++)
    {
        const PathOrderOptimizer<ConstPolygonRef>::Path& path = order_optimizer.paths[order_idx];
        ConstPolygonRef polygon = *path.vertices;
        const size_t start = path.start_vertex;
        const size_t end = 1 - start;
        const Point& p0 = polygon[start];
        const Point& p1 = polygon[end];
        // ignore line segments that are less than 5uM long
        if(vSize2(p1 - p0) < MINIMUM_SQUARED_LINE_LENGTH)
        {
            continue;
        }
        addTravel(p0);
        addExtrusionMove(p1, config, space_fill_type, flow_ratio, false, 1.0, fan_speed);

        // Wipe
        if(wipe_dist != 0)
        {
            bool wipe = true;
            int line_width = config.getLineWidth();

            // Don't wipe is current extrusion is too small
            if(vSize2(p1 - p0) <= line_width * line_width * 4)
            {
                wipe = false;
            }

            // Don't wipe if next starting point is very near
            if(wipe && (order_idx < order_optimizer.paths.size() - 1))
            {
                const PathOrderOptimizer<ConstPolygonRef>::Path& next_path = order_optimizer.paths[order_idx + 1];
                ConstPolygonRef next_polygon = *next_path.vertices;
                const size_t next_start = next_path.start_vertex;
                const Point& next_p0 = next_polygon[next_start];
                if (vSize2(next_p0 - p1) <= line_width * line_width * 4)
                {
                    wipe = false;
                }
            }

            if (wipe)
            {
                addExtrusionMove(p1 + normal(p1 - p0, wipe_dist), config, space_fill_type, 0.0, false, 1.0, fan_speed);
            }
        }
    }
}

void LayerPlan::addLinesMonotonic(const Polygons& polygons, const GCodePathConfig& config, const SpaceFillType space_fill_type, const AngleRadians monotonic_direction, const coord_t max_adjacent_distance, const coord_t wipe_dist, const Ratio flow_ratio, const double fan_speed)
{
    const Point last_position = getLastPlannedPositionOrStartingPosition();
    PathOrderMonotonic<ConstPolygonRef> order(monotonic_direction, max_adjacent_distance, last_position);
    for(size_t line_idx = 0; line_idx < polygons.size(); ++line_idx)
    {
        order.addPolyline(polygons[line_idx]);
    }
    order.optimize();

    for (unsigned int order_idx = 0; order_idx < order.paths.size(); order_idx++)
    {
        const PathOrder<ConstPolygonRef>::Path& path = order.paths[order_idx];
        ConstPolygonRef polygon = *path.vertices;
        const size_t start = path.start_vertex;
        const size_t end = 1 - start;
        const Point& p0 = polygon[start];
        const Point& p1 = polygon[end];
        // ignore line segments that are less than 5uM long
        if(vSize2(p1 - p0) < MINIMUM_SQUARED_LINE_LENGTH)
        {
            continue;
        }
        addTravel(p0);
        addExtrusionMove(p1, config, space_fill_type, flow_ratio, false, 1.0, fan_speed);

        // Wipe
        if (wipe_dist != 0)
        {
            bool wipe = true;
            int line_width = config.getLineWidth();

            // Don't wipe is current extrusion is too small
            if (vSize2(p1 - p0) <= line_width * line_width * 4)
            {
                wipe = false;
            }

            // Don't wipe if next starting point is very near
            if (wipe && (order_idx < order.paths.size() - 1))
            {
                const PathOrder<ConstPolygonRef>::Path& next_path = order.paths[order_idx + 1];
                ConstPolygonRef next_polygon = *next_path.vertices;
                const size_t next_start = next_path.start_vertex;
                const Point& next_p0 = next_polygon[next_start];
                if (vSize2(next_p0 - p1) <= line_width * line_width * 4)
                {
                    wipe = false;
                }
            }

            if(wipe)
            {
                addExtrusionMove(p1 + normal(p1 - p0, wipe_dist), config, space_fill_type, 0.0, false, 1.0, fan_speed);
            }
        }
    }
}

void LayerPlan::spiralizeWallSlice(const GCodePathConfig& config, ConstPolygonRef wall, ConstPolygonRef last_wall, const int seam_vertex_idx, const int last_seam_vertex_idx, const bool is_top_layer, const bool is_bottom_layer)
{
    const bool smooth_contours = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("smooth_spiralized_contours");

    // once we are into the spiral we always start at the end point of the last layer (if any)
    const Point origin = (last_seam_vertex_idx >= 0 && !is_bottom_layer) ? last_wall[last_seam_vertex_idx] : wall[seam_vertex_idx];
    addTravel_simple(origin);

    if (!smooth_contours && last_seam_vertex_idx >= 0) {
        // when not smoothing, we get to the (unchanged) outline for this layer as quickly as possible so that the remainder of the
        // outline wall has the correct direction - although this creates a little step, the end result is generally better because when the first
        // outline wall has the wrong direction (due to it starting from the finish point of the last layer) the visual effect is very noticeable
        Point join_first_wall_at = LinearAlg2D::getClosestOnLineSegment(origin, wall[seam_vertex_idx], wall[(seam_vertex_idx + 1) % wall.size()]);
        if (vSize(join_first_wall_at - origin) > 10)
        {
            addExtrusionMove(join_first_wall_at, config, SpaceFillType::Polygons, 1.0, true);
        }
    }

    const int n_points = wall.size();
    Polygons last_wall_polygons;
    last_wall_polygons.add(last_wall);
    const int max_dist2 = config.getLineWidth() * config.getLineWidth() * 4; // (2 * lineWidth)^2;

    double total_length = 0.0; // determine the length of the complete wall
    Point p0 = origin;
    for (int wall_point_idx = 1; wall_point_idx <= n_points; ++wall_point_idx)
    {
        const Point& p1 = wall[(seam_vertex_idx + wall_point_idx) % n_points];
        total_length += vSizeMM(p1 - p0);
        p0 = p1;
    }

    if (total_length == 0.0)
    {
        // nothing to do
        return;
    }

    // if this is the bottom layer, avoid creating a big elephants foot by starting with a reduced flow and increasing the flow
    // so that by the time the end of the first spiral is complete the flow is 100% - note that immediately before the spiral
    // is output, the extruder will be printing a normal wall line and so will be fully pressurised so that will tend to keep the
    // flow going

    // if this is the top layer, avoid an abrupt end by printing the same outline again but this time taper the spiral by reducing
    // the flow whilst keeping the same height - once the flow is down to a minimum allowed value, coast a little further

    const double min_bottom_layer_flow = 0.25; // start the bottom spiral at this flow rate
    const double min_top_layer_flow = 0.25; // lowest allowed flow while tapering the last spiral

    double speed_factor = 1; // may be reduced when printing the top layer so as to avoid a jump in extrusion rate as the layer starts

    if (is_top_layer)
    {
        // HACK ALERT - the last layer is approx 50% longer than the previous layer so it should take longer to print but the
        // normal slow down for quick layers mechanism can kick in and speed this layer up (because it is longer) but we prefer
        // the layer to be printed at a similar speed to the previous layer to avoid abrupt changes in extrusion rate so we slow it down

        const FanSpeedLayerTimeSettings& layer_time_settings = extruder_plans.back().fan_speed_layer_time_settings;
        const double min_time = layer_time_settings.cool_min_layer_time;
        const double normal_layer_time = total_length / config.getSpeed();

        // would this layer's speed normally get reduced to satisfy the min layer time?
        if (normal_layer_time < min_time)
        {
            // yes, so the extended version will not get slowed down so much and we want to compensate for that
            const double extended_layer_time = (total_length * (2 - min_top_layer_flow)) / config.getSpeed();

            // modify the speed factor to cancel out the speed increase that would normally happen due to the longer layer time
            speed_factor = normal_layer_time / std::min(extended_layer_time, min_time);
        }
    }

    // extrude to the points following the seam vertex
    // the last point is the seam vertex as the polygon is a loop
    double wall_length = 0.0;
    p0 = origin;
    for (int wall_point_idx = 1; wall_point_idx <= n_points; ++wall_point_idx)
    {
        // p is a point from the current wall polygon
        const Point& p = wall[(seam_vertex_idx + wall_point_idx) % n_points];
        wall_length += vSizeMM(p - p0);
        p0 = p;

        const double flow = (is_bottom_layer) ? (min_bottom_layer_flow + ((1 - min_bottom_layer_flow) * wall_length / total_length)) : 1.0;

        // if required, use interpolation to smooth the x/y coordinates between layers but not for the first spiralized layer
        // as that lies directly on top of a non-spiralized wall with exactly the same outline and not for the last point in each layer
        // because we want that to be numerically exactly the same as the starting point on the next layer (not subject to any rounding)
        if (smooth_contours && !is_bottom_layer && wall_point_idx < n_points)
        {
            // now find the point on the last wall that is closest to p
            ClosestPolygonPoint cpp = PolygonUtils::findClosest(p, last_wall_polygons);

            // if we found a point and it's not further away than max_dist2, use it
            if (cpp.isValid() && vSize2(cpp.location - p) <= max_dist2)
            {
                // interpolate between cpp.location and p depending on how far we have progressed along wall
                addExtrusionMove(cpp.location + (p - cpp.location) * (wall_length / total_length), config, SpaceFillType::Polygons, flow, true, speed_factor);
            }
            else
            {
                // no point in the last wall was found close enough to the current wall point so don't interpolate
                addExtrusionMove(p, config, SpaceFillType::Polygons, flow, true, speed_factor);
            }
        }
        else
        {
            // no smoothing, use point verbatim
            addExtrusionMove(p, config, SpaceFillType::Polygons, flow, true, speed_factor);
        }
    }

    if (is_top_layer)
    {
        // add the tapering spiral
        const double min_spiral_coast_dist = 10; // mm
        double distance_coasted = 0;
        wall_length = 0;
        for (int wall_point_idx = 1; wall_point_idx <= n_points && distance_coasted < min_spiral_coast_dist; wall_point_idx++)
        {
            const Point& p = wall[(seam_vertex_idx + wall_point_idx) % n_points];
            const double seg_length = vSizeMM(p - p0);
            wall_length += seg_length;
            p0 = p;
            // flow is reduced in step with the distance travelled so the wall width should remain roughly constant
            double flow = 1 - (wall_length / total_length);
            if (flow < min_top_layer_flow)
            {
                flow = 0;
                distance_coasted += seg_length;
            }
            // reduce number of paths created when polygon has many points by limiting precision of flow
            addExtrusionMove(p, config, SpaceFillType::Polygons, ((int)(flow * 20)) / 20.0, false, speed_factor);
        }
    }
}

void ExtruderPlan::forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrudeTime)
{
    double totalTime = travelTime + extrudeTime; 
    if (totalTime < minTime && extrudeTime > 0.0)
    {
        double minExtrudeTime = minTime - travelTime;
        if (minExtrudeTime < 1)
            minExtrudeTime = 1;
        double factor = extrudeTime / minExtrudeTime;
        for (GCodePath& path : paths)
        {
            if (path.isTravelPath())
                continue;
            double speed = path.config->getSpeed() * path.speed_factor * factor;
            if (speed < minimalSpeed)
                factor = minimalSpeed / (path.config->getSpeed() * path.speed_factor);
        }

        //Only slow down for the minimal time if that will be slower.
        assert(getExtrudeSpeedFactor() == 1.0); // The extrude speed factor is assumed not to be changed yet
        if (factor < 1.0)
        {
            setExtrudeSpeedFactor(factor);
        }
        else 
        {
            factor = 1.0;
        }
        
        double inv_factor = 1.0 / factor; // cause multiplication is faster than division
        
        // Adjust stored naive time estimates
        estimates.extrude_time *= inv_factor;
        for (GCodePath& path : paths)
        {
            path.estimates.extrude_time *= inv_factor;
        }

        if (minTime - (extrudeTime * inv_factor) - travelTime > 0.1)
        {
            extraTime = minTime - (extrudeTime * inv_factor) - travelTime;
        }
        totalPrintTime = (extrudeTime * inv_factor) + travelTime;
    }
}

TimeMaterialEstimates ExtruderPlan::computeNaiveTimeEstimates(Point starting_position)
{
    TimeMaterialEstimates ret;
    Point p0 = starting_position;

    bool was_retracted = false; // wrong assumption; won't matter that much. (TODO)
    for (GCodePath& path : paths)
    {
        bool is_extrusion_path = false;
        double* path_time_estimate;
        double& material_estimate = path.estimates.material;
        if (!path.isTravelPath())
        {
            is_extrusion_path = true;
            path_time_estimate = &path.estimates.extrude_time;
        }
        else 
        {
            if (path.retract)
            {
                path_time_estimate = &path.estimates.retracted_travel_time;
            }
            else 
            {
                path_time_estimate = &path.estimates.unretracted_travel_time;
            }
            if (path.retract != was_retracted)
            { // handle retraction times
                double retract_unretract_time;
                if (path.retract)
                {
                    retract_unretract_time = retraction_config.distance / retraction_config.speed;
                }
                else 
                {
                    retract_unretract_time = retraction_config.distance / retraction_config.primeSpeed;
                }
                path.estimates.retracted_travel_time += 0.5 * retract_unretract_time;
                path.estimates.unretracted_travel_time += 0.5 * retract_unretract_time;
            }
        }
        for(Point& p1 : path.points)
        {
            double length = vSizeMM(p0 - p1);
            if (is_extrusion_path)
            {
                material_estimate += length * INT2MM(layer_thickness) * INT2MM(path.config->getLineWidth());
            }
            double thisTime = length / (path.config->getSpeed() * path.speed_factor);
            *path_time_estimate += thisTime;
            p0 = p1;
        }
        estimates += path.estimates;
    }
    return estimates;
}

void ExtruderPlan::processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time, Point starting_position)
{
    TimeMaterialEstimates estimates = computeNaiveTimeEstimates(starting_position);
    totalPrintTime = estimates.getTotalTime();
    if (force_minimal_layer_time)
    {
        forceMinimalLayerTime(fan_speed_layer_time_settings.cool_min_layer_time, fan_speed_layer_time_settings.cool_min_speed, estimates.getTravelTime(), estimates.getExtrudeTime());
    }

    /*
                   min layer time
                   :
                   :  min layer time fan speed min
                |  :  :
      ^    max..|__:  :
                |  \  :
     fan        |   \ :
    speed  min..|... \:___________
                |________________
                  layer time >


    */
    // interpolate fan speed (for cool_fan_full_layer and for cool_min_layer_time_fan_speed_max)
    fan_speed = fan_speed_layer_time_settings.cool_fan_speed_min;
    double totalLayerTime = estimates.unretracted_travel_time + estimates.extrude_time;
    if (force_minimal_layer_time && totalLayerTime < fan_speed_layer_time_settings.cool_min_layer_time)
    {
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_max;
    }
    else if (fan_speed_layer_time_settings.cool_min_layer_time >= fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max)
    {
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_min;
    }
    else if (force_minimal_layer_time && totalLayerTime < fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max)
    {
        // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
        double fan_speed_diff = fan_speed_layer_time_settings.cool_fan_speed_max - fan_speed_layer_time_settings.cool_fan_speed_min;
        double layer_time_diff = fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max - fan_speed_layer_time_settings.cool_min_layer_time;
        double fraction_of_slope = (totalLayerTime - fan_speed_layer_time_settings.cool_min_layer_time) / layer_time_diff;
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_max - fan_speed_diff * fraction_of_slope;
    }
    /*
    Supposing no influence of minimal layer time;
    i.e. layer time > min layer time fan speed min:

              max..   fan 'full' on layer
                   |  :
                   |  :
      ^       min..|..:________________
     fan           |  /
    speed          | /
          speed_0..|/
                   |
                   |__________________
                     layer nr >

    */
    if (layer_nr < fan_speed_layer_time_settings.cool_fan_full_layer
        && fan_speed_layer_time_settings.cool_fan_full_layer > 0 // don't apply initial layer fan speed speedup if disabled.
        && !is_raft_layer // don't apply initial layer fan speed speedup to raft, but to model layers
    )
    {
        //Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_0 + (fan_speed - fan_speed_layer_time_settings.cool_fan_speed_0) * std::max(LayerIndex(0), layer_nr) / fan_speed_layer_time_settings.cool_fan_full_layer;
    }
}

void LayerPlan::processFanSpeedAndMinimalLayerTime(Point starting_position)
{
    for (unsigned int extr_plan_idx = 0; extr_plan_idx < extruder_plans.size(); extr_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extr_plan_idx];
        bool force_minimal_layer_time = extr_plan_idx == extruder_plans.size() - 1;
        extruder_plan.processFanSpeedAndMinimalLayerTime(force_minimal_layer_time, starting_position);
        if (!extruder_plan.paths.empty() && !extruder_plan.paths.back().points.empty())
        {
            starting_position = extruder_plan.paths.back().points.back();
        }
    }
}



void LayerPlan::writeGCode(GCodeExport& gcode)
{
    Communication* communication = Application::getInstance().communication;
    communication->setLayerForSend(layer_nr);
    communication->sendCurrentPosition(gcode.getPositionXY());

    gcode.resetExtrusionValue();
    gcode.setLayerNr(layer_nr);
    gcode.writeLayerComment(layer_nr);

    // flow-rate compensation
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    gcode.setFlowRateExtrusionSettings(mesh_group_settings.get<double>("flow_rate_max_extrusion_offset"), mesh_group_settings.get<Ratio>("flow_rate_extrusion_offset_factor")); //Offset is in mm.

    static LayerIndex layer_1 {1 - static_cast<LayerIndex>(Raft::getTotalExtraLayers())};
    if (layer_nr == layer_1 && mesh_group_settings.get<bool>("machine_heated_bed") && mesh_group_settings.get<Temperature>("material_bed_temperature") != mesh_group_settings.get<Temperature>("material_bed_temperature_layer_0"))
    {
        constexpr bool wait = false;
        gcode.writeBedTemperatureCommand(mesh_group_settings.get<Temperature>("material_bed_temperature"), wait);
    }

    gcode.setZ(z);

    const GCodePathConfig* last_extrusion_config = nullptr; // used to check whether we need to insert a TYPE comment in the gcode.

    size_t extruder_nr = gcode.getExtruderNr();
    const bool acceleration_enabled = mesh_group_settings.get<bool>("acceleration_enabled");
    const bool jerk_enabled = mesh_group_settings.get<bool>("jerk_enabled");
    std::string current_mesh = "NONMESH";

    for(size_t extruder_plan_idx = 0; extruder_plan_idx < extruder_plans.size(); extruder_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
        const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_plan.extruder_nr];
        coord_t z_hop_height = retraction_config.zHop;

        if (extruder_nr != extruder_plan.extruder_nr)
        {
            int prev_extruder = extruder_nr;
            extruder_nr = extruder_plan.extruder_nr;

            gcode.ResetLastEValueAfterWipe(prev_extruder);

            const ExtruderTrain& prev_extruder_train = Application::getInstance().current_slice->scene.extruders[prev_extruder];
            if (prev_extruder_train.settings.get<bool>("retraction_hop_after_extruder_switch"))
            {
                z_hop_height = storage.extruder_switch_retraction_config_per_extruder[prev_extruder].zHop;
                gcode.switchExtruder(extruder_nr, storage.extruder_switch_retraction_config_per_extruder[prev_extruder], z_hop_height);
            }
            else
            {
                gcode.switchExtruder(extruder_nr, storage.extruder_switch_retraction_config_per_extruder[prev_extruder]);
            }

            const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

            { // require printing temperature to be met
                constexpr bool wait = true;
                gcode.writeTemperatureCommand(extruder_nr, extruder_plan.required_start_temperature, wait);
            }

            if (extruder_plan.prev_extruder_standby_temp)
            { // turn off previous extruder
                constexpr bool wait = false;
                Temperature prev_extruder_temp = *extruder_plan.prev_extruder_standby_temp;
                const LayerIndex prev_layer_nr = (extruder_plan_idx == 0) ? layer_nr - 1 : layer_nr;
                if (prev_layer_nr == storage.max_print_height_per_extruder[prev_extruder])
                {
                    prev_extruder_temp = 0; // TODO ? should there be a setting for extruder_off_temperature ?
                }
                gcode.writeTemperatureCommand(prev_extruder, prev_extruder_temp, wait);
            }

            const double extra_prime_amount = extruder.settings.get<bool>("retraction_enable") ? extruder.settings.get<double>("switch_extruder_extra_prime_amount") : 0;
            gcode.addExtraPrimeAmount(extra_prime_amount);
        }
        else if (extruder_plan_idx == 0)
        {
            const WipeScriptConfig& wipe_config = storage.wipe_config_per_extruder[extruder_plan.extruder_nr];
            if (wipe_config.clean_between_layers && gcode.getExtrudedVolumeAfterLastWipe(extruder_nr) > wipe_config.max_extrusion_mm3)
            {
                gcode.insertWipeScript(wipe_config);
                gcode.ResetLastEValueAfterWipe(extruder_nr);
            }
            else if (layer_nr != 0 && Application::getInstance().current_slice->scene.extruders[extruder_nr].settings.get<bool>("retract_at_layer_change"))
            {
                // only do the retract if the paths are not spiralized
                if (!mesh_group_settings.get<bool>("magic_spiralize"))
                {
                    gcode.writeRetraction(retraction_config);
                }
            }
        }
        gcode.writeFanCommand(extruder_plan.getFanSpeed());
        std::vector<GCodePath>& paths = extruder_plan.paths;

        extruder_plan.inserts.sort([](const NozzleTempInsert& a, const NozzleTempInsert& b) -> bool
            {
                return  a.path_idx < b.path_idx; 
            });

        const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

        bool update_extrusion_offset = true;

        for(unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
        {
            extruder_plan.handleInserts(path_idx, gcode);
            
            GCodePath& path = paths[path_idx];

            if (path.perform_prime)
            {
                gcode.writePrimeTrain(extruder.settings.get<Velocity>("speed_travel"));
                gcode.writeRetraction(retraction_config);
            }

            if (!path.retract && path.config->isTravelPath() && path.points.size() == 1 && path.points[0] == gcode.getPositionXY() && z == gcode.getPositionZ())
            {
                // ignore travel moves to the current location to avoid needless change of acceleration/jerk
                continue;
            }

            if (acceleration_enabled)
            {
                if (path.config->isTravelPath())
                {
                    gcode.writeTravelAcceleration(path.config->getAcceleration());
                }
                else
                {
                    gcode.writePrintAcceleration(path.config->getAcceleration());
                }
            }
            if (jerk_enabled)
            {
                gcode.writeJerk(path.config->getJerk());
            }

            if (path.retract)
            {
                gcode.writeRetraction(retraction_config);
                if (path.perform_z_hop)
                {
                    gcode.writeZhopStart(z_hop_height);
                    z_hop_height = retraction_config.zHop; // back to normal z hop
                }
                else
                {
                    gcode.writeZhopEnd();
                }
            }
            if (!path.config->isTravelPath() && last_extrusion_config != path.config)
            {
                gcode.writeTypeComment(path.config->type);
                if (path.config->isBridgePath())
                {
                    gcode.writeComment("BRIDGE");
                }
                last_extrusion_config = path.config;
                update_extrusion_offset = true;
            }
            else
            {
                update_extrusion_offset = false;
            }

            double speed = path.config->getSpeed();

            // for some movements such as prime tower purge, the speed may get changed by this factor
            speed *= path.speed_factor;

            //Apply the extrusion speed factor if it's an extrusion move.
            if (!path.config->isTravelPath())
            {
                speed *= extruder_plan.getExtrudeSpeedFactor();
            }
            //This seems to be the best location to place this, but still not ideal.
            if (path.mesh_id != current_mesh)
            {
                current_mesh = path.mesh_id;
                std::stringstream ss;
                ss << "MESH:" << current_mesh;
                gcode.writeComment(ss.str());
            }
            if (path.config->isTravelPath())
            { // early comp for travel paths, which are handled more simply
                if (!path.perform_z_hop && final_travel_z != z && extruder_plan_idx == (extruder_plans.size() - 1) && path_idx == (paths.size() - 1))
                {
                    // Before the final travel, move up to the next layer height, on the current spot, with a sensible speed.
                    Point3 current_position = gcode.getPosition();
                    current_position.z = final_travel_z;
                    gcode.writeTravel(current_position, extruder.settings.get<Velocity>("speed_z_hop"));

                    // Prevent the final travel(s) from resetting to the 'previous' layer height.
                    gcode.setZ(final_travel_z);
                }
                for(unsigned int point_idx = 0; point_idx < path.points.size() - 1; point_idx++)
                {
                    gcode.writeTravel(path.points[point_idx], speed);
                }
                if (path.unretract_before_last_travel_move)
                {
                    // We need to unretract before the last travel move of the path if the next path is an outer wall.
                    gcode.writeUnretractionAndPrime();
                }
                gcode.writeTravel(path.points.back(), speed);
                continue;
            }

            bool spiralize = path.spiralize;
            if (!spiralize) // normal (extrusion) move (with coasting)
            {
                // if path provides a valid (in range 0-100) fan speed, use it
                const double path_fan_speed = path.getFanSpeed();
                gcode.writeFanCommand(path_fan_speed != GCodePathConfig::FAN_SPEED_DEFAULT ? path_fan_speed : extruder_plan.getFanSpeed());

                bool coasting = extruder.settings.get<bool>("coasting_enable");
                if (coasting)
                {
                    coasting = writePathWithCoasting(gcode, extruder_plan_idx, path_idx, layer_thickness);
                }
                if (!coasting) // not same as 'else', cause we might have changed [coasting] in the line above...
                { // normal path to gcode algorithm
                    for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                    {
                        const double extrude_speed = speed * path.speed_back_pressure_factor;
                        communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), extrude_speed);
                        gcode.writeExtrusion(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM(), path.config->type, update_extrusion_offset);
                    }
                }
            }
            else
            { // SPIRALIZE
                //If we need to spiralize then raise the head slowly by 1 layer as this path progresses.
                float totalLength = 0.0;
                Point p0 = gcode.getPositionXY();
                for (unsigned int _path_idx = path_idx; _path_idx < paths.size() && !paths[_path_idx].isTravelPath(); _path_idx++)
                {
                    GCodePath& _path = paths[_path_idx];
                    for (unsigned int point_idx = 0; point_idx < _path.points.size(); point_idx++)
                    {
                        Point p1 = _path.points[point_idx];
                        totalLength += vSizeMM(p0 - p1);
                        p0 = p1;
                    }
                }

                float length = 0.0;
                p0 = gcode.getPositionXY();
                for (; path_idx < paths.size() && paths[path_idx].spiralize; path_idx++)
                { // handle all consecutive spiralized paths > CHANGES path_idx!
                    GCodePath& path = paths[path_idx];

                    for (unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                    {
                        const Point p1 = path.points[point_idx];
                        length += vSizeMM(p0 - p1);
                        p0 = p1;
                        gcode.setZ(std::round(z + layer_thickness * length / totalLength));

                        const double extrude_speed = speed * path.speed_back_pressure_factor;
                        communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), extrude_speed);
                        gcode.writeExtrusion(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM(), path.config->type, update_extrusion_offset);
                    }
                    // for layer display only - the loop finished at the seam vertex but as we started from
                    // the location of the previous layer's seam vertex the loop may have a gap if this layer's
                    // seam vertex is "behind" the previous layer's seam vertex. So output another line segment
                    // that joins this layer's seam vertex to the following vertex. If the layers have been blended
                    // then this can cause a visible ridge (on the screen, not on the print) because the first vertex
                    // would have been shifted in x/y to make it nearer to the previous layer outline but the seam
                    // vertex would not be shifted (as it's the last vertex in the sequence). The smoother the model,
                    // the less the vertices are shifted and the less obvious is the ridge. If the layer display
                    // really displayed a spiral rather than slices of a spiral, this would not be required.
                    communication->sendLineTo(path.config->type, path.points[0], path.getLineWidthForLayerView(), path.config->getLayerThickness(), speed);
                }
                path_idx--; // the last path_idx didnt spiralize, so it's not part of the current spiralize path
            }
        } // paths for this extruder /\  .

        if (extruder.settings.get<bool>("cool_lift_head") && extruder_plan.extraTime > 0.0)
        {
            gcode.writeComment("Small layer, adding delay");
            const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[gcode.getExtruderNr()];
            gcode.writeRetraction(retraction_config);
            if (extruder_plan_idx == extruder_plans.size() - 1 || !extruder.settings.get<bool>("machine_extruder_end_pos_abs"))
            { // only move the head if it's the last extruder plan; otherwise it's already at the switching bay area 
                // or do it anyway when we switch extruder in-place
                gcode.setZ(gcode.getPositionZ() + MM2INT(3.0));
                gcode.writeTravel(gcode.getPositionXY(), configs_storage.travel_config_per_extruder[extruder_nr].getSpeed());

                const Point current_pos = gcode.getPositionXY();
                const Point machine_middle = storage.machine_size.flatten().getMiddle();
                const Point toward_middle_of_bed = current_pos - normal(current_pos - machine_middle, MM2INT(20.0));
                gcode.writeTravel(toward_middle_of_bed, configs_storage.travel_config_per_extruder[extruder_nr].getSpeed());
            }
            gcode.writeDelay(extruder_plan.extraTime);
        }

        extruder_plan.handleAllRemainingInserts(gcode);
    } // extruder plans /\  .
    
    gcode.updateTotalPrintTime();
}

void LayerPlan::overrideFanSpeeds(double speed)
{
    for (ExtruderPlan& extruder_plan : extruder_plans)
    {
        extruder_plan.setFanSpeed(speed);
    }
}


bool LayerPlan::makeRetractSwitchRetract(unsigned int extruder_plan_idx, unsigned int path_idx)
{
    std::vector<GCodePath>& paths = extruder_plans[extruder_plan_idx].paths;
    for (unsigned int path_idx2 = path_idx + 1; path_idx2 < paths.size(); path_idx2++)
    {
        if (paths[path_idx2].getExtrusionMM3perMM() > 0) 
        {
            return false; 
        }
    }
    
    if (extruder_plans.size() <= extruder_plan_idx+1)
    {
        return false; // TODO: check first extruder of the next layer! (generally only on the last layer of the second extruder)
    }
        
    if (extruder_plans[extruder_plan_idx + 1].extruder_nr != extruder_plans[extruder_plan_idx].extruder_nr)
    {
        return true;
    }
    else 
    {
        return false;
    }
}
    
bool LayerPlan::writePathWithCoasting(GCodeExport& gcode, const size_t extruder_plan_idx, const size_t path_idx, const coord_t layer_thickness)
{
    ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
    const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_plan.extruder_nr];
    const double coasting_volume = extruder.settings.get<double>("coasting_volume");
    if (coasting_volume <= 0)
    { 
        return false; 
    }
    const std::vector<GCodePath>& paths = extruder_plan.paths;
    const GCodePath& path = paths[path_idx];
    if (path_idx + 1 >= paths.size()
        || (path.isTravelPath() || !paths[path_idx + 1].config->isTravelPath()) 
        || path.points.size() < 2)
    {
        return false;
    }

    coord_t coasting_min_dist_considered = MM2INT(0.1); // hardcoded setting for when to not perform coasting

    const double extrude_speed = path.config->getSpeed() * extruder_plan.getExtrudeSpeedFactor() * path.speed_factor * path.speed_back_pressure_factor;

    const coord_t coasting_dist = MM2INT(MM2_2INT(coasting_volume) / layer_thickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    const double coasting_min_volume = extruder.settings.get<double>("coasting_min_volume");
    const coord_t coasting_min_dist = MM2INT(MM2_2INT(coasting_min_volume + coasting_volume) / layer_thickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    //           /\ the minimal distance when coasting will coast the full coasting volume instead of linearly less with linearly smaller paths

    std::vector<coord_t> accumulated_dist_per_point; // the first accumulated dist is that of the last point! (that of the last point is always zero...)
    accumulated_dist_per_point.push_back(0);

    coord_t accumulated_dist = 0;

    bool length_is_less_than_min_dist = true;

    unsigned int acc_dist_idx_gt_coast_dist = NO_INDEX; // the index of the first point with accumulated_dist more than coasting_dist (= index into accumulated_dist_per_point)
     // == the point printed BEFORE the start point for coasting

    const Point* last = &path.points[path.points.size() - 1];
    for (unsigned int backward_point_idx = 1; backward_point_idx < path.points.size(); backward_point_idx++)
    {
        const Point& point = path.points[path.points.size() - 1 - backward_point_idx];
        const coord_t distance = vSize(point - *last);
        accumulated_dist += distance;
        accumulated_dist_per_point.push_back(accumulated_dist);
        
        if (acc_dist_idx_gt_coast_dist == NO_INDEX && accumulated_dist >= coasting_dist)
        {
            acc_dist_idx_gt_coast_dist = backward_point_idx; // the newly added point
        }
        
        if (accumulated_dist >= coasting_min_dist)
        {
            length_is_less_than_min_dist = false;
            break;
        }
        
        last = &point;
    }
    
    if (accumulated_dist < coasting_min_dist_considered)
    {
        return false;
    }
    coord_t actual_coasting_dist = coasting_dist;
    if (length_is_less_than_min_dist)
    {
        // in this case accumulated_dist is the length of the whole path
        actual_coasting_dist = accumulated_dist * coasting_dist / coasting_min_dist;
        if(actual_coasting_dist == 0) //Downscaling due to Minimum Coasting Distance reduces coasting to less than 1 micron.
        {
            return false; //Skip coasting at all then.
        }
        for (acc_dist_idx_gt_coast_dist = 1; acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size() ; acc_dist_idx_gt_coast_dist++)
        { // search for the correct coast_dist_idx
            if (accumulated_dist_per_point[acc_dist_idx_gt_coast_dist] >= actual_coasting_dist)
            {
                break;
            }
        }
    }

    assert(acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size()); // something has gone wrong; coasting_min_dist < coasting_dist ?

    const size_t point_idx_before_start = path.points.size() - 1 - acc_dist_idx_gt_coast_dist;

    Point start;
    { // computation of begin point of coasting
        const coord_t residual_dist = actual_coasting_dist - accumulated_dist_per_point[acc_dist_idx_gt_coast_dist - 1];
        const Point& a = path.points[point_idx_before_start];
        const Point& b = path.points[point_idx_before_start + 1];
        start = b + normal(a - b, residual_dist);
    }

    { // write normal extrude path:
        Communication* communication = Application::getInstance().communication;
        for(size_t point_idx = 0; point_idx <= point_idx_before_start; point_idx++)
        {
            communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), extrude_speed);
            gcode.writeExtrusion(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM(), path.config->type);
        }
        communication->sendLineTo(path.config->type, start, path.getLineWidthForLayerView(), path.config->getLayerThickness(), extrude_speed);
        gcode.writeExtrusion(start, extrude_speed, path.getExtrusionMM3perMM(), path.config->type);
    }

    // write coasting path
    for (size_t point_idx = point_idx_before_start + 1; point_idx < path.points.size(); point_idx++)
    {
        const Ratio coasting_speed_modifier = extruder.settings.get<Ratio>("coasting_speed");
        const Velocity speed = Velocity(coasting_speed_modifier * path.config->getSpeed() * extruder_plan.getExtrudeSpeedFactor());
        gcode.writeTravel(path.points[point_idx], speed);
    }
    return true;
}

void LayerPlan::applyBackPressureCompensation()
{
    for (auto& extruder_plan : extruder_plans)
    {
        const double back_pressure_compensation =
            Application::getInstance().current_slice->scene.extruders[extruder_plan.extruder_nr].settings.get<double>("speed_equalize_flow_width_factor");
        if (back_pressure_compensation != 0.0)
        {
            extruder_plan.applyBackPressureCompensation(back_pressure_compensation);
        }
    }
}

int LayerPlan::getLayerNr() const
{
    return layer_nr;
}

Point LayerPlan::getLastPlannedPositionOrStartingPosition() const
{
    return last_planned_position.value_or(layer_start_pos_per_extruder[getExtruder()]);
}

bool LayerPlan::getIsInsideMesh() const
{
    return was_inside;
}

bool LayerPlan::getSkirtBrimIsPlanned(unsigned int extruder_nr) const
{
    return skirt_brim_is_processed[extruder_nr];
}

void LayerPlan::setSkirtBrimIsPlanned(unsigned int extruder_nr)
{
    skirt_brim_is_processed[extruder_nr] = true;
}

size_t LayerPlan::getExtruder() const
{
    return extruder_plans.back().extruder_nr;
}

void LayerPlan::setBridgeWallMask(const Polygons& polys)
{
    bridge_wall_mask = polys;
}

void LayerPlan::setOverhangMask(const Polygons& polys)
{
    overhang_mask = polys;
}

}//namespace cura
