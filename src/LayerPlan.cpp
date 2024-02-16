// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "LayerPlan.h"

#include <algorithm>
#include <cstring>
#include <numeric>
#include <optional>

#include <range/v3/algorithm/max_element.hpp>
#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To communicate layer view data.
#include "ExtruderTrain.h"
#include "PathOrderMonotonic.h" //Monotonic ordering of skin lines.
#include "Slice.h"
#include "WipeScriptConfig.h"
#include "communication/Communication.h"
#include "pathPlanning/Comb.h"
#include "pathPlanning/CombPaths.h"
#include "plugins/slots.h"
#include "raft.h" // getTotalExtraLayers
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "utils/Simplify.h"
#include "utils/linearAlg2D.h"
#include "utils/polygonUtils.h"
#include "utils/section_type.h"

namespace cura
{

constexpr int MINIMUM_LINE_LENGTH = 5; // in uM. Generated lines shorter than this may be discarded
constexpr int MINIMUM_SQUARED_LINE_LENGTH = MINIMUM_LINE_LENGTH * MINIMUM_LINE_LENGTH;


GCodePath* LayerPlan::getLatestPathWithConfig(
    const GCodePathConfig& config,
    const SpaceFillType space_fill_type,
    const coord_t z_offset,
    const Ratio flow,
    const Ratio width_factor,
    const bool spiralize,
    const Ratio speed_factor)
{
    std::vector<GCodePath>& paths = extruder_plans_.back().paths_;
    if (paths.size() > 0 && paths.back().config == config && ! paths.back().done && paths.back().flow == flow && paths.back().width_factor == width_factor
        && paths.back().speed_factor == speed_factor && paths.back().z_offset == z_offset
        && paths.back().mesh == current_mesh_) // spiralize can only change when a travel path is in between
    {
        return &paths.back();
    }
    paths.emplace_back(GCodePath{ .z_offset = z_offset,
                                  .config = config,
                                  .mesh = current_mesh_,
                                  .space_fill_type = space_fill_type,
                                  .flow = flow,
                                  .width_factor = width_factor,
                                  .spiralize = spiralize,
                                  .speed_factor = speed_factor });

    GCodePath* ret = &paths.back();
    ret->skip_agressive_merge_hint = mode_skip_agressive_merge_;
    return ret;
}

const Polygons* LayerPlan::getCombBoundaryInside() const
{
    return &comb_boundary_preferred_;
}

void LayerPlan::forceNewPathStart()
{
    std::vector<GCodePath>& paths = extruder_plans_.back().paths_;
    if (paths.size() > 0)
        paths[paths.size() - 1].done = true;
}

LayerPlan::LayerPlan(
    const SliceDataStorage& storage,
    LayerIndex layer_nr,
    coord_t z,
    coord_t layer_thickness,
    size_t start_extruder,
    const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder,
    coord_t comb_boundary_offset,
    coord_t comb_move_inside_distance,
    coord_t travel_avoid_distance)
    : configs_storage_(storage, layer_nr, layer_thickness)
    , z_(z)
    , final_travel_z_(z)
    , mode_skip_agressive_merge_(false)
    , storage_(storage)
    , layer_nr_(layer_nr)
    , is_initial_layer_(layer_nr == 0 - static_cast<LayerIndex>(Raft::getTotalExtraLayers()))
    , layer_type_(Raft::getLayerType(layer_nr))
    , layer_thickness_(layer_thickness)
    , has_prime_tower_planned_per_extruder_(Application::getInstance().current_slice_->scene.extruders.size(), false)
    , current_mesh_(nullptr)
    , last_extruder_previous_layer_(start_extruder)
    , last_planned_extruder_(&Application::getInstance().current_slice_->scene.extruders[start_extruder])
    , first_travel_destination_is_inside_(false)
    , // set properly when addTravel is called for the first time (otherwise not set properly)
    comb_boundary_minimum_(computeCombBoundary(CombBoundary::MINIMUM))
    , comb_boundary_preferred_(computeCombBoundary(CombBoundary::PREFERRED))
    , comb_move_inside_distance_(comb_move_inside_distance)
    , fan_speed_layer_time_settings_per_extruder_(fan_speed_layer_time_settings_per_extruder)
{
    size_t current_extruder = start_extruder;
    was_inside_ = true; // not used, because the first travel move is bogus
    is_inside_ = false; // assumes the next move will not be to inside a layer part (overwritten just before going into a layer part)
    if (Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<CombingMode>("retraction_combing") != CombingMode::OFF)
    {
        comb_ = new Comb(storage, layer_nr, comb_boundary_minimum_, comb_boundary_preferred_, comb_boundary_offset, travel_avoid_distance, comb_move_inside_distance);
    }
    else
    {
        comb_ = nullptr;
    }
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice_->scene.extruders)
    {
        layer_start_pos_per_extruder_.emplace_back(extruder.settings_.get<coord_t>("layer_start_x"), extruder.settings_.get<coord_t>("layer_start_y"));
    }
    extruder_plans_.reserve(Application::getInstance().current_slice_->scene.extruders.size());
    const auto is_raft_layer = layer_type_ == Raft::LayerType::RaftBase || layer_type_ == Raft::LayerType::RaftInterface || layer_type_ == Raft::LayerType::RaftSurface;
    extruder_plans_.emplace_back(
        current_extruder,
        layer_nr,
        is_initial_layer_,
        is_raft_layer,
        layer_thickness,
        fan_speed_layer_time_settings_per_extruder[current_extruder],
        storage.retraction_wipe_config_per_extruder[current_extruder].retraction_config);

    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice_->scene.extruders.size(); extruder_nr++)
    { // Skirt and brim.
        skirt_brim_is_processed_[extruder_nr] = false;
    }
}

LayerPlan::~LayerPlan()
{
    if (comb_)
        delete comb_;
}

ExtruderTrain* LayerPlan::getLastPlannedExtruderTrain()
{
    return last_planned_extruder_;
}

Polygons LayerPlan::computeCombBoundary(const CombBoundary boundary_type)
{
    Polygons comb_boundary;
    const CombingMode mesh_combing_mode = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<CombingMode>("retraction_combing");
    if (mesh_combing_mode != CombingMode::OFF && (layer_nr_ >= 0 || mesh_combing_mode != CombingMode::NO_SKIN))
    {
        switch (layer_type_)
        {
        case Raft::LayerType::RaftBase:
            comb_boundary = storage_.raftBaseOutline.offset(MM2INT(0.1));
            break;

        case Raft::LayerType::RaftInterface:
            comb_boundary = storage_.raftInterfaceOutline.offset(MM2INT(0.1));
            break;

        case Raft::LayerType::RaftSurface:
            comb_boundary = storage_.raftSurfaceOutline.offset(MM2INT(0.1));
            break;

        case Raft::LayerType::Airgap:
            // do nothing for airgap
            break;

        case Raft::LayerType::Model:
            for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage_.meshes)
            {
                const auto& mesh = *mesh_ptr;
                const SliceLayer& layer = mesh.layers[static_cast<size_t>(layer_nr_)];
                // don't process infill_mesh or anti_overhang_mesh
                if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
                {
                    continue;
                }
                coord_t offset;
                switch (boundary_type)
                {
                case CombBoundary::MINIMUM:
                    offset = -mesh.settings.get<coord_t>("machine_nozzle_size") / 2 - mesh.settings.get<coord_t>("wall_line_width_0") / 2;
                    break;
                case CombBoundary::PREFERRED:
                    offset = -mesh.settings.get<coord_t>("machine_nozzle_size") * 3 / 2 - mesh.settings.get<coord_t>("wall_line_width_0") / 2;
                    break;
                default:
                    offset = 0;
                    spdlog::warn("Unknown combing boundary type. Did you forget to configure the comb offset for a new boundary type?");
                    break;
                }

                const CombingMode combing_mode = mesh.settings.get<CombingMode>("retraction_combing");
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
                    else if (combing_mode == CombingMode::NO_OUTER_SURFACES)
                    {
                        Polygons top_and_bottom_most_fill;
                        for (const SliceLayerPart& outer_surface_part : layer.parts)
                        {
                            for (const SkinPart& skin_part : outer_surface_part.skin_parts)
                            {
                                top_and_bottom_most_fill.add(skin_part.top_most_surface_fill);
                                top_and_bottom_most_fill.add(skin_part.bottom_most_surface_fill);
                            }
                        }
                        comb_boundary.add(part.outline.offset(offset).difference(top_and_bottom_most_fill));
                    }
                    else if (combing_mode == CombingMode::INFILL) // Add the infill (infill only)
                    {
                        comb_boundary.add(part.infill_area);
                    }
                }
            }
            break;
        }
    }
    return comb_boundary;
}

void LayerPlan::setIsInside(bool _is_inside)
{
    is_inside_ = _is_inside;
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
        const bool end_pos_absolute = extruder->settings_.get<bool>("machine_extruder_end_pos_abs");
        Point2LL end_pos(extruder->settings_.get<coord_t>("machine_extruder_end_pos_x"), extruder->settings_.get<coord_t>("machine_extruder_end_pos_y"));
        if (! end_pos_absolute)
        {
            end_pos += getLastPlannedPositionOrStartingPosition();
        }
        else
        {
            const Point2LL extruder_offset(extruder->settings_.get<coord_t>("machine_nozzle_offset_x"), extruder->settings_.get<coord_t>("machine_nozzle_offset_y"));
            end_pos += extruder_offset; // absolute end pos is given as a head position
        }
        if (end_pos_absolute || last_planned_position_)
        {
            addTravel(end_pos); //  + extruder_offset cause it
        }
    }
    if (extruder_plans_.back().paths_.empty() && extruder_plans_.back().inserts_.empty())
    { // first extruder plan in a layer might be empty, cause it is made with the last extruder planned in the previous layer
        extruder_plans_.back().extruder_nr_ = extruder_nr;
    }

    const auto is_raft_layer = layer_type_ == Raft::LayerType::RaftBase || layer_type_ == Raft::LayerType::RaftInterface || layer_type_ == Raft::LayerType::RaftSurface;
    extruder_plans_.emplace_back(
        extruder_nr,
        layer_nr_,
        is_initial_layer_,
        is_raft_layer,
        layer_thickness_,
        fan_speed_layer_time_settings_per_extruder_[extruder_nr],
        storage_.retraction_wipe_config_per_extruder[extruder_nr].retraction_config);
    assert(extruder_plans_.size() <= Application::getInstance().current_slice_->scene.extruders.size() && "Never use the same extruder twice on one layer!");
    last_planned_extruder_ = &Application::getInstance().current_slice_->scene.extruders[extruder_nr];

    { // handle starting pos of the new extruder
        ExtruderTrain* extruder = getLastPlannedExtruderTrain();
        const bool start_pos_absolute = extruder->settings_.get<bool>("machine_extruder_start_pos_abs");
        Point2LL start_pos(extruder->settings_.get<coord_t>("machine_extruder_start_pos_x"), extruder->settings_.get<coord_t>("machine_extruder_start_pos_y"));
        if (! start_pos_absolute)
        {
            start_pos += getLastPlannedPositionOrStartingPosition();
        }
        else
        {
            Point2LL extruder_offset(extruder->settings_.get<coord_t>("machine_nozzle_offset_x"), extruder->settings_.get<coord_t>("machine_nozzle_offset_y"));
            start_pos += extruder_offset; // absolute start pos is given as a head position
        }
        if (start_pos_absolute || last_planned_position_)
        {
            last_planned_position_ = start_pos;
        }
    }
    return true;
}
void LayerPlan::setMesh(const std::shared_ptr<const SliceMeshStorage>& mesh)
{
    current_mesh_ = mesh;
}

void LayerPlan::moveInsideCombBoundary(const coord_t distance, const std::optional<SliceLayerPart>& part)
{
    constexpr coord_t max_dist2 = MM2INT(2.0) * MM2INT(2.0); // if we are further than this distance, we conclude we are not inside even though we thought we were.
    // this function is to be used to move from the boundary of a part to inside the part
    Point2LL p = getLastPlannedPositionOrStartingPosition(); // copy, since we are going to move p
    if (PolygonUtils::moveInside(comb_boundary_preferred_, p, distance, max_dist2) != NO_INDEX)
    {
        // Move inside again, so we move out of tight 90deg corners
        PolygonUtils::moveInside(comb_boundary_preferred_, p, distance, max_dist2);
        if (comb_boundary_preferred_.inside(p) && (part == std::nullopt || part->outline.inside(p)))
        {
            addTravel_simple(p);
            // Make sure the that any retraction happens after this move, not before it by starting a new move path.
            forceNewPathStart();
        }
    }
}

bool LayerPlan::getPrimeTowerIsPlanned(unsigned int extruder_nr) const
{
    return has_prime_tower_planned_per_extruder_[extruder_nr];
}

void LayerPlan::setPrimeTowerIsPlanned(unsigned int extruder_nr)
{
    has_prime_tower_planned_per_extruder_[extruder_nr] = true;
}

std::optional<std::pair<Point2LL, bool>> LayerPlan::getFirstTravelDestinationState() const
{
    std::optional<std::pair<Point2LL, bool>> ret;
    if (first_travel_destination_)
    {
        ret = std::make_pair(*first_travel_destination_, first_travel_destination_is_inside_);
    }
    return ret;
}

GCodePath& LayerPlan::addTravel(const Point2LL& p, const bool force_retract, const coord_t z_offset)
{
    const GCodePathConfig& travel_config = configs_storage_.travel_config_per_extruder[getExtruder()];

    const RetractionConfig& retraction_config
        = current_mesh_ ? current_mesh_->retraction_wipe_config.retraction_config : storage_.retraction_wipe_config_per_extruder[getExtruder()].retraction_config;

    GCodePath* path = getLatestPathWithConfig(travel_config, SpaceFillType::None, z_offset);

    bool combed = false;

    const ExtruderTrain* extruder = getLastPlannedExtruderTrain();
    const Settings& mesh_or_extruder_settings = current_mesh_ ? current_mesh_->settings : extruder->settings_;


    const bool is_first_travel_of_extruder_after_switch
        = extruder_plans_.back().paths_.size() == 1 && (extruder_plans_.size() > 1 || last_extruder_previous_layer_ != getExtruder());
    bool bypass_combing = is_first_travel_of_extruder_after_switch && mesh_or_extruder_settings.get<bool>("retraction_hop_after_extruder_switch");

    const bool is_first_travel_of_layer = ! static_cast<bool>(last_planned_position_);
    const bool retraction_enable = mesh_or_extruder_settings.get<bool>("retraction_enable");
    if (is_first_travel_of_layer)
    {
        bypass_combing = true; // first travel move is bogus; it is added after this and the previous layer have been planned in LayerPlanBuffer::addConnectingTravelMove
        first_travel_destination_ = p;
        first_travel_destination_is_inside_ = is_inside_;
        if (layer_nr_ == 0 && retraction_enable && mesh_or_extruder_settings.get<bool>("retraction_hop_enabled"))
        {
            path->retract = true;
            path->perform_z_hop = true;
        }
        forceNewPathStart(); // force a new travel path after this first bogus move
    }
    else if (force_retract && last_planned_position_ && ! shorterThen(*last_planned_position_ - p, retraction_config.retraction_min_travel_distance))
    {
        // path is not shorter than min travel distance, force a retraction
        path->retract = true;
        if (comb_ == nullptr)
        {
            path->perform_z_hop = mesh_or_extruder_settings.get<bool>("retraction_hop_enabled");
        }
    }

    if (comb_ != nullptr && ! bypass_combing)
    {
        CombPaths combPaths;

        // Divide by 2 to get the radius
        // Multiply by 2 because if two lines start and end points places very close then will be applied combing with retractions. (Ex: for brim)
        const coord_t max_distance_ignored = mesh_or_extruder_settings.get<coord_t>("machine_nozzle_tip_outer_diameter") / 2 * 2;

        bool unretract_before_last_travel_move = false; // Decided when calculating the combing
        const bool perform_z_hops = mesh_or_extruder_settings.get<bool>("retraction_hop_enabled");
        const bool perform_z_hops_only_when_collides = mesh_or_extruder_settings.get<bool>("retraction_hop_only_when_collides");
        combed = comb_->calc(
            perform_z_hops,
            perform_z_hops_only_when_collides,
            *extruder,
            *last_planned_position_,
            p,
            combPaths,
            was_inside_,
            is_inside_,
            max_distance_ignored,
            unretract_before_last_travel_move);
        if (combed)
        {
            bool retract = path->retract || (combPaths.size() > 1 && retraction_enable);
            if (! retract)
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
            }

            const coord_t maximum_travel_resolution = mesh_or_extruder_settings.get<coord_t>("meshfix_maximum_travel_resolution");
            coord_t distance = 0;
            Point2LL last_point((last_planned_position_) ? *last_planned_position_ : Point2LL(0, 0));
            for (CombPath& combPath : combPaths)
            { // add all comb paths (don't do anything special for paths which are moving through air)
                if (combPath.empty())
                {
                    continue;
                }
                for (Point2LL& comb_point : combPath)
                {
                    if (path->points.empty() || vSize2(path->points.back() - comb_point) > maximum_travel_resolution * maximum_travel_resolution)
                    {
                        path->points.push_back(comb_point);
                        distance += vSize(last_point - comb_point);
                        last_point = comb_point;
                    }
                }
                distance += vSize(last_point - p);
                const coord_t retract_threshold = mesh_or_extruder_settings.get<coord_t>("retraction_combing_max_distance");
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
    if (! is_first_travel_of_layer && last_planned_position_ && shorterThen(*last_planned_position_ - p, retraction_config.retraction_min_travel_distance))
    {
        path->retract = false;
        path->perform_z_hop = false;
    }

    // no combing? retract only when path is not shorter than minimum travel distance
    if (! combed && ! is_first_travel_of_layer && last_planned_position_ && ! shorterThen(*last_planned_position_ - p, retraction_config.retraction_min_travel_distance))
    {
        if (was_inside_) // when the previous location was from printing something which is considered inside (not support or prime tower etc)
        { // then move inside the printed part, so that we don't ooze on the outer wall while retraction, but on the inside of the print.
            assert(extruder != nullptr);
            coord_t innermost_wall_line_width
                = mesh_or_extruder_settings.get<coord_t>((mesh_or_extruder_settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
            if (layer_nr_ == 0)
            {
                innermost_wall_line_width *= mesh_or_extruder_settings.get<Ratio>("initial_layer_line_width_factor");
            }
            moveInsideCombBoundary(innermost_wall_line_width);
        }
        path->retract = retraction_enable;
        path->perform_z_hop = retraction_enable && mesh_or_extruder_settings.get<bool>("retraction_hop_enabled");
    }

    // must start new travel path as retraction can be enabled or not depending on path length, etc.
    forceNewPathStart();

    GCodePath& ret = addTravel_simple(p, path);
    was_inside_ = is_inside_;
    return ret;
}

GCodePath& LayerPlan::addTravel_simple(const Point2LL& p, GCodePath* path)
{
    bool is_first_travel_of_layer = ! static_cast<bool>(last_planned_position_);
    if (is_first_travel_of_layer)
    { // spiralize calls addTravel_simple directly as the first travel move in a layer
        first_travel_destination_ = p;
        first_travel_destination_is_inside_ = is_inside_;
    }
    if (path == nullptr)
    {
        path = getLatestPathWithConfig(configs_storage_.travel_config_per_extruder[getExtruder()], SpaceFillType::None);
    }
    path->points.push_back(p);
    last_planned_position_ = p;
    return *path;
}

void LayerPlan::planPrime(double prime_blob_wipe_length)
{
    forceNewPathStart();
    GCodePath& prime_travel = addTravel_simple(getLastPlannedPositionOrStartingPosition() + Point2LL(0, MM2INT(prime_blob_wipe_length)));
    prime_travel.retract = false;
    prime_travel.perform_z_hop = false;
    prime_travel.perform_prime = true;
    forceNewPathStart();
}

void LayerPlan::addExtrusionMove(
    const Point2LL p,
    const GCodePathConfig& config,
    const SpaceFillType space_fill_type,
    const Ratio& flow,
    const Ratio width_factor,
    const bool spiralize,
    const Ratio speed_factor,
    const double fan_speed)
{
    GCodePath* path = getLatestPathWithConfig(config, space_fill_type, config.z_offset, flow, width_factor, spiralize, speed_factor);
    path->points.push_back(p);
    path->setFanSpeed(fan_speed);
    if (! static_cast<bool>(first_extrusion_acc_jerk_))
    {
        first_extrusion_acc_jerk_ = std::make_pair(path->config.getAcceleration(), path->config.getJerk());
    }
    last_planned_position_ = p;
}

void LayerPlan::addPolygon(
    ConstPolygonRef polygon,
    int start_idx,
    const bool backwards,
    const GCodePathConfig& config,
    coord_t wall_0_wipe_dist,
    bool spiralize,
    const Ratio& flow_ratio,
    bool always_retract)
{
    constexpr Ratio width_ratio = 1.0_r; // Not printed with variable line width.
    Point2LL p0 = polygon[start_idx];
    addTravel(p0, always_retract, config.z_offset);
    const int direction = backwards ? -1 : 1;
    for (size_t point_idx = 1; point_idx < polygon.size(); point_idx++)
    {
        Point2LL p1 = polygon[(start_idx + point_idx * direction + polygon.size()) % polygon.size()];
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow_ratio, width_ratio, spiralize);
        p0 = p1;
    }
    if (polygon.size() > 2)
    {
        addExtrusionMove(polygon[start_idx], config, SpaceFillType::Polygons, flow_ratio, width_ratio, spiralize);

        if (wall_0_wipe_dist > 0)
        { // apply outer wall wipe
            p0 = polygon[start_idx];
            int distance_traversed = 0;
            for (size_t point_idx = 1;; point_idx++)
            {
                Point2LL p1 = polygon[(start_idx + point_idx * direction + polygon.size()) % polygon.size()];
                int p0p1_dist = vSize(p1 - p0);
                if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
                {
                    Point2LL vector = p1 - p0;
                    Point2LL half_way = p0 + normal(vector, wall_0_wipe_dist - distance_traversed);
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
        spdlog::warn("line added as polygon! (LayerPlan)");
    }
}

void LayerPlan::addPolygonsByOptimizer(
    const Polygons& polygons,
    const GCodePathConfig& config,
    const ZSeamConfig& z_seam_config,
    coord_t wall_0_wipe_dist,
    bool spiralize,
    const Ratio flow_ratio,
    bool always_retract,
    bool reverse_order,
    const std::optional<Point2LL> start_near_location)
{
    if (polygons.empty())
    {
        return;
    }
    PathOrderOptimizer<ConstPolygonPointer> orderOptimizer(start_near_location ? start_near_location.value() : getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for (size_t poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(polygons[poly_idx]);
    }
    orderOptimizer.optimize();

    if (! reverse_order)
    {
        for (const PathOrdering<ConstPolygonPointer>& path : orderOptimizer.paths_)
        {
            addPolygon(*path.vertices_, path.start_vertex_, path.backwards_, config, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
        }
    }
    else
    {
        for (int index = orderOptimizer.paths_.size() - 1; index >= 0; --index)
        {
            const PathOrdering<ConstPolygonPointer>& path = orderOptimizer.paths_[index];
            addPolygon(**path.vertices_, path.start_vertex_, path.backwards_, config, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
        }
    }
}

static constexpr double max_non_bridge_line_volume = MM2INT(100); // limit to accumulated "volume" of non-bridge lines which is proportional to distance x extrusion rate

static int i = 0;

void LayerPlan::addWallLine(
    const Point2LL& p0,
    const Point2LL& p1,
    const Settings& settings,
    const GCodePathConfig& default_config,
    const GCodePathConfig& roofing_config,
    const GCodePathConfig& bridge_config,
    double flow,
    const Ratio width_factor,
    double& non_bridge_line_volume,
    Ratio speed_factor,
    double distance_to_bridge_start)
{
    const coord_t min_line_len = 5; // we ignore lines less than 5um long
    const double acceleration_segment_len = MM2INT(1); // accelerate using segments of this length
    const double acceleration_factor = 0.75; // must be < 1, the larger the value, the slower the acceleration
    const bool spiralize = false;

    const coord_t min_bridge_line_len = settings.get<coord_t>("bridge_wall_min_length");
    const Ratio bridge_wall_coast = settings.get<Ratio>("bridge_wall_coast");
    const Ratio overhang_speed_factor = settings.get<Ratio>("wall_overhang_speed_factor");

    Point2LL cur_point = p0;

    // helper function to add a single non-bridge line

    // If the line precedes a bridge line, it may be coasted to reduce the nozzle pressure before the bridge is reached

    // alternatively, if the line follows a bridge line, it may be segmented and the print speed gradually increased to reduce under-extrusion

    auto addNonBridgeLine = [&](const Point2LL& line_end)
    {
        coord_t distance_to_line_end = vSize(cur_point - line_end);

        while (distance_to_line_end > min_line_len)
        {
            // if we are accelerating after a bridge line, the segment length is less than the whole line length
            Point2LL segment_end = (speed_factor == 1 || distance_to_line_end < acceleration_segment_len)
                                     ? line_end
                                     : cur_point + (line_end - cur_point) * acceleration_segment_len / distance_to_line_end;

            // flow required for the next line segment - when accelerating after a bridge segment, the flow is increased in inverse proportion to the speed_factor
            // so the slower the feedrate, the greater the flow - the idea is to get the extruder back to normal pressure as quickly as possible
            const double segment_flow = (speed_factor > 1) ? flow * (1 / speed_factor) : flow;

            // if a bridge is present in this wall, this particular segment may need to be partially or wholely coasted
            if (distance_to_bridge_start > 0)
            {
                // speed_flow_factor approximates how the extrusion rate alters between the non-bridge wall line and the following bridge wall line
                // if the extrusion rates are the same, its value will be 1, if the bridge config extrusion rate is < the non-bridge config extrusion rate, the value is < 1

                const Ratio speed_flow_factor((bridge_config.getSpeed() * bridge_config.getFlowRatio()) / (default_config.getSpeed() * default_config.getFlowRatio()));

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
                        addExtrusionMove(
                            segment_end + coast_dist * (cur_point - segment_end) / len,
                            default_config,
                            SpaceFillType::Polygons,
                            segment_flow,
                            width_factor,
                            spiralize,
                            speed_factor);
                    }
                    // then coast to start of bridge segment
                    constexpr Ratio no_flow = 0.0_r; // Coasting has no flow rate.
                    addExtrusionMove(segment_end, default_config, SpaceFillType::Polygons, no_flow, width_factor, spiralize, speed_factor);
                }
                else
                {
                    // no coasting required, just normal segment using non-bridge config
                    addExtrusionMove(
                        segment_end,
                        default_config,
                        SpaceFillType::Polygons,
                        segment_flow,
                        width_factor,
                        spiralize,
                        (overhang_mask_.empty() || (! overhang_mask_.inside(p0, true) && ! overhang_mask_.inside(p1, true))) ? speed_factor : overhang_speed_factor);
                }

                distance_to_bridge_start -= len;
            }
            else
            {
                // no coasting required, just normal segment using non-bridge config
                addExtrusionMove(
                    segment_end,
                    default_config,
                    SpaceFillType::Polygons,
                    segment_flow,
                    width_factor,
                    spiralize,
                    (overhang_mask_.empty() || (! overhang_mask_.inside(p0, true) && ! overhang_mask_.inside(p1, true))) ? speed_factor : overhang_speed_factor);
            }
            non_bridge_line_volume += vSize(cur_point - segment_end) * segment_flow * width_factor * speed_factor * default_config.getSpeed();
            cur_point = segment_end;
            speed_factor = 1 - (1 - speed_factor) * acceleration_factor;
            if (speed_factor >= 0.9)
            {
                speed_factor = 1.0;
            }
            distance_to_line_end = vSize(cur_point - line_end);
        }
    };

    const auto use_roofing_config = [&]() -> bool
    {
        if (roofing_config == default_config)
        {
            // if the roofing config and normal config are the same any way there is no need to check
            // what part of the line segment will be printed with what config.
            return false;
        }
        return PolygonUtils::polygonCollidesWithLineSegment(roofing_mask_, p0, p1) || roofing_mask_.inside(p1, true);
    }();

    if (use_roofing_config)
    {
        // The line segment is wholly or partially in the roofing area. The line is intersected
        // with the roofing area into line segments. Each line segment left in this intersection
        // will be printed using the roofing config, all removed segments will be printed using
        // the default_config. Since the original line segment was straight we can simply print
        // to the first and last point of the intersected line segments alternating between
        // roofing and default_config's.
        Polygons line_polys;
        line_polys.addLine(p0, p1);
        constexpr bool restitch = false; // only a single line doesn't need stitching
        auto roofing_line_segments = roofing_mask_.intersectionPolyLines(line_polys, restitch);

        if (roofing_line_segments.empty())
        {
            // roofing_line_segments should never be empty since we already checked that the line segment
            // intersects with the roofing area. But if it is empty then just print the line segment
            // using the default_config.
            addExtrusionMove(p1, default_config, SpaceFillType::Polygons, flow, width_factor, spiralize, 1.0_r);
        }
        else
        {
            // reorder all the line segments so all lines start at p0 and end at p1
            for (auto& line_poly : roofing_line_segments)
            {
                const Point2LL& line_p0 = line_poly.front();
                const Point2LL& line_p1 = line_poly.back();
                if (vSize2(line_p1 - p0) < vSize2(line_p0 - p0))
                {
                    std::reverse(line_poly.begin(), line_poly.end());
                }
            }
            std::sort(
                roofing_line_segments.begin(),
                roofing_line_segments.end(),
                [&](auto& a, auto& b)
                {
                    return vSize2(a.front() - p0) < vSize2(b.front() - p0);
                });

            // add intersected line segments, alternating between roofing and default_config
            for (const auto& line_poly : roofing_line_segments)
            {
                // This is only relevant for the very fist iteration of the loop
                // if the start of the line segment is not at minimum distance from p0
                if (vSize2(line_poly.front() - p0) > min_line_len * min_line_len)
                {
                    addExtrusionMove(line_poly.front(), default_config, SpaceFillType::Polygons, flow, width_factor, spiralize, 1.0_r);
                }

                addExtrusionMove(line_poly.back(), roofing_config, SpaceFillType::Polygons, flow, width_factor, spiralize, 1.0_r);
            }

            // if the last point is not yet at a minimum distance from p1 then add a move to p1
            if (vSize2(roofing_line_segments.back().back() - p1) > min_line_len * min_line_len)
            {
                addExtrusionMove(p1, default_config, SpaceFillType::Polygons, flow, width_factor, spiralize, 1.0_r);
            }
        }
    }
    else if (bridge_wall_mask_.empty())
    {
        // no bridges required
        addExtrusionMove(
            p1,
            default_config,
            SpaceFillType::Polygons,
            flow,
            width_factor,
            spiralize,
            (overhang_mask_.empty() || (! overhang_mask_.inside(p0, true) && ! overhang_mask_.inside(p1, true))) ? 1.0_r : overhang_speed_factor);
    }
    else
    {
        // bridges may be required
        if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask_, p0, p1))
        {
            // the line crosses the boundary between supported and non-supported regions so one or more bridges are required

            // determine which segments of the line are bridges

            Polygons line_polys;
            line_polys.addLine(p0, p1);
            constexpr bool restitch = false; // only a single line doesn't need stitching
            line_polys = bridge_wall_mask_.intersectionPolyLines(line_polys, restitch);

            // line_polys now contains the wall lines that need to be printed using bridge_config

            while (line_polys.size() > 0)
            {
                // find the bridge line segment that's nearest to the current point
                int nearest = 0;
                double smallest_dist2 = vSize2f(cur_point - line_polys[0][0]);
                for (unsigned i = 1; i < line_polys.size(); ++i)
                {
                    double dist2 = vSize2f(cur_point - line_polys[i][0]);
                    if (dist2 < smallest_dist2)
                    {
                        nearest = i;
                        smallest_dist2 = dist2;
                    }
                }
                ConstPolygonRef bridge = line_polys[nearest];

                // set b0 to the nearest vertex and b1 the furthest
                Point2LL b0 = bridge[0];
                Point2LL b1 = bridge[1];

                if (vSize2f(cur_point - b1) < vSize2f(cur_point - b0))
                {
                    // swap vertex order
                    b0 = bridge[1];
                    b1 = bridge[0];
                }

                // extrude using default_config to the start of the next bridge segment

                addNonBridgeLine(b0);

                const double bridge_line_len = vSize(b1 - cur_point);

                if (bridge_line_len >= min_bridge_line_len)
                {
                    // extrude using bridge_config to the end of the next bridge segment

                    if (bridge_line_len > min_line_len)
                    {
                        addExtrusionMove(b1, bridge_config, SpaceFillType::Polygons, flow, width_factor);
                        non_bridge_line_volume = 0;
                        cur_point = b1;
                        // after a bridge segment, start slow and accelerate to avoid under-extrusion due to extruder lag
                        speed_factor = std::max(std::min(Ratio(bridge_config.getSpeed() / default_config.getSpeed()), 1.0_r), 0.5_r);
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

            // if we haven't yet reached p1, fill the gap with default_config line
            addNonBridgeLine(p1);
        }
        else if (bridge_wall_mask_.inside(p0, true) && vSize(p0 - p1) >= min_bridge_line_len)
        {
            // both p0 and p1 must be above air (the result will be ugly!)
            addExtrusionMove(p1, bridge_config, SpaceFillType::Polygons, flow, width_factor);
            non_bridge_line_volume = 0;
        }
        else
        {
            // no part of the line is above air or the line is too short to print as a bridge line
            addNonBridgeLine(p1);
        }
    }
}

void LayerPlan::addWall(
    ConstPolygonRef wall,
    int start_idx,
    const Settings& settings,
    const GCodePathConfig& default_config,
    const GCodePathConfig& roofing_config,
    const GCodePathConfig& bridge_config,
    coord_t wall_0_wipe_dist,
    double flow_ratio,
    bool always_retract)
{
    // TODO: Deprecated in favor of ExtrusionJunction version below.
    if (wall.size() < 3)
    {
        spdlog::warn("Point, or line added as (polygon) wall sequence! (LayerPlan)");
    }

    constexpr size_t dummy_perimeter_id = 0; // <-- Here, don't care about which perimeter any more.
    const coord_t nominal_line_width
        = default_config
              .getLineWidth(); // <-- The line width which it's 'supposed to' be will be used to adjust the flow ratio each time, this'll give a flow-ratio-multiplier of 1.

    ExtrusionLine ewall;
    std::for_each(
        wall.begin(),
        wall.end(),
        [&dummy_perimeter_id, &nominal_line_width, &ewall](const Point2LL& p)
        {
            ewall.emplace_back(p, nominal_line_width, dummy_perimeter_id);
        });
    ewall.emplace_back(*wall.begin(), nominal_line_width, dummy_perimeter_id);
    constexpr bool is_closed = true;
    constexpr bool is_reversed = false;
    constexpr bool is_linked_path = false;
    addWall(ewall, start_idx, settings, default_config, roofing_config, bridge_config, wall_0_wipe_dist, flow_ratio, always_retract, is_closed, is_reversed, is_linked_path);
}

void LayerPlan::addWall(
    const ExtrusionLine& wall,
    int start_idx,
    const Settings& settings,
    const GCodePathConfig& default_config,
    const GCodePathConfig& roofing_config,
    const GCodePathConfig& bridge_config,
    coord_t wall_0_wipe_dist,
    double flow_ratio,
    bool always_retract,
    const bool is_closed,
    const bool is_reversed,
    const bool is_linked_path)
{
    if (wall.empty())
    {
        return;
    }
    if (is_closed)
    {
        // make sure wall start point is not above air!
        start_idx = locateFirstSupportedVertex(wall, start_idx);
    }

    double non_bridge_line_volume = max_non_bridge_line_volume; // assume extruder is fully pressurised before first non-bridge line is output
    double speed_factor = 1.0; // start first line at normal speed
    coord_t distance_to_bridge_start = 0; // will be updated before each line is processed

    const coord_t min_bridge_line_len = settings.get<coord_t>("bridge_wall_min_length");

    const Ratio nominal_line_width_multiplier{
        1.0 / Ratio{ static_cast<Ratio::value_type>(default_config.getLineWidth()) }
    }; // we multiply the flow with the actual wanted line width (for that junction), and then multiply with this

    // helper function to calculate the distance from the start of the current wall line to the first bridge segment

    auto computeDistanceToBridgeStart = [&](unsigned current_index)
    {
        distance_to_bridge_start = 0;

        if (! bridge_wall_mask_.empty())
        {
            // there is air below the part so iterate through the lines that have not yet been output accumulating the total distance to the first bridge segment
            for (unsigned point_idx = current_index; point_idx < wall.size(); ++point_idx)
            {
                const ExtrusionJunction& p0 = wall[point_idx];
                const ExtrusionJunction& p1 = wall[(point_idx + 1) % wall.size()];

                if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask_, p0.p_, p1.p_))
                {
                    // the line crosses the boundary between supported and non-supported regions so it will contain one or more bridge segments

                    // determine which segments of the line are bridges

                    Polygons line_polys;
                    line_polys.addLine(p0.p_, p1.p_);
                    constexpr bool restitch = false; // only a single line doesn't need stitching
                    line_polys = bridge_wall_mask_.intersectionPolyLines(line_polys, restitch);

                    while (line_polys.size() > 0)
                    {
                        // find the bridge line segment that's nearest to p0
                        int nearest = 0;
                        double smallest_dist2 = vSize2f(p0.p_ - line_polys[0][0]);
                        for (unsigned i = 1; i < line_polys.size(); ++i)
                        {
                            double dist2 = vSize2f(p0.p_ - line_polys[i][0]);
                            if (dist2 < smallest_dist2)
                            {
                                nearest = i;
                                smallest_dist2 = dist2;
                            }
                        }
                        ConstPolygonRef bridge = line_polys[nearest];

                        // set b0 to the nearest vertex and b1 the furthest
                        Point2LL b0 = bridge[0];
                        Point2LL b1 = bridge[1];

                        if (vSize2f(p0.p_ - b1) < vSize2f(p0.p_ - b0))
                        {
                            // swap vertex order
                            b0 = bridge[1];
                            b1 = bridge[0];
                        }

                        distance_to_bridge_start += vSize(b0 - p0.p_);

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
                else if (! bridge_wall_mask_.inside(p0.p_, true))
                {
                    // none of the line is over air
                    distance_to_bridge_start += vSize(p1.p_ - p0.p_);
                }
            }

            // we have got all the way to the end of the wall without finding a bridge segment so disable coasting by setting distance_to_bridge_start back to 0

            distance_to_bridge_start = 0;
        }
    };

    bool first_line = true;
    const coord_t small_feature_max_length = settings.get<coord_t>("small_feature_max_length");
    const bool is_small_feature = (small_feature_max_length > 0) && (layer_nr_ == 0 || wall.inset_idx_ == 0) && cura::shorterThan(wall, small_feature_max_length);
    Ratio small_feature_speed_factor = settings.get<Ratio>((layer_nr_ == 0) ? "small_feature_speed_factor_0" : "small_feature_speed_factor");
    const Velocity min_speed = fan_speed_layer_time_settings_per_extruder_[getLastPlannedExtruderTrain()->extruder_nr_].cool_min_speed;
    small_feature_speed_factor = std::max((double)small_feature_speed_factor, (double)(min_speed / default_config.getSpeed()));
    const coord_t max_area_deviation = std::max(settings.get<int>("meshfix_maximum_extrusion_area_deviation"), 1); // Square micrometres!
    const coord_t max_resolution = std::max(settings.get<coord_t>("meshfix_maximum_resolution"), coord_t(1));

    ExtrusionJunction p0 = wall[start_idx];

    const int direction = is_reversed ? -1 : 1;
    const size_t max_index = is_closed ? wall.size() + 1 : wall.size();
    for (size_t point_idx = 1; point_idx < max_index; point_idx++)
    {
        const ExtrusionJunction& p1 = wall[(wall.size() + start_idx + point_idx * direction) % wall.size()];

        if (! bridge_wall_mask_.empty())
        {
            computeDistanceToBridgeStart((wall.size() + start_idx + point_idx * direction - 1) % wall.size());
        }

        if (first_line)
        {
            addTravel(p0.p_, always_retract);
            first_line = false;
        }

        /*
        If the line has variable width, break it up into pieces with the
        following constraints:
        - Each piece must be smaller than the Maximum Resolution setting.
        - The difference between the trapezoidal shape of the line and the
          rectangular shape of the line may not exceed the Maximum Extrusion
          Area Deviation setting, unless required by the first constraint.
        Since breaking up a line segment into N pieces (each with averaged
        width) divides the area deviation by N, we can simply check how many
        pieces we'd want to get low enough deviation, then check if each piece
        is not too short at the end.
        */
        const coord_t delta_line_width = p1.w_ - p0.w_;
        const Point2LL line_vector = p1.p_ - p0.p_;
        const coord_t line_length = vSize(line_vector);
        /*
        Calculate how much the line would deviate from the trapezoidal shape if printed at average width.
        This formula is:
        - Half the length times half the delta width, for the rectangular shape of the deviating side.
        - Half of that because the ideal line width is trapezoidal, making the deviating part triangular.
        - Double of that because the deviation occurs on both sides of the idealised line width.
        This results in delta_line_width / 2 * line_length / 2 / 2 * 2 == delta_line_width * line_length / 4.
        */
        const coord_t line_area_deviation = std::abs(delta_line_width) * line_length / 4;
        const size_t pieces_limit_deviation = round_up_divide(line_area_deviation, max_area_deviation); // How many pieces we'd need to stay beneath the max area deviation.
        const size_t pieces_limit_resolution = line_length / max_resolution; // Round down this time, to not exceed the maximum resolution.
        const size_t pieces = std::max(size_t(1), std::min(pieces_limit_deviation, pieces_limit_resolution)); // Resolution overrides deviation, if resolution is a constraint.
        const coord_t piece_length = round_divide(line_length, pieces);

        for (size_t piece = 0; piece < pieces; ++piece)
        {
            const double average_progress = (double(piece) + 0.5) / pieces; // How far along this line to sample the line width in the middle of this piece.
            const coord_t line_width = p0.w_ + average_progress * delta_line_width;
            const Point2LL destination = p0.p_ + normal(line_vector, piece_length * (piece + 1));
            if (is_small_feature)
            {
                constexpr bool spiralize = false;
                addExtrusionMove(
                    destination,
                    default_config,
                    SpaceFillType::Polygons,
                    flow_ratio,
                    line_width * nominal_line_width_multiplier,
                    spiralize,
                    small_feature_speed_factor);
            }
            else
            {
                const Point2LL origin = p0.p_ + normal(line_vector, piece_length * piece);
                addWallLine(
                    origin,
                    destination,
                    settings,
                    default_config,
                    roofing_config,
                    bridge_config,
                    flow_ratio,
                    line_width * nominal_line_width_multiplier,
                    non_bridge_line_volume,
                    speed_factor,
                    distance_to_bridge_start);
            }
        }

        p0 = p1;
    }

    if (wall.size() >= 2)
    {
        if (! bridge_wall_mask_.empty())
        {
            computeDistanceToBridgeStart((start_idx + wall.size() - 1) % wall.size());
        }

        if (wall_0_wipe_dist > 0 && ! is_linked_path)
        { // apply outer wall wipe
            p0 = wall[start_idx];
            int distance_traversed = 0;
            for (unsigned int point_idx = 1;; point_idx++)
            {
                if (point_idx > wall.size() && distance_traversed == 0) // Wall has a total circumference of 0. This loop would never end.
                {
                    break; // No wipe if the wall has no circumference.
                }
                ExtrusionJunction p1 = wall[(start_idx + point_idx) % wall.size()];
                int p0p1_dist = vSize(p1 - p0);
                if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
                {
                    Point2LL vector = p1.p_ - p0.p_;
                    Point2LL half_way = p0.p_ + normal(vector, wall_0_wipe_dist - distance_traversed);
                    addTravel_simple(half_way);
                    break;
                }
                else
                {
                    addTravel_simple(p1.p_);
                    distance_traversed += p0p1_dist;
                }
                p0 = p1;
            }
            forceNewPathStart();
        }
    }
    else
    {
        spdlog::warn("Point added as wall sequence! (LayerPlan)");
    }
}

void LayerPlan::addInfillWall(const ExtrusionLine& wall, const GCodePathConfig& path_config, bool force_retract)
{
    assert(! wall.empty() && "All empty walls should have been filtered at this stage");
    ExtrusionJunction junction{ *wall.begin() };
    addTravel(junction.p_, force_retract);

    for (const auto& junction_n : wall)
    {
        const Ratio width_factor{ static_cast<Ratio::value_type>(junction_n.w_) / Ratio{ static_cast<Ratio::value_type>(path_config.getLineWidth()) } };
        constexpr SpaceFillType space_fill_type = SpaceFillType::Polygons;
        constexpr Ratio flow = 1.0_r;
        addExtrusionMove(junction_n.p_, path_config, space_fill_type, flow, width_factor);
        junction = junction_n;
    }
}

void LayerPlan::addWalls(
    const Polygons& walls,
    const Settings& settings,
    const GCodePathConfig& default_config,
    const GCodePathConfig& roofing_config,
    const GCodePathConfig& bridge_config,
    const ZSeamConfig& z_seam_config,
    coord_t wall_0_wipe_dist,
    double flow_ratio,
    bool always_retract)
{
    // TODO: Deprecated in favor of ExtrusionJunction version below.
    PathOrderOptimizer<ConstPolygonPointer> orderOptimizer(getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for (size_t poly_idx = 0; poly_idx < walls.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(walls[poly_idx]);
    }
    orderOptimizer.optimize();
    for (const PathOrdering<ConstPolygonPointer>& path : orderOptimizer.paths_)
    {
        addWall(**path.vertices_, path.start_vertex_, settings, default_config, roofing_config, bridge_config, wall_0_wipe_dist, flow_ratio, always_retract);
    }
}


void LayerPlan::addLinesByOptimizer(
    const Polygons& polygons,
    const GCodePathConfig& config,
    const SpaceFillType space_fill_type,
    const bool enable_travel_optimization,
    const coord_t wipe_dist,
    const Ratio flow_ratio,
    const std::optional<Point2LL> near_start_location,
    const double fan_speed,
    const bool reverse_print_direction,
    const std::unordered_multimap<ConstPolygonPointer, ConstPolygonPointer>& order_requirements)
{
    Polygons boundary;
    if (enable_travel_optimization && ! comb_boundary_minimum_.empty())
    {
        // use the combing boundary inflated so that all infill lines are inside the boundary
        int dist = 0;
        if (layer_nr_ >= 0)
        {
            // determine how much the skin/infill lines overlap the combing boundary
            for (const std::shared_ptr<SliceMeshStorage>& mesh : storage_.meshes)
            {
                const coord_t overlap = std::max(mesh->settings.get<coord_t>("skin_overlap_mm"), mesh->settings.get<coord_t>("infill_overlap_mm"));
                if (overlap > dist)
                {
                    dist = overlap;
                }
            }
            dist += 100; // ensure boundary is slightly outside all skin/infill lines
        }
        boundary.add(comb_boundary_minimum_.offset(dist));
        // simplify boundary to cut down processing time
        boundary = Simplify(MM2INT(0.1), MM2INT(0.1), 0).polygon(boundary);
    }
    constexpr bool detect_loops = true;
    PathOrderOptimizer<ConstPolygonPointer> order_optimizer(
        near_start_location.value_or(getLastPlannedPositionOrStartingPosition()),
        ZSeamConfig(),
        detect_loops,
        &boundary,
        reverse_print_direction,
        order_requirements);
    for (size_t line_idx = 0; line_idx < polygons.size(); line_idx++)
    {
        order_optimizer.addPolyline(polygons[line_idx]);
    }
    order_optimizer.optimize();

    addLinesInGivenOrder(order_optimizer.paths_, config, space_fill_type, wipe_dist, flow_ratio, fan_speed);
}


void LayerPlan::addLinesInGivenOrder(
    const std::vector<PathOrdering<ConstPolygonPointer>>& paths,
    const GCodePathConfig& config,
    const SpaceFillType space_fill_type,
    const coord_t wipe_dist,
    const Ratio flow_ratio,
    const double fan_speed)
{
    coord_t half_line_width = config.getLineWidth() / 2;
    coord_t line_width_2 = half_line_width * half_line_width;
    for (size_t order_idx = 0; order_idx < paths.size(); order_idx++)
    {
        const PathOrdering<ConstPolygonPointer>& path = paths[order_idx];
        ConstPolygonRef polyline = *path.vertices_;
        const size_t start_idx = path.start_vertex_;
        assert(start_idx == 0 || start_idx == polyline.size() - 1 || path.is_closed_);
        const Point2LL start = polyline[start_idx];

        if (vSize2(getLastPlannedPositionOrStartingPosition() - start) < line_width_2)
        {
            // Instead of doing a small travel that is shorter than the line width (which is generally done at pretty high jerk & move) do a
            // "fake" extrusion move
            constexpr Ratio flow = 0.0_r;
            constexpr Ratio width_factor = 1.0_r;
            constexpr bool spiralize = false;
            constexpr Ratio speed_factor = 1.0_r;
            addExtrusionMove(start, config, space_fill_type, flow, width_factor, spiralize, speed_factor, fan_speed);
        }
        else
        {
            addTravel(start, false, config.z_offset);
        }

        Point2LL p0 = start;
        for (size_t idx = 0; idx < polyline.size(); idx++)
        {
            size_t point_idx;
            if (path.is_closed_)
            {
                point_idx = (start_idx + idx + 1) % polyline.size();
            }
            else if (start_idx == 0)
            {
                point_idx = idx;
            }
            else
            {
                assert(start_idx == polyline.size() - 1);
                point_idx = start_idx - idx;
            }
            Point2LL p1 = polyline[point_idx];

            // ignore line segments that are less than 5uM long
            if (vSize2(p1 - p0) >= MINIMUM_SQUARED_LINE_LENGTH)
            {
                constexpr Ratio width_factor = 1.0_r;
                constexpr bool spiralize = false;
                constexpr Ratio speed_factor = 1.0_r;
                addExtrusionMove(p1, config, space_fill_type, flow_ratio, width_factor, spiralize, speed_factor, fan_speed);
                p0 = p1;
            }
        }

        Point2LL p1 = polyline[(start_idx == 0) ? polyline.size() - 1 : 0];
        p0 = (polyline.size() <= 1) ? p1 : polyline[(start_idx == 0) ? polyline.size() - 2 : 1];

        // Wipe
        if (wipe_dist != 0)
        {
            bool wipe = true;
            int line_width = config.getLineWidth();

            // Don't wipe if current extrusion is too small
            if (polyline.polylineLength() <= line_width * 2)
            {
                wipe = false;
            }

            // Don't wipe if next starting point is very near
            if (wipe && (order_idx < paths.size() - 1))
            {
                const PathOrdering<ConstPolygonPointer>& next_path = paths[order_idx + 1];
                ConstPolygonRef next_polygon = *next_path.vertices_;
                const size_t next_start = next_path.start_vertex_;
                const Point2LL& next_p0 = next_polygon[next_start];
                if (vSize2(next_p0 - p1) <= line_width * line_width * 4)
                {
                    wipe = false;
                }
            }

            if (wipe)
            {
                constexpr Ratio flow = 0.0_r;
                constexpr Ratio width_factor = 1.0_r;
                constexpr bool spiralize = false;
                constexpr Ratio speed_factor = 1.0_r;
                addExtrusionMove(p1 + normal(p1 - p0, wipe_dist), config, space_fill_type, flow, width_factor, spiralize, speed_factor, fan_speed);
            }
        }
    }
}

void LayerPlan::addLinesMonotonic(
    const Polygons& area,
    const Polygons& polygons,
    const GCodePathConfig& config,
    const SpaceFillType space_fill_type,
    const AngleRadians monotonic_direction,
    const coord_t max_adjacent_distance,
    const coord_t exclude_distance,
    const coord_t wipe_dist,
    const Ratio flow_ratio,
    const double fan_speed)
{
    const Polygons exclude_areas = area.tubeShape(exclude_distance, exclude_distance);
    const coord_t exclude_dist2 = exclude_distance * exclude_distance;
    const Point2LL last_position = getLastPlannedPositionOrStartingPosition();

    // First lay all adjacent lines next to each other, to have a sensible input to the monotonic part of the algorithm.
    PathOrderOptimizer<ConstPolygonPointer> line_order(last_position);
    for (const ConstPolygonRef polyline : polygons)
    {
        line_order.addPolyline(polyline);
    }
    line_order.optimize();

    const auto is_inside_exclusion = [&exclude_areas, &exclude_dist2](ConstPolygonRef path)
    {
        return vSize2(path[1] - path[0]) < exclude_dist2 && exclude_areas.inside((path[0] + path[1]) / 2);
    };

    // Order monotonically, except for line-segments which stay in the excluded areas (read: close to the walls) consecutively.
    PathOrderMonotonic<ConstPolygonPointer> order(monotonic_direction, max_adjacent_distance, last_position);
    Polygons left_over;
    bool last_would_have_been_excluded = false;
    for (size_t line_idx = 0; line_idx < line_order.paths_.size(); ++line_idx)
    {
        const ConstPolygonRef polyline = *line_order.paths_[line_idx].vertices_;
        const bool inside_exclusion = is_inside_exclusion(polyline);
        const bool next_would_have_been_included = inside_exclusion && (line_idx < line_order.paths_.size() - 1 && is_inside_exclusion(*line_order.paths_[line_idx + 1].vertices_));
        if (inside_exclusion && last_would_have_been_excluded && next_would_have_been_included)
        {
            left_over.add(polyline);
        }
        else
        {
            order.addPolyline(polyline);
        }
        last_would_have_been_excluded = inside_exclusion;
    }
    order.optimize();

    // Read out and process the monotonically ordered lines.
    addLinesInGivenOrder(order.paths_, config, space_fill_type, wipe_dist, flow_ratio, fan_speed);

    // Add all lines in the excluded areas the 'normal' way.
    addLinesByOptimizer(left_over, config, space_fill_type, true, wipe_dist, flow_ratio, getLastPlannedPositionOrStartingPosition(), fan_speed);
}

void LayerPlan::spiralizeWallSlice(
    const GCodePathConfig& config,
    ConstPolygonRef wall,
    ConstPolygonRef last_wall,
    const int seam_vertex_idx,
    const int last_seam_vertex_idx,
    const bool is_top_layer,
    const bool is_bottom_layer)
{
    const bool smooth_contours = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<bool>("smooth_spiralized_contours");
    constexpr bool spiralize = true; // In addExtrusionMove calls, enable spiralize and use nominal line width.
    constexpr Ratio width_factor = 1.0_r;

    // once we are into the spiral we always start at the end point of the last layer (if any)
    const Point2LL origin = (last_seam_vertex_idx >= 0 && ! is_bottom_layer) ? last_wall[last_seam_vertex_idx] : wall[seam_vertex_idx];
    // NOTE: this used to use addTravel_simple() but if support is being generated then combed travel is required to avoid
    // the nozzle crossing the model on its return from printing the support.
    addTravel(origin);

    if (! smooth_contours && last_seam_vertex_idx >= 0)
    {
        // when not smoothing, we get to the (unchanged) outline for this layer as quickly as possible so that the remainder of the
        // outline wall has the correct direction - although this creates a little step, the end result is generally better because when the first
        // outline wall has the wrong direction (due to it starting from the finish point of the last layer) the visual effect is very noticeable
        Point2LL join_first_wall_at = LinearAlg2D::getClosestOnLineSegment(origin, wall[seam_vertex_idx % wall.size()], wall[(seam_vertex_idx + 1) % wall.size()]);
        if (vSize(join_first_wall_at - origin) > 10)
        {
            constexpr Ratio flow = 1.0_r;
            addExtrusionMove(join_first_wall_at, config, SpaceFillType::Polygons, flow, width_factor, spiralize);
        }
    }

    const int n_points = wall.size();
    Polygons last_wall_polygons;
    last_wall_polygons.add(last_wall);
    const int max_dist2 = config.getLineWidth() * config.getLineWidth() * 4; // (2 * lineWidth)^2;

    double total_length = 0.0; // determine the length of the complete wall
    Point2LL p0 = origin;
    for (int wall_point_idx = 1; wall_point_idx <= n_points; ++wall_point_idx)
    {
        const Point2LL& p1 = wall[(seam_vertex_idx + wall_point_idx) % n_points];
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

        const FanSpeedLayerTimeSettings& layer_time_settings = extruder_plans_.back().fan_speed_layer_time_settings_;
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
        const Point2LL& p = wall[(seam_vertex_idx + wall_point_idx) % n_points];
        wall_length += vSizeMM(p - p0);
        p0 = p;

        const double flow = (is_bottom_layer) ? (min_bottom_layer_flow + ((1 - min_bottom_layer_flow) * wall_length / total_length)) : 1.0;

        // if required, use interpolation to smooth the x/y coordinates between layers but not for the first spiralized layer
        // as that lies directly on top of a non-spiralized wall with exactly the same outline and not for the last point in each layer
        // because we want that to be numerically exactly the same as the starting point on the next layer (not subject to any rounding)
        if (smooth_contours && ! is_bottom_layer && wall_point_idx < n_points)
        {
            // now find the point on the last wall that is closest to p
            ClosestPolygonPoint cpp = PolygonUtils::findClosest(p, last_wall_polygons);

            // if we found a point and it's not further away than max_dist2, use it
            if (cpp.isValid() && vSize2(cpp.location_ - p) <= max_dist2)
            {
                // interpolate between cpp.location and p depending on how far we have progressed along wall
                addExtrusionMove(cpp.location_ + (p - cpp.location_) * (wall_length / total_length), config, SpaceFillType::Polygons, flow, width_factor, spiralize, speed_factor);
            }
            else
            {
                // no point in the last wall was found close enough to the current wall point so don't interpolate
                addExtrusionMove(p, config, SpaceFillType::Polygons, flow, width_factor, spiralize, speed_factor);
            }
        }
        else
        {
            // no smoothing, use point verbatim
            addExtrusionMove(p, config, SpaceFillType::Polygons, flow, width_factor, spiralize, speed_factor);
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
            const Point2LL& p = wall[(seam_vertex_idx + wall_point_idx) % n_points];
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
            constexpr bool no_spiralize = false;
            addExtrusionMove(p, config, SpaceFillType::Polygons, ((int)(flow * 20)) / 20.0, width_factor, no_spiralize, speed_factor);
        }
    }
}

void ExtruderPlan::forceMinimalLayerTime(double minTime, double time_other_extr_plans)
{
    const double minimalSpeed = fan_speed_layer_time_settings_.cool_min_speed;
    const double travelTime = estimates_.getTravelTime();
    const double extrudeTime = estimates_.extrude_time;

    const double totalTime = travelTime + extrudeTime + time_other_extr_plans;
    constexpr double epsilon = 0.01;

    double total_extrude_time_at_minimum_speed = 0.0;
    double total_extrude_time_at_slowest_speed = 0.0;
    for (GCodePath& path : paths_)
    {
        total_extrude_time_at_minimum_speed += path.estimates.extrude_time_at_minimum_speed;
        total_extrude_time_at_slowest_speed += path.estimates.extrude_time_at_slowest_path_speed;
    }

    if (totalTime < minTime - epsilon && extrudeTime > 0.0)
    {
        const double minExtrudeTime = minTime - (totalTime - extrudeTime);

        double target_speed = 0.0;
        std::function<double(const GCodePath&)> slow_down_func{ [&target_speed](const GCodePath& path)
                                                                {
                                                                    return std::min(target_speed / (path.config.getSpeed() * path.speed_factor), 1.0);
                                                                } };

        if (minExtrudeTime >= total_extrude_time_at_minimum_speed)
        {
            // Even at cool min speed extrusion is not taken enough time. So speed is set to cool min speed.
            target_speed = minimalSpeed;
            temperature_factor_ = 1.0;

            // Update stored naive time estimates
            estimates_.extrude_time = total_extrude_time_at_minimum_speed;
            if (minTime - total_extrude_time_at_minimum_speed - travelTime > epsilon)
            {
                extra_time_ = minTime - total_extrude_time_at_minimum_speed - travelTime;
            }
        }
        else if (minExtrudeTime >= total_extrude_time_at_slowest_speed && std::abs(total_extrude_time_at_minimum_speed - total_extrude_time_at_slowest_speed) >= epsilon)
        {
            // Slowing down to the slowest path speed is not sufficient, need to slow down further to the minimum speed.
            // Linear interpolate between total_extrude_time_at_slowest_speed and total_extrude_time_at_minimum_speed
            const double factor
                = (1 / total_extrude_time_at_minimum_speed - 1 / minExtrudeTime) / (1 / total_extrude_time_at_minimum_speed - 1 / total_extrude_time_at_slowest_speed);
            target_speed = minimalSpeed * (1.0 - factor) + slowest_path_speed_ * factor;
            temperature_factor_ = 1.0 - factor;

            // Update stored naive time estimates
            estimates_.extrude_time = minExtrudeTime;
        }
        else
        {
            // Slowing down to the slowest_speed is sufficient to respect the minimum layer time.
            // Linear interpolate between extrudeTime and total_extrude_time_at_slowest_speed
            const double factor = (1 / total_extrude_time_at_slowest_speed - 1 / minExtrudeTime) / (1 / total_extrude_time_at_slowest_speed - 1 / extrudeTime);
            slow_down_func = [&slowest_path_speed = slowest_path_speed_, factor](const GCodePath& path)
            {
                const double actual_target_speed = slowest_path_speed * (1.0 - factor) + (path.config.getSpeed() * path.speed_factor) * factor;
                return std::min(actual_target_speed / (path.config.getSpeed() * path.speed_factor), 1.0);
            };

            // Update stored naive time estimates
            estimates_.extrude_time = minExtrudeTime;
        }

        for (GCodePath& path : paths_)
        {
            if (path.isTravelPath())
            {
                continue;
            }
            Ratio slow_down_factor = slow_down_func(path);
            path.speed_factor *= slow_down_factor;
            path.estimates.extrude_time /= slow_down_factor;
        }
    }
}

double ExtruderPlan::getRetractTime(const GCodePath& path)
{
    return retraction_config_.distance / (path.retract ? retraction_config_.speed : retraction_config_.primeSpeed);
}

std::pair<double, double> ExtruderPlan::getPointToPointTime(const Point2LL& p0, const Point2LL& p1, const GCodePath& path)
{
    const double length = vSizeMM(p0 - p1);
    return { length, length / (path.config.getSpeed() * path.speed_factor) };
}

TimeMaterialEstimates ExtruderPlan::computeNaiveTimeEstimates(Point2LL starting_position)
{
    Point2LL p0 = starting_position;

    const double min_path_speed = fan_speed_layer_time_settings_.cool_min_speed;
    slowest_path_speed_ = std::accumulate(
        paths_.begin(),
        paths_.end(),
        std::numeric_limits<double>::max(),
        [](double value, const GCodePath& path)
        {
            return path.isTravelPath() ? value : std::min(value, path.config.getSpeed().value * path.speed_factor);
        });

    bool was_retracted = false; // wrong assumption; won't matter that much. (TODO)
    for (GCodePath& path : paths_)
    {
        bool is_extrusion_path = false;
        double* path_time_estimate;
        double& material_estimate = path.estimates.material;

        path.estimates.extrude_time_at_minimum_speed = 0.0;
        path.estimates.extrude_time_at_slowest_path_speed = 0.0;

        if (! path.isTravelPath())
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
                    retract_unretract_time = retraction_config_.distance / retraction_config_.speed;
                }
                else
                {
                    retract_unretract_time = retraction_config_.distance / retraction_config_.primeSpeed;
                }
                path.estimates.retracted_travel_time += 0.5 * retract_unretract_time;
                path.estimates.unretracted_travel_time += 0.5 * retract_unretract_time;
            }
        }
        for (Point2LL& p1 : path.points)
        {
            double length = vSizeMM(p0 - p1);
            if (is_extrusion_path)
            {
                if (length > 0)
                {
                    path.estimates.extrude_time_at_minimum_speed += length / min_path_speed;
                    path.estimates.extrude_time_at_slowest_path_speed += length / slowest_path_speed_;
                }
                material_estimate += length * INT2MM(layer_thickness_) * INT2MM(path.config.getLineWidth());
            }
            double thisTime = length / (path.config.getSpeed() * path.speed_factor);
            *path_time_estimate += thisTime;
            p0 = p1;
        }
        estimates_ += path.estimates;
    }
    return estimates_;
}

void ExtruderPlan::processFanSpeedForMinimalLayerTime(Duration minTime, double time_other_extr_plans)
{
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
    double totalLayerTime = estimates_.getTotalTime() + time_other_extr_plans;
    if (totalLayerTime < minTime)
    {
        fan_speed = fan_speed_layer_time_settings_.cool_fan_speed_max;
    }
    else if (minTime >= fan_speed_layer_time_settings_.cool_min_layer_time_fan_speed_max)
    {
        // ignore gradual increase of fan speed
        return;
    }
    else if (totalLayerTime < fan_speed_layer_time_settings_.cool_min_layer_time_fan_speed_max)
    {
        // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
        double fan_speed_diff = fan_speed_layer_time_settings_.cool_fan_speed_max - fan_speed;
        double layer_time_diff = fan_speed_layer_time_settings_.cool_min_layer_time_fan_speed_max - minTime;
        double fraction_of_slope = (totalLayerTime - minTime) / layer_time_diff;
        fan_speed = fan_speed_layer_time_settings_.cool_fan_speed_max - fan_speed_diff * fraction_of_slope;
    }
}

void ExtruderPlan::processFanSpeedForFirstLayers()
{
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
    fan_speed = fan_speed_layer_time_settings_.cool_fan_speed_min;
    if (layer_nr_ < fan_speed_layer_time_settings_.cool_fan_full_layer
        && fan_speed_layer_time_settings_.cool_fan_full_layer > 0 // don't apply initial layer fan speed speedup if disabled.
        && ! is_raft_layer_ // don't apply initial layer fan speed speedup to raft, but to model layers
    )
    {
        // Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
        fan_speed = fan_speed_layer_time_settings_.cool_fan_speed_0
                  + (fan_speed - fan_speed_layer_time_settings_.cool_fan_speed_0) * std::max(LayerIndex(0), layer_nr_) / fan_speed_layer_time_settings_.cool_fan_full_layer;
    }
}

void LayerPlan::processFanSpeedAndMinimalLayerTime(Point2LL starting_position)
{
    // the minimum layer time behaviour is only applied to the last extruder.
    const size_t last_extruder_nr = ranges::max_element(
                                        extruder_plans_,
                                        [](const ExtruderPlan& a, const ExtruderPlan& b)
                                        {
                                            return a.extruder_nr_ < b.extruder_nr_;
                                        })
                                        ->extruder_nr_;
    unsigned int last_extruder_idx;
    double other_extr_plan_time = 0.0;
    Duration maximum_cool_min_layer_time;

    for (unsigned int extr_plan_idx = 0; extr_plan_idx < extruder_plans_.size(); extr_plan_idx++)
    {
        {
            ExtruderPlan& extruder_plan = extruder_plans_[extr_plan_idx];

            // Precalculate the time estimates. Don't call this function twice, since it is works cumulative.
            extruder_plan.computeNaiveTimeEstimates(starting_position);
            if (extruder_plan.extruder_nr_ == last_extruder_nr)
            {
                last_extruder_idx = extr_plan_idx;
            }
            else
            {
                other_extr_plan_time += extruder_plan.estimates_.getTotalTime();
            }
            maximum_cool_min_layer_time = std::max(maximum_cool_min_layer_time, extruder_plan.fan_speed_layer_time_settings_.cool_min_layer_time);

            // Modify fan speeds for the first layer(s)
            extruder_plan.processFanSpeedForFirstLayers();

            if (! extruder_plan.paths_.empty() && ! extruder_plan.paths_.back().points.empty())
            {
                starting_position = extruder_plan.paths_.back().points.back();
            }
        }
    }

    // apply minimum layer time behaviour
    ExtruderPlan& last_extruder_plan = extruder_plans_[last_extruder_idx];
    last_extruder_plan.forceMinimalLayerTime(maximum_cool_min_layer_time, other_extr_plan_time);
    last_extruder_plan.processFanSpeedForMinimalLayerTime(maximum_cool_min_layer_time, other_extr_plan_time);
}


void LayerPlan::writeGCode(GCodeExport& gcode)
{
    Communication* communication = Application::getInstance().communication_;
    communication->setLayerForSend(layer_nr_);
    communication->sendCurrentPosition(gcode.getPositionXY());
    gcode.setLayerNr(layer_nr_);

    gcode.writeLayerComment(layer_nr_);

    // flow-rate compensation
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    gcode.setFlowRateExtrusionSettings(
        mesh_group_settings.get<double>("flow_rate_max_extrusion_offset"),
        mesh_group_settings.get<Ratio>("flow_rate_extrusion_offset_factor")); // Offset is in mm.

    static LayerIndex layer_1{ 1 - static_cast<LayerIndex>(Raft::getTotalExtraLayers()) };
    if (layer_nr_ == layer_1 && mesh_group_settings.get<bool>("machine_heated_bed"))
    {
        constexpr bool wait = false;
        gcode.writeBedTemperatureCommand(mesh_group_settings.get<Temperature>("material_bed_temperature"), wait);
    }

    gcode.setZ(z_);

    std::optional<GCodePathConfig> last_extrusion_config = std::nullopt; // used to check whether we need to insert a TYPE comment in the gcode.

    size_t extruder_nr = gcode.getExtruderNr();
    const bool acceleration_enabled = mesh_group_settings.get<bool>("acceleration_enabled");
    const bool acceleration_travel_enabled = mesh_group_settings.get<bool>("acceleration_travel_enabled");
    const bool jerk_enabled = mesh_group_settings.get<bool>("jerk_enabled");
    const bool jerk_travel_enabled = mesh_group_settings.get<bool>("jerk_travel_enabled");
    std::shared_ptr<const SliceMeshStorage> current_mesh;

    for (size_t extruder_plan_idx = 0; extruder_plan_idx < extruder_plans_.size(); extruder_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans_[extruder_plan_idx];

        const RetractionAndWipeConfig* retraction_config
            = current_mesh ? &current_mesh->retraction_wipe_config : &storage_.retraction_wipe_config_per_extruder[extruder_plan.extruder_nr_];
        coord_t z_hop_height = retraction_config->retraction_config.zHop;

        if (extruder_nr != extruder_plan.extruder_nr_)
        {
            int prev_extruder = extruder_nr;
            extruder_nr = extruder_plan.extruder_nr_;

            gcode.ResetLastEValueAfterWipe(prev_extruder);

            const RetractionAndWipeConfig& prev_retraction_config = storage_.retraction_wipe_config_per_extruder[prev_extruder];
            if (prev_retraction_config.retraction_hop_after_extruder_switch)
            {
                z_hop_height = prev_retraction_config.extruder_switch_retraction_config.zHop;
                gcode.switchExtruder(extruder_nr, prev_retraction_config.extruder_switch_retraction_config, z_hop_height);
            }
            else
            {
                gcode.switchExtruder(extruder_nr, prev_retraction_config.extruder_switch_retraction_config);
            }

            { // require printing temperature to be met
                constexpr bool wait = true;
                gcode.writeTemperatureCommand(extruder_nr, extruder_plan.required_start_temperature_, wait);
            }

            if (extruder_plan.prev_extruder_standby_temp_)
            { // turn off previous extruder
                constexpr bool wait = false;
                Temperature prev_extruder_temp = *extruder_plan.prev_extruder_standby_temp_;
                const LayerIndex prev_layer_nr = (extruder_plan_idx == 0) ? layer_nr_ - 1 : layer_nr_;
                if (prev_layer_nr == storage_.max_print_height_per_extruder[prev_extruder])
                {
                    prev_extruder_temp = 0; // TODO ? should there be a setting for extruder_off_temperature ?
                }
                gcode.writeTemperatureCommand(prev_extruder, prev_extruder_temp, wait);
            }

            { // require printing temperature to be met
                constexpr bool wait = true;
                gcode.writeTemperatureCommand(extruder_nr, extruder_plan.required_start_temperature_, wait);
            }

            const double extra_prime_amount = retraction_config->retraction_config.distance ? retraction_config->switch_extruder_extra_prime_amount : 0;
            gcode.addExtraPrimeAmount(extra_prime_amount);
        }
        else if (extruder_plan_idx == 0)
        {
            const WipeScriptConfig& wipe_config = storage_.retraction_wipe_config_per_extruder[extruder_plan.extruder_nr_].wipe_config;
            if (wipe_config.clean_between_layers && gcode.getExtrudedVolumeAfterLastWipe(extruder_nr) > wipe_config.max_extrusion_mm3)
            {
                gcode.insertWipeScript(wipe_config);
                gcode.ResetLastEValueAfterWipe(extruder_nr);
            }
            else if (layer_nr_ != 0 && Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_.get<bool>("retract_at_layer_change"))
            {
                // only do the retract if the paths are not spiralized
                if (! mesh_group_settings.get<bool>("magic_spiralize"))
                {
                    gcode.writeRetraction(retraction_config->retraction_config);
                }
            }
        }
        gcode.writeFanCommand(extruder_plan.getFanSpeed());
        std::vector<GCodePath>& paths = extruder_plan.paths_;

        extruder_plan.inserts_.sort();

        const ExtruderTrain& extruder = Application::getInstance().current_slice_->scene.extruders[extruder_nr];

        bool update_extrusion_offset = true;

        double cumulative_path_time = 0.; // Time in seconds.
        const std::function<void(const double, const int64_t)> insertTempOnTime = [&](const double to_add, const int64_t path_idx)
        {
            cumulative_path_time += to_add;
            extruder_plan.handleInserts(path_idx, gcode, cumulative_path_time);
        };

        for (int64_t path_idx = 0; path_idx < paths.size(); path_idx++)
        {
            extruder_plan.handleInserts(path_idx, gcode);
            cumulative_path_time = 0.; // reset to 0 for current path.

            GCodePath& path = paths[path_idx];

            if (path.perform_prime)
            {
                gcode.writePrimeTrain(extruder.settings_.get<Velocity>("speed_travel"));
                // Don't update cumulative path time, as ComputeNaiveTimeEstimates also doesn't.
                gcode.writeRetraction(retraction_config->retraction_config);
            }

            if (z_ > 0)
            {
                gcode.setZ(z_ + path.z_offset);
            }

            if (! path.retract && path.config.isTravelPath() && path.points.size() == 1 && path.points[0] == gcode.getPositionXY() && (z_ + path.z_offset) == gcode.getPositionZ())
            {
                // ignore travel moves to the current location to avoid needless change of acceleration/jerk
                continue;
            }

            // In some cases we want to find the next non-travel move.
            size_t next_extrusion_idx = path_idx + 1;
            if ((acceleration_enabled && ! acceleration_travel_enabled) || (jerk_enabled && ! jerk_travel_enabled))
            {
                while (next_extrusion_idx < paths.size() && paths[next_extrusion_idx].config.isTravelPath())
                {
                    ++next_extrusion_idx;
                }
            }

            if (acceleration_enabled)
            {
                if (path.config.isTravelPath())
                {
                    if (acceleration_travel_enabled)
                    {
                        gcode.writeTravelAcceleration(path.config.getAcceleration());
                    }
                    else
                    {
                        // Use the acceleration of the first non-travel move *after* the travel.
                        if (next_extrusion_idx >= paths.size()) // Only travel moves for the remainder of the layer.
                        {
                            if (static_cast<bool>(next_layer_acc_jerk_))
                            {
                                gcode.writeTravelAcceleration(next_layer_acc_jerk_->first);
                            } // If the next layer has no extruded move, just keep the old acceleration. Should be very rare to have an empty layer.
                        }
                        else
                        {
                            gcode.writeTravelAcceleration(paths[next_extrusion_idx].config.getAcceleration());
                        }
                    }
                }
                else
                {
                    gcode.writePrintAcceleration(path.config.getAcceleration());
                }
            }
            if (jerk_enabled)
            {
                if (jerk_travel_enabled)
                {
                    gcode.writeJerk(path.config.getJerk());
                }
                else
                {
                    // Use the jerk of the first non-travel move *after* the travel.
                    if (next_extrusion_idx >= paths.size()) // Only travel moves for the remainder of the layer.
                    {
                        if (static_cast<bool>(next_layer_acc_jerk_))
                        {
                            gcode.writeJerk(next_layer_acc_jerk_->second);
                        } // If the next layer has no extruded move, just keep the old jerk. Should be very rare to have an empty layer.
                    }
                    else
                    {
                        gcode.writeJerk(paths[next_extrusion_idx].config.getJerk());
                    }
                }
            }

            if (path.retract)
            {
                retraction_config = path.mesh ? &path.mesh->retraction_wipe_config : retraction_config;
                gcode.writeRetraction(retraction_config->retraction_config);
                insertTempOnTime(extruder_plan.getRetractTime(path), path_idx);
                if (path.perform_z_hop)
                {
                    gcode.writeZhopStart(z_hop_height);
                    z_hop_height = retraction_config->retraction_config.zHop; // back to normal z hop
                }
                else
                {
                    gcode.writeZhopEnd();
                }
            }
            const auto& extruder_changed = ! last_extrusion_config.has_value() || (last_extrusion_config.value().type != path.config.type);
            if (! path.config.isTravelPath() && extruder_changed)
            {
                gcode.writeTypeComment(path.config.type);
                if (path.config.isBridgePath())
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

            double speed = path.config.getSpeed();

            // for some movements such as prime tower purge, the speed may get changed by this factor
            speed *= path.speed_factor;

            // This seems to be the best location to place this, but still not ideal.
            if (path.mesh != current_mesh)
            {
                current_mesh = path.mesh;
                std::stringstream ss;
                ss << "MESH:" << (current_mesh ? current_mesh->mesh_name : "NONMESH");
                gcode.writeComment(ss.str());
            }
            if (path.config.isTravelPath())
            { // early comp for travel paths, which are handled more simply
                if (! path.perform_z_hop && final_travel_z_ != z_ && extruder_plan_idx == (extruder_plans_.size() - 1) && path_idx == (paths.size() - 1))
                {
                    // Before the final travel, move up to the next layer height, on the current spot, with a sensible speed.
                    Point3LL current_position = gcode.getPosition();
                    current_position.z_ = final_travel_z_;
                    gcode.writeTravel(current_position, extruder.settings_.get<Velocity>("speed_z_hop"));

                    // Prevent the final travel(s) from resetting to the 'previous' layer height.
                    gcode.setZ(final_travel_z_);
                }
                for (size_t point_idx = 0; point_idx + 1 < path.points.size(); point_idx++)
                {
                    gcode.writeTravel(path.points[point_idx], speed);
                }
                if (path.unretract_before_last_travel_move && final_travel_z_ == z_)
                {
                    // We need to unretract before the last travel move of the path if the next path is an outer wall.
                    gcode.writeUnretractionAndPrime();
                }
                gcode.writeTravel(path.points.back(), speed);
                continue;
            }

            bool spiralize = path.spiralize;
            if (! spiralize) // normal (extrusion) move (with coasting)
            {
                // if path provides a valid (in range 0-100) fan speed, use it
                const double path_fan_speed = path.getFanSpeed();
                gcode.writeFanCommand(path_fan_speed != GCodePathConfig::FAN_SPEED_DEFAULT ? path_fan_speed : extruder_plan.getFanSpeed());

                bool coasting = extruder.settings_.get<bool>("coasting_enable");
                if (coasting)
                {
                    coasting = writePathWithCoasting(gcode, extruder_plan_idx, path_idx, layer_thickness_, insertTempOnTime);
                }
                if (! coasting) // not same as 'else', cause we might have changed [coasting] in the line above...
                { // normal path to gcode algorithm
                    Point2LL prev_point = gcode.getPositionXY();
                    for (unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                    {
                        const auto [_, time] = extruder_plan.getPointToPointTime(prev_point, path.points[point_idx], path);
                        insertTempOnTime(time, path_idx);

                        const double extrude_speed = speed * path.speed_back_pressure_factor;
                        communication->sendLineTo(path.config.type, path.points[point_idx], path.getLineWidthForLayerView(), path.config.getLayerThickness(), extrude_speed);
                        gcode.writeExtrusion(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM(), path.config.type, update_extrusion_offset);

                        prev_point = path.points[point_idx];
                    }
                }
            }
            else
            { // SPIRALIZE
                // If we need to spiralize then raise the head slowly by 1 layer as this path progresses.
                double totalLength = 0.0;
                Point2LL p0 = gcode.getPositionXY();
                for (unsigned int _path_idx = path_idx; _path_idx < paths.size() && ! paths[_path_idx].isTravelPath(); _path_idx++)
                {
                    GCodePath& _path = paths[_path_idx];
                    for (unsigned int point_idx = 0; point_idx < _path.points.size(); point_idx++)
                    {
                        Point2LL p1 = _path.points[point_idx];
                        totalLength += vSizeMM(p0 - p1);
                        p0 = p1;
                    }
                }

                double length = 0.0;
                p0 = gcode.getPositionXY();
                for (; path_idx < paths.size() && paths[path_idx].spiralize; path_idx++)
                { // handle all consecutive spiralized paths > CHANGES path_idx!
                    GCodePath& spiral_path = paths[path_idx];

                    for (unsigned int point_idx = 0; point_idx < spiral_path.points.size(); point_idx++)
                    {
                        const Point2LL p1 = spiral_path.points[point_idx];
                        length += vSizeMM(p0 - p1);
                        p0 = p1;
                        gcode.setZ(std::round(z_ + layer_thickness_ * length / totalLength));

                        const double extrude_speed = speed * spiral_path.speed_back_pressure_factor;
                        communication->sendLineTo(
                            spiral_path.config.type,
                            spiral_path.points[point_idx],
                            spiral_path.getLineWidthForLayerView(),
                            spiral_path.config.getLayerThickness(),
                            extrude_speed);
                        gcode.writeExtrusion(spiral_path.points[point_idx], extrude_speed, spiral_path.getExtrusionMM3perMM(), spiral_path.config.type, update_extrusion_offset);
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
                    communication
                        ->sendLineTo(spiral_path.config.type, spiral_path.points[0], spiral_path.getLineWidthForLayerView(), spiral_path.config.getLayerThickness(), speed);
                }
                path_idx--; // the last path_idx didnt spiralize, so it's not part of the current spiralize path
            }
        } // paths for this extruder /\  .

        if (extruder.settings_.get<bool>("cool_lift_head") && extruder_plan.extra_time_ > 0.0)
        {
            gcode.writeComment("Small layer, adding delay");
            const RetractionAndWipeConfig& actual_retraction_config
                = current_mesh ? current_mesh->retraction_wipe_config : storage_.retraction_wipe_config_per_extruder[gcode.getExtruderNr()];
            gcode.writeRetraction(actual_retraction_config.retraction_config);
            if (extruder_plan_idx == extruder_plans_.size() - 1 || ! extruder.settings_.get<bool>("machine_extruder_end_pos_abs"))
            { // only do the z-hop if it's the last extruder plan; otherwise it's already at the switching bay area
                // or do it anyway when we switch extruder in-place
                gcode.writeZhopStart(MM2INT(3.0));
            }
            gcode.writeDelay(extruder_plan.extra_time_);
        }

        extruder_plan.handleAllRemainingInserts(gcode);
        scripta::log(
            "extruder_plan_2",
            extruder_plan.paths_,
            SectionType::NA,
            layer_nr_,
            scripta::CellVDI{ "flow", &GCodePath::flow },
            scripta::CellVDI{ "width_factor", &GCodePath::width_factor },
            scripta::CellVDI{ "spiralize", &GCodePath::spiralize },
            scripta::CellVDI{ "speed_factor", &GCodePath::speed_factor },
            scripta::CellVDI{ "speed_back_pressure_factor", &GCodePath::speed_back_pressure_factor },
            scripta::CellVDI{ "retract", &GCodePath::retract },
            scripta::CellVDI{ "unretract_before_last_travel_move", &GCodePath::unretract_before_last_travel_move },
            scripta::CellVDI{ "perform_z_hop", &GCodePath::perform_z_hop },
            scripta::CellVDI{ "perform_prime", &GCodePath::perform_prime },
            scripta::CellVDI{ "fan_speed", &GCodePath::getFanSpeed },
            scripta::CellVDI{ "is_travel_path", &GCodePath::isTravelPath },
            scripta::CellVDI{ "extrusion_mm3_per_mm", &GCodePath::getExtrusionMM3perMM });
    } // extruder plans /\  .

    communication->sendLayerComplete(layer_nr_, z_, layer_thickness_);
    gcode.updateTotalPrintTime();
}

void LayerPlan::overrideFanSpeeds(double speed)
{
    for (ExtruderPlan& extruder_plan : extruder_plans_)
    {
        extruder_plan.setFanSpeed(speed);
    }
}


bool LayerPlan::makeRetractSwitchRetract(unsigned int extruder_plan_idx, unsigned int path_idx)
{
    std::vector<GCodePath>& paths = extruder_plans_[extruder_plan_idx].paths_;
    for (unsigned int path_idx2 = path_idx + 1; path_idx2 < paths.size(); path_idx2++)
    {
        if (paths[path_idx2].getExtrusionMM3perMM() > 0)
        {
            return false;
        }
    }

    if (extruder_plans_.size() <= extruder_plan_idx + 1)
    {
        return false; // TODO: check first extruder of the next layer! (generally only on the last layer of the second extruder)
    }

    if (extruder_plans_[extruder_plan_idx + 1].extruder_nr_ != extruder_plans_[extruder_plan_idx].extruder_nr_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool LayerPlan::writePathWithCoasting(
    GCodeExport& gcode,
    const size_t extruder_plan_idx,
    const size_t path_idx,
    const coord_t layer_thickness,
    const std::function<void(const double, const int64_t)> insertTempOnTime)
{
    ExtruderPlan& extruder_plan = extruder_plans_[extruder_plan_idx];
    const ExtruderTrain& extruder = Application::getInstance().current_slice_->scene.extruders[extruder_plan.extruder_nr_];
    const double coasting_volume = extruder.settings_.get<double>("coasting_volume");
    if (coasting_volume <= 0)
    {
        return false;
    }
    const std::vector<GCodePath>& paths = extruder_plan.paths_;
    const GCodePath& path = paths[path_idx];
    if (path_idx + 1 >= paths.size() || (path.isTravelPath() || ! paths[path_idx + 1].config.isTravelPath()) || path.points.size() < 2)
    {
        return false;
    }

    coord_t coasting_min_dist_considered = MM2INT(0.1); // hardcoded setting for when to not perform coasting

    const double extrude_speed = path.config.getSpeed() * path.speed_factor * path.speed_back_pressure_factor;

    const coord_t coasting_dist
        = MM2INT(MM2_2INT(coasting_volume) / layer_thickness) / path.config.getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    const double coasting_min_volume = extruder.settings_.get<double>("coasting_min_volume");
    const coord_t coasting_min_dist
        = MM2INT(MM2_2INT(coasting_min_volume + coasting_volume) / layer_thickness) / path.config.getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    //           /\ the minimal distance when coasting will coast the full coasting volume instead of linearly less with linearly smaller paths

    std::vector<coord_t> accumulated_dist_per_point; // the first accumulated dist is that of the last point! (that of the last point is always zero...)
    accumulated_dist_per_point.push_back(0);

    coord_t accumulated_dist = 0;

    bool length_is_less_than_min_dist = true;

    std::optional<size_t> acc_dist_idx_gt_coast_dist; // the index of the first point with accumulated_dist more than coasting_dist (= index into accumulated_dist_per_point)
                                                      // == the point printed BEFORE the start point for coasting

    const Point2LL* last = &path.points[path.points.size() - 1];
    for (unsigned int backward_point_idx = 1; backward_point_idx < path.points.size(); backward_point_idx++)
    {
        const Point2LL& point = path.points[path.points.size() - 1 - backward_point_idx];
        const coord_t distance = vSize(point - *last);
        accumulated_dist += distance;
        accumulated_dist_per_point.push_back(accumulated_dist);

        if (! acc_dist_idx_gt_coast_dist.has_value() && accumulated_dist >= coasting_dist)
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
        if (actual_coasting_dist == 0) // Downscaling due to Minimum Coasting Distance reduces coasting to less than 1 micron.
        {
            return false; // Skip coasting at all then.
        }
        for (acc_dist_idx_gt_coast_dist = 1; acc_dist_idx_gt_coast_dist.value() < accumulated_dist_per_point.size(); acc_dist_idx_gt_coast_dist.value()++)
        { // search for the correct coast_dist_idx
            if (accumulated_dist_per_point[acc_dist_idx_gt_coast_dist.value()] >= actual_coasting_dist)
            {
                break;
            }
        }
    }

    assert(
        acc_dist_idx_gt_coast_dist.has_value() && acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size()); // something has gone wrong; coasting_min_dist < coasting_dist ?

    const size_t point_idx_before_start = path.points.size() - 1 - acc_dist_idx_gt_coast_dist.value();

    Point2LL start;
    { // computation of begin point of coasting
        const coord_t residual_dist = actual_coasting_dist - accumulated_dist_per_point[acc_dist_idx_gt_coast_dist.value() - 1];
        const Point2LL& a = path.points[point_idx_before_start];
        const Point2LL& b = path.points[point_idx_before_start + 1];
        start = b + normal(a - b, residual_dist);
    }

    Point2LL prev_pt = gcode.getPositionXY();
    { // write normal extrude path:
        Communication* communication = Application::getInstance().communication_;
        for (size_t point_idx = 0; point_idx <= point_idx_before_start; point_idx++)
        {
            auto [_, time] = extruder_plan.getPointToPointTime(prev_pt, path.points[point_idx], path);
            insertTempOnTime(time, path_idx);

            communication->sendLineTo(path.config.type, path.points[point_idx], path.getLineWidthForLayerView(), path.config.getLayerThickness(), extrude_speed);
            gcode.writeExtrusion(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM(), path.config.type);

            prev_pt = path.points[point_idx];
        }
        communication->sendLineTo(path.config.type, start, path.getLineWidthForLayerView(), path.config.getLayerThickness(), extrude_speed);
        gcode.writeExtrusion(start, extrude_speed, path.getExtrusionMM3perMM(), path.config.type);
    }

    // write coasting path
    for (size_t point_idx = point_idx_before_start + 1; point_idx < path.points.size(); point_idx++)
    {
        auto [_, time] = extruder_plan.getPointToPointTime(prev_pt, path.points[point_idx], path);
        insertTempOnTime(time, path_idx);

        const Ratio coasting_speed_modifier = extruder.settings_.get<Ratio>("coasting_speed");
        const Velocity speed = Velocity(coasting_speed_modifier * path.config.getSpeed());
        gcode.writeTravel(path.points[point_idx], speed);

        prev_pt = path.points[point_idx];
    }
    return true;
}

void LayerPlan::applyModifyPlugin()
{
    for (auto& extruder_plan : extruder_plans_)
    {
        scripta::log(
            "extruder_plan_0",
            extruder_plan.paths_,
            SectionType::NA,
            layer_nr_,
            scripta::CellVDI{ "flow", &GCodePath::flow },
            scripta::CellVDI{ "width_factor", &GCodePath::width_factor },
            scripta::CellVDI{ "spiralize", &GCodePath::spiralize },
            scripta::CellVDI{ "speed_factor", &GCodePath::speed_factor },
            scripta::CellVDI{ "speed_back_pressure_factor", &GCodePath::speed_back_pressure_factor },
            scripta::CellVDI{ "retract", &GCodePath::retract },
            scripta::CellVDI{ "unretract_before_last_travel_move", &GCodePath::unretract_before_last_travel_move },
            scripta::CellVDI{ "perform_z_hop", &GCodePath::perform_z_hop },
            scripta::CellVDI{ "perform_prime", &GCodePath::perform_prime },
            scripta::CellVDI{ "fan_speed", &GCodePath::getFanSpeed },
            scripta::CellVDI{ "is_travel_path", &GCodePath::isTravelPath },
            scripta::CellVDI{ "extrusion_mm3_per_mm", &GCodePath::getExtrusionMM3perMM });

        extruder_plan.paths_ = slots::instance().modify<plugins::v0::SlotID::GCODE_PATHS_MODIFY>(extruder_plan.paths_, extruder_plan.extruder_nr_, layer_nr_);

        scripta::log(
            "extruder_plan_1",
            extruder_plan.paths_,
            SectionType::NA,
            layer_nr_,
            scripta::CellVDI{ "flow", &GCodePath::flow },
            scripta::CellVDI{ "width_factor", &GCodePath::width_factor },
            scripta::CellVDI{ "spiralize", &GCodePath::spiralize },
            scripta::CellVDI{ "speed_factor", &GCodePath::speed_factor },
            scripta::CellVDI{ "speed_back_pressure_factor", &GCodePath::speed_back_pressure_factor },
            scripta::CellVDI{ "retract", &GCodePath::retract },
            scripta::CellVDI{ "unretract_before_last_travel_move", &GCodePath::unretract_before_last_travel_move },
            scripta::CellVDI{ "perform_z_hop", &GCodePath::perform_z_hop },
            scripta::CellVDI{ "perform_prime", &GCodePath::perform_prime },
            scripta::CellVDI{ "fan_speed", &GCodePath::getFanSpeed },
            scripta::CellVDI{ "is_travel_path", &GCodePath::isTravelPath },
            scripta::CellVDI{ "extrusion_mm3_per_mm", &GCodePath::getExtrusionMM3perMM });
    }
}

void LayerPlan::applyBackPressureCompensation()
{
    for (auto& extruder_plan : extruder_plans_)
    {
        const Ratio back_pressure_compensation
            = Application::getInstance().current_slice_->scene.extruders[extruder_plan.extruder_nr_].settings_.get<Ratio>("speed_equalize_flow_width_factor");
        if (back_pressure_compensation != 0.0)
        {
            extruder_plan.applyBackPressureCompensation(back_pressure_compensation);
        }
    }
}

LayerIndex LayerPlan::getLayerNr() const
{
    return layer_nr_;
}

Point2LL LayerPlan::getLastPlannedPositionOrStartingPosition() const
{
    return last_planned_position_.value_or(layer_start_pos_per_extruder_[getExtruder()]);
}

bool LayerPlan::getIsInsideMesh() const
{
    return was_inside_;
}

bool LayerPlan::getSkirtBrimIsPlanned(unsigned int extruder_nr) const
{
    return skirt_brim_is_processed_[extruder_nr];
}

void LayerPlan::setSkirtBrimIsPlanned(unsigned int extruder_nr)
{
    skirt_brim_is_processed_[extruder_nr] = true;
}

size_t LayerPlan::getExtruder() const
{
    return extruder_plans_.back().extruder_nr_;
}

void LayerPlan::setBridgeWallMask(const Polygons& polys)
{
    bridge_wall_mask_ = polys;
}

void LayerPlan::setOverhangMask(const Polygons& polys)
{
    overhang_mask_ = polys;
}

void LayerPlan::setRoofingMask(const Polygons& polys)
{
    roofing_mask_ = polys;
}

} // namespace cura
