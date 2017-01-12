/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include <cstring>
#include "gcodePlanner.h"
#include "pathOrderOptimizer.h"
#include "sliceDataStorage.h"
#include "utils/polygonUtils.h"
#include "MergeInfillLines.h"
#include "raft.h" // getTotalExtraLayers

namespace cura {


ExtruderPlan::ExtruderPlan(int extruder, Point start_position, int layer_nr, bool is_initial_layer, int layer_thickness, FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config)
: extruder(extruder)
, heated_pre_travel_time(0)
, initial_printing_temperature(-1)
, printing_temperature(-1)
, start_position(start_position)
, layer_nr(layer_nr)
, is_initial_layer(is_initial_layer)
, layer_thickness(layer_thickness)
, fan_speed_layer_time_settings(fan_speed_layer_time_settings)
, retraction_config(retraction_config)
, extrudeSpeedFactor(1.0)
, travelSpeedFactor(1.0)
, extraTime(0.0)
, totalPrintTime(0)
{
}

void ExtruderPlan::setExtrudeSpeedFactor(double speedFactor)
{
    this->extrudeSpeedFactor = speedFactor;
}
double ExtruderPlan::getExtrudeSpeedFactor()
{
    return this->extrudeSpeedFactor;
}
void ExtruderPlan::setTravelSpeedFactor(double speedFactor)
{
    if (speedFactor < 1) speedFactor = 1.0;
    this->travelSpeedFactor = speedFactor;
}
double ExtruderPlan::getTravelSpeedFactor()
{
    return this->travelSpeedFactor;
}

void ExtruderPlan::setFanSpeed(double _fan_speed)
{
    fan_speed = _fan_speed;
}
double ExtruderPlan::getFanSpeed()
{
    return fan_speed;
}


GCodePath* GCodePlanner::getLatestPathWithConfig(GCodePathConfig* config, SpaceFillType space_fill_type, float flow, bool spiralize)
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0 && paths.back().config == config && !paths.back().done && paths.back().flow == flow) // spiralize can only change when a travel path is in between
        return &paths.back();
    paths.emplace_back();
    GCodePath* ret = &paths.back();
    ret->retract = false;
    ret->perform_prime = false;
    ret->perform_z_hop = false;
    ret->config = config;
    ret->done = false;
    ret->flow = flow;
    ret->spiralize = spiralize;
    ret->space_fill_type = space_fill_type;
    return ret;
}

void GCodePlanner::forceNewPathStart()
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0)
        paths[paths.size()-1].done = true;
}

GCodePlanner::GCodePlanner(SliceDataStorage& storage, int layer_nr, int z, int layer_thickness, Point last_position, int current_extruder, bool is_inside_mesh, std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, CombingMode combing_mode, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance)
: storage(storage)
, layer_nr(layer_nr)
, is_initial_layer(layer_nr == 0 - Raft::getTotalExtraLayers(storage))
, z(z)
, layer_thickness(layer_thickness)
, start_position(last_position)
, lastPosition(last_position)
, last_extruder_previous_layer(current_extruder)
, last_planned_extruder_setting_base(storage.meshgroup->getExtruderTrain(current_extruder))
, comb_boundary_inside(computeCombBoundaryInside(combing_mode))
, fan_speed_layer_time_settings_per_extruder(fan_speed_layer_time_settings_per_extruder)
{
    extruder_plans.reserve(storage.meshgroup->getExtruderCount());
    extruder_plans.emplace_back(current_extruder, start_position, layer_nr, is_initial_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[current_extruder], storage.retraction_config_per_extruder[current_extruder]);
    comb = nullptr;
    was_inside = is_inside_mesh; 
    is_inside = false; // assumes the next move will not be to inside a layer part (overwritten just before going into a layer part)
    if (combing_mode != CombingMode::OFF)
    {
        comb = new Comb(storage, layer_nr, comb_boundary_inside, comb_boundary_offset, travel_avoid_other_parts, travel_avoid_distance);
    }
    else
        comb = nullptr;
}

GCodePlanner::~GCodePlanner()
{
    if (comb)
        delete comb;
}

SettingsBaseVirtual* GCodePlanner::getLastPlannedExtruderTrainSettings()
{
    return last_planned_extruder_setting_base;
}


Polygons GCodePlanner::computeCombBoundaryInside(CombingMode combing_mode)
{
    if (combing_mode == CombingMode::OFF)
    {
        return Polygons();
    }
    if (layer_nr < 0)
    { // when a raft is present
        if (combing_mode == CombingMode::NO_SKIN)
        {
            return Polygons();
        }
        else
        {
            return storage.raftOutline.offset(MM2INT(0.1));
        }
    }
    else 
    {
        Polygons comb_boundary;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[layer_nr];
            if (mesh.getSettingAsCombingMode("retraction_combing") == CombingMode::NO_SKIN)
            {
                for (SliceLayerPart& part : layer.parts)
                {
                    comb_boundary.add(part.infill_area);
                }
            }
            else
            {
                if (mesh.getSettingBoolean("infill_mesh"))
                {
                    continue;
                }
                layer.getSecondOrInnermostWalls(comb_boundary);
            }
        }
        return comb_boundary;
    }
}

void GCodePlanner::setIsInside(bool _is_inside)
{
    is_inside = _is_inside;
}

bool GCodePlanner::setExtruder(int extruder)
{
    if (extruder == getExtruder())
    {
        return false;
    }
    setIsInside(false);
    { // handle end position of the prev extruder
        SettingsBaseVirtual* train = getLastPlannedExtruderTrainSettings();
        bool end_pos_absolute = train->getSettingBoolean("machine_extruder_end_pos_abs");
        Point end_pos(train->getSettingInMicrons("machine_extruder_end_pos_x"), train->getSettingInMicrons("machine_extruder_end_pos_y"));
        if (!end_pos_absolute)
        {
            end_pos += lastPosition;
        }
        else 
        {
            Point extruder_offset(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
            end_pos += extruder_offset; // absolute end pos is given as a head position
        }
        addTravel(end_pos); //  + extruder_offset cause it 
    }
    if (extruder_plans.back().paths.empty() && extruder_plans.back().inserts.empty())
    { // first extruder plan in a layer might be empty, cause it is made with the last extruder planned in the previous layer
        extruder_plans.back().extruder = extruder;
    }
    else 
    {
        extruder_plans.emplace_back(extruder, lastPosition, layer_nr, is_initial_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[extruder], storage.retraction_config_per_extruder[extruder]);
        assert((int)extruder_plans.size() <= storage.meshgroup->getExtruderCount() && "Never use the same extruder twice on one layer!");
    }
    last_planned_extruder_setting_base = storage.meshgroup->getExtruderTrain(extruder);

//     forceNewPathStart(); // automatic by the fact that we start a new ExtruderPlan

    { // handle starting pos of the new extruder
        SettingsBaseVirtual* train = getLastPlannedExtruderTrainSettings();
        bool start_pos_absolute = train->getSettingBoolean("machine_extruder_start_pos_abs");
        Point start_pos(train->getSettingInMicrons("machine_extruder_start_pos_x"), train->getSettingInMicrons("machine_extruder_start_pos_y"));
        if (!start_pos_absolute)
        {
            start_pos += lastPosition;
        }
        else 
        {
            Point extruder_offset(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
            start_pos += extruder_offset; // absolute start pos is given as a head position
        }
        lastPosition = start_pos;
    }
    return true;
}

void GCodePlanner::moveInsideCombBoundary(int distance)
{
    int max_dist2 = MM2INT(2.0) * MM2INT(2.0); // if we are further than this distance, we conclude we are not inside even though we thought we were.
    // this function is to be used to move from the boudary of a part to inside the part
    Point p = lastPosition; // copy, since we are going to move p
    if (PolygonUtils::moveInside(comb_boundary_inside, p, distance, max_dist2) != NO_INDEX)
    {
        //Move inside again, so we move out of tight 90deg corners
        PolygonUtils::moveInside(comb_boundary_inside, p, distance, max_dist2);
        if (comb_boundary_inside.inside(p))
        {
            addTravel_simple(p);
            //Make sure the that any retraction happens after this move, not before it by starting a new move path.
            forceNewPathStart();
        }
    }
}

GCodePath& GCodePlanner::addTravel(Point p)
{
    GCodePath* path = nullptr;
    GCodePathConfig& travel_config = storage.travel_config_per_extruder[getExtruder()];
    RetractionConfig& retraction_config = storage.retraction_config_per_extruder[getExtruder()];
    
    bool combed = false;

    SettingsBaseVirtual* extr = getLastPlannedExtruderTrainSettings();

    const bool perform_z_hops = extr->getSettingBoolean("retraction_hop_enabled");

    const bool is_first_travel_of_extruder_after_switch = extruder_plans.back().paths.size() == 0 && (extruder_plans.size() > 1 || last_extruder_previous_layer != getExtruder());
    const bool bypass_combing = is_first_travel_of_extruder_after_switch && extr->getSettingBoolean("retraction_hop_after_extruder_switch");

    if (comb != nullptr && !bypass_combing && lastPosition != no_point)
    {
        const bool perform_z_hops_only_when_collides = extr->getSettingBoolean("retraction_hop_only_when_collides");

        CombPaths combPaths;
        bool via_outside_makes_combing_fail = perform_z_hops && !perform_z_hops_only_when_collides;
        bool fail_on_unavoidable_obstacles = perform_z_hops && perform_z_hops_only_when_collides;
        combed = comb->calc(lastPosition, p, combPaths, was_inside, is_inside, retraction_config.retraction_min_travel_distance, via_outside_makes_combing_fail, fail_on_unavoidable_obstacles);
        if (combed)
        {
            bool retract = combPaths.size() > 1;
            if (!retract)
            { // check whether we want to retract
                if (combPaths.throughAir)
                {
                    retract = true;
                }
                else
                {
                    for (CombPath& combPath : combPaths)
                    { // retract when path moves through a boundary
                        if (combPath.cross_boundary)
                        {
                            retract = true;
                            break;
                        }
                    }
                }
                if (combPaths.size() == 1)
                {
                    CombPath path = combPaths[0];
                    if (combPaths.throughAir && !path.cross_boundary && path.size() == 2 && path[0] == lastPosition && path[1] == p)
                    { // limit the retractions from support to support, which didn't cross anything
                        retract = false;
                    }
                }
            }

            for (CombPath& combPath : combPaths)
            { // add all comb paths (don't do anything special for paths which are moving through air)
                if (combPath.size() == 0)
                {
                    continue;
                }
                path = getLatestPathWithConfig(&travel_config, SpaceFillType::None);
                path->retract = retract;
                // don't perform a z-hop
                for (Point& combPoint : combPath)
                {
                    path->points.push_back(combPoint);
                }
                lastPosition = combPath.back();
            }
        }
    }
    
    if (!combed) {
        // no combing? always retract!
        if (!shorterThen(lastPosition - p, retraction_config.retraction_min_travel_distance))
        {
            if (was_inside) // when the previous location was from printing something which is considered inside (not support or prime tower etc)
            {               // then move inside the printed part, so that we don't ooze on the outer wall while retraction, but on the inside of the print.
                assert (extr != nullptr);
                moveInsideCombBoundary(extr->getSettingInMicrons((extr->getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0") * 1);
            }
            path = getLatestPathWithConfig(&travel_config, SpaceFillType::None);
            path->retract = true;
            path->perform_z_hop = perform_z_hops;
        }
    }

    GCodePath& ret = addTravel_simple(p, path);
    was_inside = is_inside;
    return ret;
}

GCodePath& GCodePlanner::addTravel_simple(Point p, GCodePath* path)
{
    if (path == nullptr)
    {
        path = getLatestPathWithConfig(&storage.travel_config_per_extruder[getExtruder()], SpaceFillType::None);
    }
    path->points.push_back(p);
    lastPosition = p;
    return *path;
}

void GCodePlanner::planPrime()
{
    forceNewPathStart();
    GCodePath& prime_travel = addTravel_simple(lastPosition + Point(0, 100));
    prime_travel.retract = false;
    prime_travel.perform_prime = true;
    forceNewPathStart();
}

void GCodePlanner::addExtrusionMove(Point p, GCodePathConfig* config, SpaceFillType space_fill_type, float flow, bool spiralize)
{
    getLatestPathWithConfig(config, space_fill_type, flow, spiralize)->points.push_back(p);
    lastPosition = p;
}

void GCodePlanner::addPolygon(PolygonRef polygon, int start_idx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation, coord_t wall_0_wipe_dist, bool spiralize, float flow_ratio)
{
    Point p0 = polygon[start_idx];
    addTravel(p0);
    for (unsigned int point_idx = 1; point_idx < polygon.size(); point_idx++)
    {
        Point p1 = polygon[(start_idx + point_idx) % polygon.size()];
        float flow = (wall_overlap_computation)? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow, spiralize);
        p0 = p1;
    }
    if (polygon.size() > 2)
    {
        Point& p1 = polygon[start_idx];
        float flow = (wall_overlap_computation)? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow, spiralize);

        if (wall_0_wipe_dist > 0)
        { // apply outer wall wipe
            p0 = polygon[start_idx];
            int distance_traversed = 0;
            for (unsigned int point_idx = 1; ; point_idx++)
            {
                Point p1 = polygon[(start_idx + point_idx) % polygon.size()];
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
        logWarning("WARNING: line added as polygon! (gcodePlanner)\n");
    }
}

void GCodePlanner::addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation, EZSeamType z_seam_type, Point z_seam_pos, coord_t wall_0_wipe_dist, bool spiralize, float flow_ratio)
{
    if (polygons.size() == 0)
    {
        return;
    }
    PathOrderOptimizer orderOptimizer(lastPosition, z_seam_pos, z_seam_type);
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(polygons[poly_idx]);
    }
    orderOptimizer.optimize();
    for (unsigned int poly_idx : orderOptimizer.polyOrder)
    {
        addPolygon(polygons[poly_idx], orderOptimizer.polyStart[poly_idx], config, wall_overlap_computation, wall_0_wipe_dist, spiralize, flow_ratio);
    }
}
void GCodePlanner::addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config, SpaceFillType space_fill_type, int wipe_dist, float flow_ratio)
{
    LineOrderOptimizer orderOptimizer(lastPosition);
    for (unsigned int line_idx = 0; line_idx < polygons.size(); line_idx++)
    {
        orderOptimizer.addPolygon(polygons[line_idx]);
    }
    orderOptimizer.optimize();
    for (int poly_idx : orderOptimizer.polyOrder)
    {
        PolygonRef polygon = polygons[poly_idx];
        int start = orderOptimizer.polyStart[poly_idx];
        int end = 1 - start;
        Point& p0 = polygon[start];
        addTravel(p0);
        Point& p1 = polygon[end];
        addExtrusionMove(p1, config, space_fill_type, flow_ratio);
        if (wipe_dist != 0)
        {
            int line_width = config->getLineWidth();
            if (vSize2(p1-p0) > line_width * line_width * 4)
            { // otherwise line will get optimized by combining multiple into a single extrusion move
                addExtrusionMove(p1 + normal(p1-p0, wipe_dist), config, space_fill_type, 0.0);
            }
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
            double speed = path.config->getSpeed() * factor;
            if (speed < minimalSpeed)
                factor = minimalSpeed / path.config->getSpeed();
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
            this->extraTime = minTime - (extrudeTime * inv_factor) - travelTime;
        }
        this->totalPrintTime = (extrudeTime * inv_factor) + travelTime;
    }
}
TimeMaterialEstimates ExtruderPlan::computeNaiveTimeEstimates()
{
    TimeMaterialEstimates ret;
    Point p0 = start_position;

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
            double thisTime = length / path.config->getSpeed();
            *path_time_estimate += thisTime;
            p0 = p1;
        }
        estimates += path.estimates;
    }
    return estimates;
}

void ExtruderPlan::processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time)
{
    FanSpeedLayerTimeSettings& fsml = fan_speed_layer_time_settings;
    TimeMaterialEstimates estimates = computeNaiveTimeEstimates();
    totalPrintTime = estimates.getTotalTime();
    if (force_minimal_layer_time)
    {
        forceMinimalLayerTime(fsml.cool_min_layer_time, fsml.cool_min_speed, estimates.getTravelTime(), estimates.getExtrudeTime());
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
    fan_speed = fsml.cool_fan_speed_min;
    double totalLayerTime = estimates.unretracted_travel_time + estimates.extrude_time;
    if (force_minimal_layer_time && totalLayerTime < fsml.cool_min_layer_time)
    {
        fan_speed = fsml.cool_fan_speed_max;
    }
    else if (force_minimal_layer_time && totalLayerTime < fsml.cool_min_layer_time_fan_speed_max)
    { 
        // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
        double fan_speed_diff = fsml.cool_fan_speed_max - fsml.cool_fan_speed_min;
        double layer_time_diff = fsml.cool_min_layer_time_fan_speed_max - fsml.cool_min_layer_time;
        double fraction_of_slope = (totalLayerTime - fsml.cool_min_layer_time) / layer_time_diff;
        fan_speed = fsml.cool_fan_speed_max - fan_speed_diff * fraction_of_slope;
    }
    /*
    Supposing no influence of minimal layer time; i.e. layer time > min layer time fan speed min:

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
    if (layer_nr < fsml.cool_fan_full_layer)
    {
        //Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
        fan_speed = fsml.cool_fan_speed_0 + (fan_speed - fsml.cool_fan_speed_0) * std::max(0, layer_nr) / fsml.cool_fan_full_layer;
    }
}

TimeMaterialEstimates GCodePlanner::computeNaiveTimeEstimates()
{
    TimeMaterialEstimates ret;
    for (ExtruderPlan& extruder_plan : extruder_plans)
    {
        ret += extruder_plan.computeNaiveTimeEstimates();
    }
    return ret;
}

void GCodePlanner::processFanSpeedAndMinimalLayerTime()
{
    for (unsigned int extr_plan_idx = 0; extr_plan_idx < extruder_plans.size(); extr_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extr_plan_idx];
        bool force_minimal_layer_time = extr_plan_idx == extruder_plans.size() - 1;
        extruder_plan.processFanSpeedAndMinimalLayerTime(force_minimal_layer_time);
    }
}



void GCodePlanner::writeGCode(GCodeExport& gcode)
{
    completeConfigs();
    
    CommandSocket::setLayerForSend(layer_nr);
    CommandSocket::setSendCurrentPosition( gcode.getPositionXY() );
    gcode.setLayerNr(layer_nr);
    
    gcode.writeLayerComment(layer_nr);

    if (layer_nr == 1 - Raft::getTotalExtraLayers(storage))
    {
        bool wait = false;
        gcode.writeBedTemperatureCommand(storage.getSettingInDegreeCelsius("material_bed_temperature"), wait);
    }

    gcode.setZ(z);
    
    
    GCodePathConfig* last_extrusion_config = nullptr; // used to check whether we need to insert a TYPE comment in the gcode.

    int extruder = gcode.getExtruderNr();
    bool acceleration_enabled = storage.getSettingBoolean("acceleration_enabled");
    bool jerk_enabled = storage.getSettingBoolean("jerk_enabled");

    for(unsigned int extruder_plan_idx = 0; extruder_plan_idx < extruder_plans.size(); extruder_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
        RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_plan.extruder];

        if (extruder != extruder_plan.extruder)
        {
            int prev_extruder = extruder;
            extruder = extruder_plan.extruder;
            gcode.switchExtruder(extruder, storage.extruder_switch_retraction_config_per_extruder[prev_extruder]);

            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder);
            if (train->getSettingInMillimetersPerSecond("max_feedrate_z_override") > 0)
            {
                gcode.writeMaxZFeedrate(train->getSettingInMillimetersPerSecond("max_feedrate_z_override"));
            }

            { // require printing temperature to be met
                constexpr bool wait = true;
                gcode.writeTemperatureCommand(extruder, extruder_plan.initial_printing_temperature, wait);
            }

            // prime extruder if it hadn't been used yet
            gcode.writePrimeTrain(storage.meshgroup->getExtruderTrain(extruder)->getSettingInMillimetersPerSecond("speed_travel"));
            gcode.writeRetraction(retraction_config);

            if (extruder_plan.prev_extruder_standby_temp)
            { // turn off previous extruder
                constexpr bool wait = false;
                double prev_extruder_temp = *extruder_plan.prev_extruder_standby_temp;
                int prev_layer_nr = (extruder_plan_idx == 0)? layer_nr - 1 : layer_nr;
                if (prev_layer_nr == storage.max_print_height_per_extruder[prev_extruder])
                {
                    prev_extruder_temp = 0; // TODO ? should there be a setting for extruder_off_temperature ?
                }
                gcode.writeTemperatureCommand(prev_extruder, prev_extruder_temp, wait);
            }
        }
        else if (extruder_plan_idx == 0 && layer_nr != 0 && storage.meshgroup->getExtruderTrain(extruder)->getSettingBoolean("retract_at_layer_change"))
        {
            gcode.writeRetraction(retraction_config);
        }
        gcode.writeFanCommand(extruder_plan.getFanSpeed());
        std::vector<GCodePath>& paths = extruder_plan.paths;

        extruder_plan.inserts.sort([](const NozzleTempInsert& a, const NozzleTempInsert& b) -> bool { 
                return  a.path_idx < b.path_idx; 
            } );

        const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder);
        if (train->getSettingInMillimetersPerSecond("max_feedrate_z_override") > 0)
        {
            gcode.writeMaxZFeedrate(train->getSettingInMillimetersPerSecond("max_feedrate_z_override"));
        }
        bool speed_equalize_flow_enabled = train->getSettingBoolean("speed_equalize_flow_enabled");
        double speed_equalize_flow_max = train->getSettingInMillimetersPerSecond("speed_equalize_flow_max");
        int64_t nozzle_size = gcode.getNozzleSize(extruder);

        for(unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
        {
            extruder_plan.handleInserts(path_idx, gcode);
            
            GCodePath& path = paths[path_idx];

            if (path.perform_prime)
            {
                gcode.writePrimeTrain(train->getSettingInMillimetersPerSecond("speed_travel"));
                gcode.writeRetraction(retraction_config);
            }

            if (acceleration_enabled)
            {
                gcode.writeAcceleration(path.config->getAcceleration());
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
                    gcode.writeZhopStart(retraction_config.zHop);
                }
                else
                {
                    gcode.writeZhopEnd();
                }
            }
            if (!path.config->isTravelPath() && last_extrusion_config != path.config)
            {
                gcode.writeTypeComment(path.config->type);
                last_extrusion_config = path.config;
            }

            double speed = path.config->getSpeed();

            // Apply the relevant factor
            if (path.config->isTravelPath())
                speed *= extruder_plan.getTravelSpeedFactor();
            else
                speed *= extruder_plan.getExtrudeSpeedFactor();

            if (MergeInfillLines(gcode, layer_nr, paths, extruder_plan, storage.travel_config_per_extruder[extruder], nozzle_size, speed_equalize_flow_enabled, speed_equalize_flow_max).mergeInfillLines(path_idx)) // !! has effect on path_idx !!
            { // !! has effect on path_idx !!
                // works when path_idx is the index of the travel move BEFORE the infill lines to be merged
                continue;
            }

            if (path.config->isTravelPath())
            { // early comp for travel paths, which are handled more simply
                for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                {
                    gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
                    if (point_idx == path.points.size() - 1)
                    {
                        gcode.setZ(z); // go down to extrusion level when we spiralized before on this layer
                        gcode.writeMove(gcode.getPositionXY(), speed, path.getExtrusionMM3perMM());
                    }
                }
                continue;
            }
            
            bool spiralize = path.spiralize;
            if (!spiralize) // normal (extrusion) move (with coasting
            {
                CoastingConfig& coasting_config = storage.coasting_config[extruder];
                bool coasting = coasting_config.coasting_enable; 
                if (coasting)
                {
                    coasting = writePathWithCoasting(gcode, extruder_plan_idx, path_idx, layer_thickness, coasting_config.coasting_volume, coasting_config.coasting_speed, coasting_config.coasting_min_volume);
                }
                if (! coasting) // not same as 'else', cause we might have changed [coasting] in the line above...
                { // normal path to gcode algorithm
                    if (  // change infill  ||||||   to  /\/\/\/\/ ...
                        false &&
                        path_idx + 2 < paths.size() // has a next move
                        && paths[path_idx+1].points.size() == 1 // is single extruded line
                        && !paths[path_idx+1].config->isTravelPath() // next move is extrusion
                        && paths[path_idx+2].config->isTravelPath() // next next move is travel
                        && shorterThen(path.points.back() - gcode.getPositionXY(), 2 * nozzle_size) // preceding extrusion is close by
                        && shorterThen(paths[path_idx+1].points.back() - path.points.back(), 2 * nozzle_size) // extrusion move is small
                        && shorterThen(paths[path_idx+2].points.back() - paths[path_idx+1].points.back(), 2 * nozzle_size) // consecutive extrusion is close by
                    )
                    {
                        sendLineTo(paths[path_idx+2].config->type, paths[path_idx+2].points.back(), paths[path_idx+2].getLineWidth());
                        gcode.writeMove(paths[path_idx+2].points.back(), speed, paths[path_idx+1].getExtrusionMM3perMM());
                        path_idx += 2;
                    }
                    else 
                    {
                        for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                        {
                            sendLineTo(path.config->type, path.points[point_idx], path.getLineWidth());
                            gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
                        }
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
                        Point p1 = path.points[point_idx];
                        length += vSizeMM(p0 - p1);
                        p0 = p1;
                        gcode.setZ(z + layer_thickness * length / totalLength);
                        sendLineTo(path.config->type, path.points[point_idx], path.getLineWidth());
                        gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
                    }
                }
                path_idx--; // the last path_idx didnt spiralize, so it's not part of the current spiralize path
            }
        } // paths for this extruder /\  .

        if (train->getSettingBoolean("cool_lift_head") && extruder_plan.extraTime > 0.0)
        {
            gcode.writeComment("Small layer, adding delay");
            RetractionConfig& retraction_config = storage.retraction_config_per_extruder[gcode.getExtruderNr()];
            gcode.writeRetraction(retraction_config);
            if (extruder_plan_idx == extruder_plans.size() - 1 || !train->getSettingBoolean("machine_extruder_end_pos_abs"))
            { // only move the head if it's the last extruder plan; otherwise it's already at the switching bay area 
                // or do it anyway when we switch extruder in-place
                gcode.setZ(gcode.getPositionZ() + MM2INT(3.0));
                gcode.writeMove(gcode.getPositionXY(), storage.travel_config_per_extruder[extruder].getSpeed(), 0);
                // TODO: is this safe?! wouldn't the head move into the sides then?!
                gcode.writeMove(gcode.getPositionXY() - Point(-MM2INT(20.0), 0), storage.travel_config_per_extruder[extruder].getSpeed(), 0);
            }
            gcode.writeDelay(extruder_plan.extraTime);
        }

        extruder_plan.handleAllRemainingInserts(gcode);
    } // extruder plans /\  .
    
    gcode.updateTotalPrintTime();
}

void GCodePlanner::overrideFanSpeeds(double speed)
{
    for (ExtruderPlan& extruder_plan : extruder_plans)
    {
        extruder_plan.setFanSpeed(speed);
    }
}


void GCodePlanner::completeConfigs()
{
    storage.support_config.setLayerHeight(layer_thickness);
    storage.support_skin_config.setLayerHeight(layer_thickness);
    
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.inset0_config.setLayerHeight(layer_thickness);

        mesh.insetX_config.setLayerHeight(layer_thickness);
        mesh.skin_config.setLayerHeight(layer_thickness);
        mesh.perimeter_gap_config.setLayerHeight(layer_thickness);
        for(unsigned int idx=0; idx<MAX_INFILL_COMBINE; idx++)
        {
            mesh.infill_config[idx].setLayerHeight(layer_thickness);
        }
    }
    
    storage.primeTower.setConfigs(storage.meshgroup, layer_thickness);
    
    processInitialLayersSpeedup();
}


void GCodePlanner::processInitialLayersSpeedup()
{
    int initial_speedup_layers = storage.getSettingAsCount("speed_slowdown_layers");
    if (layer_nr >= 0 && layer_nr < initial_speedup_layers)
    {
        GCodePathConfig::BasicConfig initial_layer_speed_config;
        int extruder_nr_support_infill = storage.getSettingAsIndex((layer_nr == 0)? "support_extruder_nr_layer_0" : "support_infill_extruder_nr");
        initial_layer_speed_config.speed = storage.meshgroup->getExtruderTrain(extruder_nr_support_infill)->getSettingInMillimetersPerSecond("speed_print_layer_0");
        initial_layer_speed_config.acceleration = storage.meshgroup->getExtruderTrain(extruder_nr_support_infill)->getSettingInMillimetersPerSecond("acceleration_print_layer_0");
        initial_layer_speed_config.jerk = storage.meshgroup->getExtruderTrain(extruder_nr_support_infill)->getSettingInMillimetersPerSecond("jerk_print_layer_0");

        //Support (global).
        storage.support_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);

        //Support roof (global).
        int extruder_nr_support_skin = storage.getSettingAsIndex("support_interface_extruder_nr");
        initial_layer_speed_config.speed = storage.meshgroup->getExtruderTrain(extruder_nr_support_skin)->getSettingInMillimetersPerSecond("speed_print_layer_0");
        initial_layer_speed_config.acceleration = storage.meshgroup->getExtruderTrain(extruder_nr_support_skin)->getSettingInMillimetersPerSecond("acceleration_print_layer_0");
        initial_layer_speed_config.jerk = storage.meshgroup->getExtruderTrain(extruder_nr_support_skin)->getSettingInMillimetersPerSecond("jerk_print_layer_0");
        storage.support_skin_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);

        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); ++extruder_nr)
        {
            const ExtruderTrain* extruder_train = storage.meshgroup->getExtruderTrain(extruder_nr);
            initial_layer_speed_config.speed = extruder_train->getSettingInMillimetersPerSecond("speed_travel_layer_0");
            initial_layer_speed_config.acceleration = extruder_train->getSettingInMillimetersPerSecond("acceleration_travel_layer_0");
            initial_layer_speed_config.jerk = extruder_train->getSettingInMillimetersPerSecond("jerk_travel_layer_0");

            //Travel speed (per extruder).
            storage.travel_config_per_extruder[extruder_nr].smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);
        }

        for (SliceMeshStorage& mesh : storage.meshes)
        {
            initial_layer_speed_config.speed = mesh.getSettingInMillimetersPerSecond("speed_print_layer_0");
            initial_layer_speed_config.acceleration = mesh.getSettingInMillimetersPerSecond("acceleration_print_layer_0");
            initial_layer_speed_config.jerk = mesh.getSettingInMillimetersPerSecond("jerk_print_layer_0");

            //Outer wall speed (per mesh).
            mesh.inset0_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);

            //Inner wall speed (per mesh).
            mesh.insetX_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);

            //Skin speed (per mesh).
            mesh.skin_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);
            mesh.perimeter_gap_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);

            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                //Infill speed (per combine part per mesh).
                mesh.infill_config[idx].smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layers);
            }
        }
    }
    else if (layer_nr == initial_speedup_layers) //At the topmost layer of the gradient, reset all speeds to the typical speeds.
    {
        storage.support_config.setSpeedIconic();
        storage.support_skin_config.setSpeedIconic();
        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); ++extruder_nr)
        {
            storage.travel_config_per_extruder[extruder_nr].setSpeedIconic();
        }
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            mesh.inset0_config.setSpeedIconic();
            mesh.insetX_config.setSpeedIconic();
            mesh.skin_config.setSpeedIconic();
            mesh.perimeter_gap_config.setSpeedIconic();
            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                mesh.infill_config[idx].setSpeedIconic();
            }
        }
    }
}



bool GCodePlanner::makeRetractSwitchRetract(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx)
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
        
    if (extruder_plans[extruder_plan_idx + 1].extruder != extruder_plans[extruder_plan_idx].extruder)
    {
        return true;
    }
    else 
    {
        return false;
    }
}
    
bool GCodePlanner::writePathWithCoasting(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx, int64_t layerThickness, double coasting_volume, double coasting_speed, double coasting_min_volume)
{
    if (coasting_volume <= 0) 
    { 
        return false; 
    }
    ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
    std::vector<GCodePath>& paths = extruder_plan.paths;
    GCodePath& path = paths[path_idx];
    if (path_idx + 1 >= paths.size()
        ||
        ! (!path.isTravelPath() &&  paths[path_idx + 1].config->isTravelPath()) 
        ||
        path.points.size() < 2
        )
    {
        return false;
    }

    int64_t coasting_min_dist_considered = 100; // hardcoded setting for when to not perform coasting

    
    double extrude_speed = path.config->getSpeed() * extruder_plan.getExtrudeSpeedFactor(); // travel speed 
    
    int64_t coasting_dist = MM2INT(MM2_2INT(coasting_volume) / layerThickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    int64_t coasting_min_dist = MM2INT(MM2_2INT(coasting_min_volume + coasting_volume) / layerThickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    //           /\ the minimal distance when coasting will coast the full coasting volume instead of linearly less with linearly smaller paths
    
    
    std::vector<int64_t> accumulated_dist_per_point; // the first accumulated dist is that of the last point! (that of the last point is always zero...)
    accumulated_dist_per_point.push_back(0);
    
    int64_t accumulated_dist = 0;
    
    bool length_is_less_than_min_dist = true;
    
    unsigned int acc_dist_idx_gt_coast_dist = NO_INDEX; // the index of the first point with accumulated_dist more than coasting_dist (= index into accumulated_dist_per_point)
     // == the point printed BEFORE the start point for coasting
    
    
    Point* last = &path.points[path.points.size() - 1];
    for (unsigned int backward_point_idx = 1; backward_point_idx < path.points.size(); backward_point_idx++)
    {
        Point& point = path.points[path.points.size() - 1 - backward_point_idx];
        int64_t dist = vSize(point - *last);
        accumulated_dist += dist;
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
    int64_t actual_coasting_dist = coasting_dist;
    if (length_is_less_than_min_dist)
    {
        // in this case accumulated_dist is the length of the whole path
        actual_coasting_dist = accumulated_dist * coasting_dist / coasting_min_dist;
        for (acc_dist_idx_gt_coast_dist = 0 ; acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size() ; acc_dist_idx_gt_coast_dist++)
        { // search for the correct coast_dist_idx
            if (accumulated_dist_per_point[acc_dist_idx_gt_coast_dist] > actual_coasting_dist)
            {
                break;
            }
        }
    }

    assert (acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size()); // something has gone wrong; coasting_min_dist < coasting_dist ?

    unsigned int point_idx_before_start = path.points.size() - 1 - acc_dist_idx_gt_coast_dist;

    Point start;
    { // computation of begin point of coasting
        int64_t residual_dist = actual_coasting_dist - accumulated_dist_per_point[acc_dist_idx_gt_coast_dist - 1];
        Point& a = path.points[point_idx_before_start];
        Point& b = path.points[point_idx_before_start + 1];
        start = b + normal(a-b, residual_dist);
    }

    { // write normal extrude path:
        for(unsigned int point_idx = 0; point_idx <= point_idx_before_start; point_idx++)
        {
            sendLineTo(path.config->type, path.points[point_idx], path.getLineWidth());
            gcode.writeMove(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM());
        }
        sendLineTo(path.config->type, start, path.getLineWidth());
        gcode.writeMove(start, extrude_speed, path.getExtrusionMM3perMM());
    }

    // write coasting path
    for (unsigned int point_idx = point_idx_before_start + 1; point_idx < path.points.size(); point_idx++)
    {
        gcode.writeMove(path.points[point_idx], coasting_speed * path.config->getSpeed(), 0);
    }

    gcode.addLastCoastedVolume(path.getExtrusionMM3perMM() * INT2MM(actual_coasting_dist));
    return true;
}

}//namespace cura
