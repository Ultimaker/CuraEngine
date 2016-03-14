#include <cstring>
#include "gcodePlanner.h"
#include "pathOrderOptimizer.h"
#include "sliceDataStorage.h"
#include "debug.h" // debugging
#include "utils/polygonUtils.h"
#include "MergeInfillLines.h"

namespace cura {

TimeMaterialEstimates TimeMaterialEstimates::operator-(const TimeMaterialEstimates& other)
{
    return TimeMaterialEstimates(extrude_time - other.extrude_time,unretracted_travel_time - other.unretracted_travel_time,retracted_travel_time - other.retracted_travel_time,material - other.material);
}

TimeMaterialEstimates& TimeMaterialEstimates::operator-=(const TimeMaterialEstimates& other)
{
    extrude_time -= other.extrude_time;
    unretracted_travel_time -= other.unretracted_travel_time;
    retracted_travel_time -= other.retracted_travel_time;
    material -= other.material;
    return *this;
}

GCodePath* GCodePlanner::getLatestPathWithConfig(GCodePathConfig* config, SpaceFillType space_fill_type, float flow)
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0 && paths.back().config == config && !paths.back().done && paths.back().flow == flow)
        return &paths.back();
    paths.emplace_back();
    GCodePath* ret = &paths.back();
    ret->retract = false;
    ret->config = config;
    ret->done = false;
    ret->flow = flow;
    ret->space_fill_type = space_fill_type;
    if (config != &storage.travel_config)
    {
        last_retraction_config = config->retraction_config;
    }
    return ret;
}

void GCodePlanner::forceNewPathStart()
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0)
        paths[paths.size()-1].done = true;
}

GCodePlanner::GCodePlanner(SliceDataStorage& storage, unsigned int layer_nr, int z, int layer_thickness, Point last_position, int current_extruder, FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, bool retraction_combing, int64_t comb_boundary_offset, bool travel_avoid_other_parts, int64_t travel_avoid_distance)
: storage(storage)
, layer_nr(layer_nr)
, z(z)
, layer_thickness(layer_thickness)
, start_position(last_position)
, lastPosition(last_position)
, comb_boundary_inside(computeCombBoundaryInside())
, fan_speed_layer_time_settings(fan_speed_layer_time_settings)
{
    extruder_plans.reserve(storage.meshgroup->getExtruderCount());
    extruder_plans.emplace_back(current_extruder);
    comb = nullptr;
    was_inside = true; // means it will try to get inside the comb boundary first
    is_inside = true; // means it will try to get inside the comb boundary 
    last_retraction_config = &storage.retraction_config; // start with general config
    setExtrudeSpeedFactor(1.0);
    setTravelSpeedFactor(1.0);
    extraTime = 0.0;
    totalPrintTime = 0.0;
    if (retraction_combing)
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

Polygons GCodePlanner::computeCombBoundaryInside()
{
    if (layer_nr < 0)
    { // when a raft is present
        return storage.raftOutline.offset(MM2INT(0.1));
    }
    else 
    {
        Polygons layer_walls;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[layer_nr];
            layer.getSecondOrInnermostWalls(layer_walls);
        }
        return layer_walls;
    }
}

void GCodePlanner::setIsInside(bool _is_inside)
{
    is_inside = _is_inside;
}

bool GCodePlanner::setExtruder(int extruder)
{
    if (extruder == extruder_plans.back().extruder)
    {
        return false;
    }
    { // handle end position of the prev extruder
        SettingsBase* train = storage.meshgroup->getExtruderTrain(extruder_plans.back().extruder);
        bool end_pos_absolute = train->getSettingBoolean("machine_extruder_end_pos_abs");
        Point extruder_offset(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
        Point end_pos(train->getSettingInMicrons("machine_extruder_end_pos_x"), train->getSettingInMicrons("machine_extruder_end_pos_y"));
        if (!end_pos_absolute)
        {
            end_pos += lastPosition;
        }
        else 
        {
            end_pos += extruder_offset; // absolute end pos is given as a head position
        }
        addTravel(end_pos); //  + extruder_offset cause it 
    }
    extruder_plans.emplace_back(extruder);

//     forceNewPathStart(); // automatic by the fact that we start a new ExtruderPlan

    { // handle starting pos of the new extruder
        SettingsBase* train = storage.meshgroup->getExtruderTrain(extruder);
        bool start_pos_absolute = train->getSettingBoolean("machine_extruder_start_pos_abs");
        Point extruder_offset(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
        Point start_pos(train->getSettingInMicrons("machine_extruder_start_pos_x"), train->getSettingInMicrons("machine_extruder_start_pos_y"));
        if (!start_pos_absolute)
        {
            start_pos += lastPosition;
        }
        else 
        {
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

void GCodePlanner::addTravel(Point p)
{
    GCodePath* path = nullptr;
    
    bool combed = false;
    
    if (comb != nullptr && lastPosition != no_point)
    {
        CombPaths combPaths;
        combed = comb->calc(lastPosition, p, combPaths, was_inside, is_inside, last_retraction_config->retraction_min_travel_distance);
        if (combed)
        {
            bool retract = combPaths.size() > 1;
            if (!retract)
            { // check whether we want to retract
                for (CombPath& combPath : combPaths)
                { // retract when path moves through a boundary
                    if (combPath.cross_boundary || combPath.throughAir)
                    {
                        retract = true;
                        break;
                    }
                }
                if (combPaths.size() == 1)
                {
                    CombPath path = combPaths[0];
                    if (path.throughAir && !path.cross_boundary && path.size() == 2 && path[0] == lastPosition && path[1] == p)
                    { // limit the retractions from support to support, which didn't cross anything
                        retract = false;
                    }
                }
            }
            
            if (retract && last_retraction_config->zHop > 0)
            { // TODO: stop comb calculation early! (as soon as we see we don't end in the same part as we began)
                path = getLatestPathWithConfig(&storage.travel_config, SpaceFillType::None);
                if (!shorterThen(lastPosition - p, last_retraction_config->retraction_min_travel_distance))
                {
                    path->retract = true;
                }
            }
            else 
            {
                for (CombPath& combPath : combPaths)
                { // add all comb paths (don't do anything special for paths which are moving through air)
                    if (combPath.size() == 0)
                    {
                        continue;
                    }
                    path = getLatestPathWithConfig(&storage.travel_config, SpaceFillType::None);
                    path->retract = retract;
                    for (Point& combPoint : combPath)
                    {
                        path->points.push_back(combPoint);
                    }
                    lastPosition = combPath.back();
                }
            }
        }
    }
    
    if (!combed) {
        // no combing? always retract!
        if (!shorterThen(lastPosition - p, last_retraction_config->retraction_min_travel_distance))
        {
            if (was_inside) // when the previous location was from printing something which is considered inside (not support or prime tower etc)
            {               // then move inside the printed part, so that we don't ooze on the outer wall while retraction, but on the inside of the print.
                ExtruderTrain* extr = storage.meshgroup->getExtruderTrain(getExtruder());
                assert (extr != nullptr);
                moveInsideCombBoundary(extr->getSettingInMicrons((extr->getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0") * 1);
            }
            path = getLatestPathWithConfig(&storage.travel_config, SpaceFillType::None);
            path->retract = true;
        }
    }

    addTravel_simple(p, path);
    was_inside = is_inside;
}

void GCodePlanner::addTravel_simple(Point p, GCodePath* path)
{
    if (path == nullptr)
    {
        path = getLatestPathWithConfig(&storage.travel_config, SpaceFillType::None);
    }
    path->points.push_back(p);
    lastPosition = p;
}


void GCodePlanner::addExtrusionMove(Point p, GCodePathConfig* config, SpaceFillType space_fill_type, float flow)
{
    getLatestPathWithConfig(config, space_fill_type, flow)->points.push_back(p);
    lastPosition = p;
}

void GCodePlanner::addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation)
{
    Point p0 = polygon[startIdx];
    addTravel(p0);
    for(unsigned int i=1; i<polygon.size(); i++)
    {
        Point p1 = polygon[(startIdx + i) % polygon.size()];
        addExtrusionMove(p1, config, SpaceFillType::Polygons, (wall_overlap_computation)? wall_overlap_computation->getFlow(p0, p1) : 1.0);
        p0 = p1;
    }
    if (polygon.size() > 2)
    {
        Point& p1 = polygon[startIdx];
        addExtrusionMove(p1, config, SpaceFillType::Polygons, (wall_overlap_computation)? wall_overlap_computation->getFlow(p0, p1) : 1.0);
    }
    else 
    {
        logWarning("WARNING: line added as polygon! (gcodePlanner)\n");
    }
}

void GCodePlanner::addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation, EZSeamType z_seam_type)
{
    PathOrderOptimizer orderOptimizer(lastPosition, z_seam_type);
    for(unsigned int i=0;i<polygons.size();i++)
        orderOptimizer.addPolygon(polygons[i]);
    orderOptimizer.optimize();
    for(unsigned int i=0;i<orderOptimizer.polyOrder.size();i++)
    {
        int nr = orderOptimizer.polyOrder[i];
        addPolygon(polygons[nr], orderOptimizer.polyStart[nr], config, wall_overlap_computation);
    }
}
void GCodePlanner::addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config, SpaceFillType space_fill_type, int wipe_dist)
{
    LineOrderOptimizer orderOptimizer(lastPosition);
    for(unsigned int i=0;i<polygons.size();i++)
        orderOptimizer.addPolygon(polygons[i]);
    orderOptimizer.optimize();
    for(unsigned int i=0;i<orderOptimizer.polyOrder.size();i++)
    {
        int nr = orderOptimizer.polyOrder[i];
//         addPolygon(polygons[nr], orderOptimizer.polyStart[nr], config);
        PolygonRef polygon = polygons[nr];
        int start = orderOptimizer.polyStart[nr];
        int end = 1 - start;
        Point& p0 = polygon[start];
        addTravel(p0);
        Point& p1 = polygon[end];
        addExtrusionMove(p1, config, space_fill_type);
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

void GCodePlanner::forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrudeTime)
{
    double totalTime = travelTime + extrudeTime; 
    if (totalTime < minTime && extrudeTime > 0.0)
    {
        double minExtrudeTime = minTime - travelTime;
        if (minExtrudeTime < 1)
            minExtrudeTime = 1;
        double factor = extrudeTime / minExtrudeTime;
        for(ExtruderPlan& extr_plan : extruder_plans)
        {
            for (GCodePath& path : extr_plan.paths)
            {
                if (path.isTravelPath())
                    continue;
                double speed = path.config->getSpeed() * factor;
                if (speed < minimalSpeed)
                    factor = minimalSpeed / path.config->getSpeed();
            }
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
        for(ExtruderPlan& extr_plan : extruder_plans)
        {
            extr_plan.estimates.extrude_time *= inv_factor;
            for (GCodePath& path : extr_plan.paths)
            {
                path.estimates.extrude_time *= inv_factor;
            }
        }

        if (minTime - (extrudeTime * inv_factor) - travelTime > 0.1)
        {
            this->extraTime = minTime - (extrudeTime * inv_factor) - travelTime;
        }
        this->totalPrintTime = (extrudeTime * inv_factor) + travelTime;
    }else{
        this->totalPrintTime = totalTime;
    }
}

TimeMaterialEstimates GCodePlanner::computeNaiveTimeEstimates()
{
    TimeMaterialEstimates ret;
    Point p0 = start_position;

    bool was_retracted = false; // wrong assumption; won't matter that much. (TODO)
    for(ExtruderPlan& extr_plan : extruder_plans)
    {
        for (GCodePath& path : extr_plan.paths)
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
                    RetractionConfig& retraction_config = *path.config->retraction_config;
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
            extr_plan.estimates += path.estimates;
        }
        ret += extr_plan.estimates;
    }
    return ret;
}

void GCodePlanner::processFanSpeedAndMinimalLayerTime()
{
    FanSpeedLayerTimeSettings& fsml = fan_speed_layer_time_settings;
    TimeMaterialEstimates estimates = computeNaiveTimeEstimates();
    forceMinimalLayerTime(fsml.cool_min_layer_time, fsml.cool_min_speed, estimates.getTravelTime(), estimates.getExtrudeTime());

    // interpolate fan speed (for cool_fan_full_layer and for cool_min_layer_time_fan_speed_max)
    fan_speed = fsml.cool_fan_speed_min;
    double totalLayerTime = estimates.unretracted_travel_time + estimates.extrude_time;
    if (totalLayerTime < fsml.cool_min_layer_time)
    {
        fan_speed = fsml.cool_fan_speed_max;
    }
    else if (totalLayerTime < fsml.cool_min_layer_time_fan_speed_max)
    { 
        // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
        fan_speed = fsml.cool_fan_speed_max - (fsml.cool_fan_speed_max-fsml.cool_fan_speed_min) * (totalLayerTime - fsml.cool_min_layer_time) / (fsml.cool_min_layer_time_fan_speed_max - fsml.cool_min_layer_time);
    }
    if (layer_nr < fsml.cool_fan_full_layer)
    {
        //Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
        fan_speed = fan_speed * layer_nr / fsml.cool_fan_full_layer;
    }
}


void GCodePlanner::writeGCode(GCodeExport& gcode, bool liftHeadIfNeeded, int layerThickness)
{
    completeConfigs();
    
    gcode.setLayerNr(layer_nr);
    
    gcode.writeLayerComment(layer_nr);
    
    gcode.setZ(z);
    
    gcode.writeFanCommand(fan_speed);
    
    GCodePathConfig* last_extrusion_config = nullptr;
    int extruder = gcode.getExtruderNr();

    for(unsigned int extruder_plan_idx = 0; extruder_plan_idx < extruder_plans.size(); extruder_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
        if (extruder != extruder_plan.extruder)
        {
            extruder = extruder_plan.extruder;
            gcode.switchExtruder(extruder);
        }
        std::vector<GCodePath>& paths = extruder_plan.paths;
        
        extruder_plan.inserts.sort([](const NozzleTempInsert& a, const NozzleTempInsert& b) -> bool { 
                return  a.path_idx < b.path_idx; 
            } );
        
        for(unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
        {
            extruder_plan.handleInserts(path_idx, gcode);
            
            GCodePath& path = paths[path_idx];
            if (path.retract)
            {
                writeRetraction(gcode, extruder_plan_idx, path_idx);
            }
            if (path.config != &storage.travel_config && last_extrusion_config != path.config)
            {
                gcode.writeTypeComment(path.config->type);
                last_extrusion_config = path.config;
            }
            double speed = path.config->getSpeed();

            if (path.isTravelPath())// Only apply the extrudeSpeed to extrusion moves
                speed *= getTravelSpeedFactor();
            else
                speed *= getExtrudeSpeedFactor();
            
            int64_t nozzle_size = 400; // TODO
            
            if (MergeInfillLines(gcode, layer_nr, paths, extruder_plan, storage.travel_config, nozzle_size).mergeInfillLines(speed, path_idx)) // !! has effect on path_idx !!
            { // !! has effect on path_idx !!
                // works when path_idx is the index of the travel move BEFORE the infill lines to be merged
                continue;
            }
            
            if (path.config == &storage.travel_config)
            { // early comp for travel paths, which are handled more simply
                for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                {
                    gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
                }
                continue;
            }
            
            bool spiralize = path.config->spiralize;
            if (spiralize)
            {
                //Check if we are the last spiralize path in the list, if not, do not spiralize.
                for(unsigned int m=path_idx+1; m<paths.size(); m++)
                {
                    if (paths[m].config->spiralize)
                        spiralize = false;
                }
            }
            if (!spiralize) // normal (extrusion) move (with coasting
            { 
                CoastingConfig& coasting_config = storage.coasting_config[extruder];
                bool coasting = coasting_config.coasting_enable; 
                if (coasting)
                {
                    coasting = writePathWithCoasting(gcode, extruder_plan_idx, path_idx, layerThickness, coasting_config.coasting_volume, coasting_config.coasting_speed, coasting_config.coasting_min_volume);
                }
                if (! coasting) // not same as 'else', cause we might have changed [coasting] in the line above...
                { // normal path to gcode algorithm
                    if (  // change infill  ||||||   to  /\/\/\/\/ ...
                        false &&
                        path_idx + 2 < paths.size() // has a next move
                        && paths[path_idx+1].points.size() == 1 // is single extruded line
                        && paths[path_idx+1].config != &storage.travel_config // next move is extrusion
                        && paths[path_idx+2].config == &storage.travel_config // next next move is travel
                        && shorterThen(path.points.back() - gcode.getPositionXY(), 2 * nozzle_size) // preceding extrusion is close by
                        && shorterThen(paths[path_idx+1].points.back() - path.points.back(), 2 * nozzle_size) // extrusion move is small
                        && shorterThen(paths[path_idx+2].points.back() - paths[path_idx+1].points.back(), 2 * nozzle_size) // consecutive extrusion is close by
                    )
                    {
                        sendPolygon(paths[path_idx+2].config->type, gcode.getPositionXY(), paths[path_idx+2].points.back(), paths[path_idx+2].getLineWidth());
                        gcode.writeMove(paths[path_idx+2].points.back(), speed, paths[path_idx+1].getExtrusionMM3perMM());
                        path_idx += 2;
                    }
                    else 
                    {
                        for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                        {
                            sendPolygon(path.config->type, gcode.getPositionXY(), path.points[point_idx], path.getLineWidth());
                            gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
                        }
                    }
                }
            }
            else
            { // SPIRALIZE
                //If we need to spiralize then raise the head slowly by 1 layer as this path progresses.
                float totalLength = 0.0;
                int z = gcode.getPositionZ();
                Point p0 = gcode.getPositionXY();
                for(unsigned int i=0; i<path.points.size(); i++)
                {
                    Point p1 = path.points[i];
                    totalLength += vSizeMM(p0 - p1);
                    p0 = p1;
                }

                float length = 0.0;
                p0 = gcode.getPositionXY();
                for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                {
                    Point p1 = path.points[point_idx];
                    length += vSizeMM(p0 - p1);
                    p0 = p1;
                    gcode.setZ(z + layerThickness * length / totalLength);
                    sendPolygon(path.config->type, gcode.getPositionXY(), path.points[point_idx], path.getLineWidth());
                    gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
                }
            }
        }
    
        extruder_plan.handleAllRemainingInserts(gcode);
    }
    
    gcode.updateTotalPrintTime();
    if (liftHeadIfNeeded && extraTime > 0.0)
    {
        gcode.writeComment("Small layer, adding delay");
        if (last_extrusion_config)
        {
            bool extruder_switch_retract = false;// TODO: check whether we should do a retractoin_extruderSwitch; is the next path with a different extruder?
            writeRetraction(gcode, extruder_switch_retract, last_extrusion_config->retraction_config);
        }
        gcode.setZ(gcode.getPositionZ() + MM2INT(3.0));
        gcode.writeMove(gcode.getPositionXY(), storage.travel_config.getSpeed(), 0);
        gcode.writeMove(gcode.getPositionXY() - Point(-MM2INT(20.0), 0), storage.travel_config.getSpeed(), 0); // TODO: is this safe?! wouldn't the head move into the sides then?!
        gcode.writeDelay(extraTime);
    }
}

void GCodePlanner::completeConfigs()
{
    storage.support_config.setLayerHeight(layer_thickness);
    storage.support_roof_config.setLayerHeight(layer_thickness);
    
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.inset0_config.setLayerHeight(layer_thickness);

        mesh.insetX_config.setLayerHeight(layer_thickness);
        mesh.skin_config.setLayerHeight(layer_thickness);
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
    double initial_speedup_layers = storage.getSettingAsCount("speed_slowdown_layers");
    if (static_cast<int>(layer_nr) < initial_speedup_layers)
    {
        double initial_layer_speed = storage.getSettingInMillimetersPerSecond("speed_layer_0");
        storage.support_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
        storage.support_roof_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            initial_layer_speed = mesh.getSettingInMillimetersPerSecond("speed_layer_0");
            mesh.inset0_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            mesh.insetX_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            mesh.skin_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            for(unsigned int idx=0; idx<MAX_INFILL_COMBINE; idx++)
            {
                mesh.infill_config[idx].smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            }
        }
    }
}

void GCodePlanner::writeRetraction(GCodeExport& gcode, unsigned int extruder_plan_idx, unsigned int path_idx_travel_after)
{
    if (makeRetractSwitchRetract(gcode, extruder_plan_idx, path_idx_travel_after))
    {
        gcode.writeRetraction_extruderSwitch();
    }
    else 
    {
        std::vector<GCodePath>& paths = extruder_plans[extruder_plan_idx].paths;
        RetractionConfig* extrusion_retraction_config = nullptr;
        for(int extrusion_path_idx = int(path_idx_travel_after) - 1; extrusion_path_idx >= 0; extrusion_path_idx--)
        { // backtrack to find the last extrusion path
            if (paths[extrusion_path_idx].config != &storage.travel_config)
            {
                extrusion_retraction_config = paths[extrusion_path_idx].config->retraction_config;
                break;
            }
        }
        writeRetraction(gcode, false, extrusion_retraction_config);
    }
}
void GCodePlanner::writeRetraction(GCodeExport& gcode, bool extruder_switch_retract, RetractionConfig* retraction_config)
{    
    if (extruder_switch_retract)
    {
        gcode.writeRetraction_extruderSwitch();
    }
    else 
    {
        if (retraction_config)
        {
            gcode.writeRetraction(retraction_config);
        }
        else 
        {
            gcode.writeRetraction(storage.travel_config.retraction_config);
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
    std::vector<GCodePath>& paths = extruder_plans[extruder_plan_idx].paths;
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

    
    double extrude_speed = path.config->getSpeed() * getExtrudeSpeedFactor(); // travel speed 
    
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
            sendPolygon(path.config->type, gcode.getPositionXY(), path.points[point_idx], path.getLineWidth());
            gcode.writeMove(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM());
        }
        sendPolygon(path.config->type, gcode.getPositionXY(), start, path.getLineWidth());
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
