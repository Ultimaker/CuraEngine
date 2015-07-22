#include "gcodePlanner.h"
#include "pathOrderOptimizer.h"
#include "sliceDataStorage.h"
#include <cstring>
#include "debug.h" // debugging
#include "MergeInfillLines.h"

namespace cura {

GCodePath* GCodePlanner::getLatestPathWithConfig(GCodePathConfig* config, float flow)
{
    if (paths.size() > 0 && paths[paths.size()-1].config == config && !paths[paths.size()-1].done && paths[paths.size()-1].flow == flow)
        return &paths[paths.size()-1];
    paths.push_back(GCodePath());
    GCodePath* ret = &paths[paths.size()-1];
    ret->retract = false;
    ret->config = config;
    ret->extruder = currentExtruder;
    ret->done = false;
    ret->flow = flow;
    return ret;
}

void GCodePlanner::forceNewPathStart()
{
    if (paths.size() > 0)
        paths[paths.size()-1].done = true;
}

GCodePlanner::GCodePlanner(GCodeExport& gcode, SliceDataStorage& storage, RetractionConfig* retraction_config, CoastingConfig& coasting_config, double travelSpeed, int retractionMinimalDistance, bool retraction_combing, unsigned int layer_nr, int64_t wall_line_width_0, bool travel_avoid_other_parts, int64_t travel_avoid_distance)
: gcode(gcode), travelConfig(retraction_config, "MOVE"), coasting_config(coasting_config)
{
    lastPosition = gcode.getPositionXY();
    travelConfig.setSpeed(travelSpeed);
    comb = nullptr;
    setExtrudeSpeedFactor(1.0);
    setTravelSpeedFactor(1.0);
    extraTime = 0.0;
    totalPrintTime = 0.0;
    alwaysRetract = false;
    currentExtruder = gcode.getExtruderNr();
    if (retraction_combing)
        comb = new Comb(storage, layer_nr, wall_line_width_0, travel_avoid_other_parts, travel_avoid_distance);
    else
        comb = nullptr;
    this->retractionMinimalDistance = retractionMinimalDistance;
}

GCodePlanner::~GCodePlanner()
{
    if (comb)
        delete comb;
}

void GCodePlanner::addTravel(Point p)
{
    GCodePath* path = nullptr;

    if (comb != nullptr && lastPosition != Point(0,0))
    {
        CombPaths combPaths;
        if (comb->calc(lastPosition, p, combPaths))
        {
            bool retract = combPaths.size() > 1;
            { // check whether we want to retract
                if (!retract && combPaths.size() == 1 && combPaths[0].throughAir && combPaths[0].size() > 2)
                { // retract when avoiding obstacles through air
                    retract = true;
                }
                
                for (unsigned int path_idx = 0; path_idx < combPaths.size() && !retract; path_idx++)
                { // retract when path moves through a boundary
                    if (combPaths[path_idx].cross_boundary) { retract = true; }
                }
            }
            
            if (retract && travelConfig.retraction_config->zHop > 0)
            { // TODO: stop comb calculation early! (as soon as we see we don't end in the same part as we began)
                path = getLatestPathWithConfig(&travelConfig);
                if (!shorterThen(lastPosition - p, retractionMinimalDistance))
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
                    path = getLatestPathWithConfig(&travelConfig);
                    path->retract = retract;
                    for (Point& combPoint : combPath)
                    {
                        path->points.push_back(combPoint);
                    }
                    lastPosition = combPath.back();
                }
            }
        }
        else
        {
            path = getLatestPathWithConfig(&travelConfig);
            if (!shorterThen(lastPosition - p, retractionMinimalDistance))
            {
                path->retract = true;
            }
        }
    }
    else if (alwaysRetract)
    {
        path = getLatestPathWithConfig(&travelConfig);
        if (!shorterThen(lastPosition - p, retractionMinimalDistance))
        {
            path->retract = true;
        }
    }
    if (path == nullptr)
    {
        path = getLatestPathWithConfig(&travelConfig);
    }
    path->points.push_back(p);
    lastPosition = p;
}

void GCodePlanner::addExtrusionMove(Point p, GCodePathConfig* config, float flow)
{
    getLatestPathWithConfig(config, flow)->points.push_back(p);
    lastPosition = p;
}

void GCodePlanner::addPolygon(PolygonRef polygon, int startIdx, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation)
{
    Point p0 = polygon[startIdx];
    addTravel(p0);
    for(unsigned int i=1; i<polygon.size(); i++)
    {
        Point p1 = polygon[(startIdx + i) % polygon.size()];
        addExtrusionMove(p1, config, (wall_overlap_computation)? wall_overlap_computation->getFlow(p0, p1) : 1.0);
        p0 = p1;
    }
    if (polygon.size() > 2)
    {
        Point& p1 = polygon[startIdx];
        addExtrusionMove(p1, config, (wall_overlap_computation)? wall_overlap_computation->getFlow(p0, p1) : 1.0);
    }
}

void GCodePlanner::addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config, WallOverlapComputation* wall_overlap_computation)
{
    //log("addPolygonsByOptimizer");
    PathOrderOptimizer orderOptimizer(lastPosition);
    for(unsigned int i=0;i<polygons.size();i++)
        orderOptimizer.addPolygon(polygons[i]);
    orderOptimizer.optimize();
    for(unsigned int i=0;i<orderOptimizer.polyOrder.size();i++)
    {
        int nr = orderOptimizer.polyOrder[i];
        addPolygon(polygons[nr], orderOptimizer.polyStart[nr], config, wall_overlap_computation);
    }
}
void GCodePlanner::addLinesByOptimizer(Polygons& polygons, GCodePathConfig* config, int wipe_dist)
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
        addExtrusionMove(p1, config);
        if (wipe_dist != 0)
        {
            int line_width = config->getLineWidth();
            if (vSize2(p1-p0) > line_width * line_width * 4)
            { // otherwise line will get optimized by combining multiple into a single extrusion move
                addExtrusionMove(p1 + normal(p1-p0, wipe_dist), config, 0.0);
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
        for(unsigned int n=0; n<paths.size(); n++)
        {
            GCodePath* path = &paths[n];
            if (path->getExtrusionMM3perMM() == 0)
                continue;
            double speed = path->config->getSpeed() * factor;
            if (speed < minimalSpeed)
                factor = minimalSpeed / path->config->getSpeed();
        }

        //Only slow down with the minimal time if that will be slower then a factor already set. First layer slowdown also sets the speed factor.
        if (factor < getExtrudeSpeedFactor())
            setExtrudeSpeedFactor(factor);
        else
            factor = getExtrudeSpeedFactor();

        if (minTime - (extrudeTime / factor) - travelTime > 0.1)
        {
            this->extraTime = minTime - (extrudeTime / factor) - travelTime;
        }
        this->totalPrintTime = (extrudeTime / factor) + travelTime;
    }else{
        this->totalPrintTime = totalTime;
    }
}

void GCodePlanner::getTimes(double& travelTime, double& extrudeTime)
{
    travelTime = 0.0;
    extrudeTime = 0.0;
    Point p0 = gcode.getPositionXY();
    for(unsigned int n=0; n<paths.size(); n++)
    {
        GCodePath* path = &paths[n];
        for(unsigned int i=0; i<path->points.size(); i++)
        {
            double thisTime = vSizeMM(p0 - path->points[i]) / path->config->getSpeed();
            if (path->getExtrusionMM3perMM() != 0)
                extrudeTime += thisTime;
            else
                travelTime += thisTime;
            p0 = path->points[i];
        }
    }
}

void GCodePlanner::writeGCode(bool liftHeadIfNeeded, int layerThickness)
{
    GCodePathConfig* lastConfig = nullptr;
    int extruder = gcode.getExtruderNr();

    for(unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
    {
        GCodePath& path = paths[path_idx];
        if (extruder != path.extruder)
        {
            extruder = path.extruder;
            gcode.switchExtruder(extruder);
        }else if (path.retract)
        {
            gcode.writeRetraction(path.config->retraction_config);
        }
        if (path.config != &travelConfig && lastConfig != path.config)
        {
            gcode.writeTypeComment(path.config->name);
            lastConfig = path.config;
        }
        double speed = path.config->getSpeed();

        if (path.getExtrusionMM3perMM() != 0)// Only apply the extrudeSpeed to extrusion moves
            speed *= getExtrudeSpeedFactor();
        else
            speed *= getExtrudeSpeedFactor();

        int64_t nozzle_size = 400; // TODO allow the machine settings to be passed on everywhere :: depends on which nozzle!
        
        if (MergeInfillLines::mergeInfillLines(gcode, paths, travelConfig, nozzle_size, speed, path_idx))
        { // !! has effect on path_idx
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
            bool coasting = coasting_config.coasting_enable; 
            if (coasting && path.getExtrusionMM3perMM() > 0)
            {
                coasting = writePathWithCoasting(path_idx, layerThickness
                            , coasting_config.coasting_volume_move, coasting_config.coasting_speed_move, coasting_config.coasting_min_volume_move
                            , coasting_config.coasting_volume_retract, coasting_config.coasting_speed_retract, coasting_config.coasting_min_volume_retract);
            }
            if (! coasting) // not same as 'else', cause we might have changed coasting in the line above...
            { // normal path to gcode algorithm
                if (
                    path_idx + 2 < paths.size() // has a next move
                    && paths[path_idx+1].points.size() == 1 // is single extruded line
                    && paths[path_idx+1].config != &travelConfig // next move is extrusion
                    && paths[path_idx+2].config == &travelConfig // next next move is travel
                    && shorterThen(path.points.back() - gcode.getPositionXY(), 2 * nozzle_size) // preceding extrusion is close by
                    && shorterThen(paths[path_idx+1].points.back() - path.points.back(), 2 * nozzle_size) // extrusion move is small
                    && shorterThen(paths[path_idx+2].points.back() - paths[path_idx+1].points.back(), 2 * nozzle_size) // consecutive extrusion is close by
                ) // change   ||||||   to  /\/\/\/\/ ...
                {
                    gcode.writeMove(paths[path_idx+2].points.back(), speed, paths[path_idx+1].getExtrusionMM3perMM());
                    path_idx += 2;
                }
                else 
                {
                    for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                    {
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
                gcode.writeMove(path.points[point_idx], speed, path.getExtrusionMM3perMM());
            }
        }
    }

    gcode.updateTotalPrintTime();
    if (liftHeadIfNeeded && extraTime > 0.0)
    {
        gcode.writeComment("Small layer, adding delay");
        if (lastConfig)
            gcode.writeRetraction(lastConfig->retraction_config, true);
        gcode.setZ(gcode.getPositionZ() + MM2INT(3.0));
        gcode.writeMove(gcode.getPositionXY(), travelConfig.getSpeed(), 0);
        gcode.writeMove(gcode.getPositionXY() - Point(-MM2INT(20.0), 0), travelConfig.getSpeed(), 0);
        gcode.writeDelay(extraTime);
    }
}

    
bool GCodePlanner::writePathWithCoasting(unsigned int path_idx, int64_t layerThickness, double coasting_volume_move, double coasting_speed_move, double coasting_min_volume_move, double coasting_volume_retract, double coasting_speed_retract, double coasting_min_volume_retract)
{
    GCodePath& path = paths[path_idx];
    if (path_idx + 1 >= paths.size()
        ||
        ! (path.getExtrusionMM3perMM() > 0.0 &&  paths[path_idx + 1].config->getExtrusionMM3perMM() == 0.0) 
        ||
        path.points.size() < 2
        )
    {
        return false;
    }
    GCodePath& path_next = paths[path_idx + 1];
    
    if (path_next.retract)
    {
        if (coasting_volume_retract <= 0) { return false; }
        return writePathWithCoasting(path, path_next, layerThickness, coasting_volume_retract, coasting_speed_retract, coasting_min_volume_retract);
    }
    else
    {
        if (coasting_volume_move <= 0) { return false; }
        return writePathWithCoasting(path, path_next, layerThickness, coasting_volume_move, coasting_speed_move, coasting_min_volume_move);
    }
}  
bool GCodePlanner::writePathWithCoasting(GCodePath& path, GCodePath& path_next, int64_t layerThickness, double coasting_volume, double coasting_speed, double coasting_min_volume)
{

    int64_t coasting_min_dist_considered = 100; // hardcoded setting for when to not perform coasting

    
    double extrude_speed = path.config->getSpeed() * getExtrudeSpeedFactor(); // travel speed 
    
    int64_t coasting_dist = MM2INT(MM2_2INT(coasting_volume) / layerThickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    int64_t coasting_min_dist = MM2INT(MM2_2INT(coasting_min_volume) / layerThickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    
    
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
    
    if (acc_dist_idx_gt_coast_dist == NO_INDEX) 
    { // something has gone wrong; coasting_min_dist < coasting_dist ?
        return false;
    }
    
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
            gcode.writeMove(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM());
        }
        gcode.writeMove(start, extrude_speed, path.getExtrusionMM3perMM());
    }
    
    if (path_next.retract)
    {
        gcode.writeRetraction(path.config->retraction_config);
    }
    
    for (unsigned int point_idx = point_idx_before_start + 1; point_idx < path.points.size(); point_idx++)
    {
        gcode.writeMove(path.points[point_idx], coasting_speed * path.config->getSpeed(), 0);
    }
    
    gcode.setLastCoastedAmountMM3(path.getExtrusionMM3perMM() * INT2MM(actual_coasting_dist));
    
    return true;
}

}//namespace cura
