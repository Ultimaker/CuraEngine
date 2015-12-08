/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */

#include "LayerPlanBuffer.h"
#include "gcodeExport.h"
#include "utils/logoutput.h"

namespace cura {



void LayerPlanBuffer::flush()
{
    if (buffer.size() > 0)
    {
        insertPreheatCommands(); // insert preheat commands of the very last layer
    }
    while (!buffer.empty())
    {
        buffer.front().writeGCode(gcode, getSettingBoolean("cool_lift_head"), buffer.front().getLayerNr() > 0 ? getSettingInMicrons("layer_height") : getSettingInMicrons("layer_height_0"));
        if (command_socket)
        {
            command_socket->flushGcode();
        }
        buffer.pop_front();
    }
    
}

void LayerPlanBuffer::insertPreheatCommand(ExtruderPlan& extruder_plan_before, double time_after_extruder_plan_start, int extruder, double temp)
{
    double acc_time = 0.0;
    for (unsigned int path_idx = 0; path_idx < extruder_plan_before.paths.size(); path_idx++)
    {
        GCodePath& path = extruder_plan_before.paths[path_idx];
        acc_time += path.estimates.getTotalTime();
        if (acc_time > time_after_extruder_plan_start)
        {
//                 logError("Inserting %f\t seconds too early!\n", acc_time - time_after_extruder_plan_start);
            extruder_plan_before.insertCommand(path_idx, extruder, temp, false, acc_time - time_after_extruder_plan_start);
            return;
        }
    }
    extruder_plan_before.insertCommand(extruder_plan_before.paths.size(), extruder, temp, false); // insert at end of extruder plan if time_after_extruder_plan_start > extruder_plan.time 
    // = special insert after all extruder plans
}

double LayerPlanBuffer::timeBeforeExtruderPlanToInsert(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    double required_temp = extruder_plan.required_temp;
    
    unsigned int extruder_plan_before_idx = extruder_plan_idx - 1;
    bool first_it = true;
    double in_between_time = 0.0;
    for (unsigned int layer_idx = layer_plan_idx; int(layer_idx) >= 0; layer_idx--)
    {
        GCodePlanner& layer = *layers[layer_idx];
        if (!first_it)
        {
            extruder_plan_before_idx = layer.extruder_plans.size() - 1;
        }
        for ( ; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
        {
            ExtruderPlan& extruder_plan = layer.extruder_plans[extruder_plan_before_idx];
            if (extruder_plan.extruder == extruder)
            {
                return preheat_config.timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(in_between_time, extruder, required_temp);
            }
            in_between_time += extruder_plan.estimates.getTotalTime();
        }
        first_it = false;
    }
    // The last extruder plan with the same extruder falls outside of the buffer
    // assume the nozzle has cooled down to strandby temperature already.
    return preheat_config.timeBeforeEndToInsertPreheatCommand_warmUp(preheat_config.getStandbyTemp(extruder), extruder, required_temp, false);
    
}

void LayerPlanBuffer::insertPreheatCommand_singleExtrusion(ExtruderPlan& prev_extruder_plan, int extruder, double required_temp)
{
    // time_before_extruder_plan_end is halved, so that at the layer change the temperature will be half way betewen the two requested temperatures
    double time_before_extruder_plan_end = 0.5 * preheat_config.timeBeforeEndToInsertPreheatCommand_warmUp(prev_extruder_plan.required_temp, extruder, required_temp, true);
    double time_after_extruder_plan_start = prev_extruder_plan.estimates.getTotalTime() - time_before_extruder_plan_end;
    if (time_after_extruder_plan_start < 0)
    {
        time_after_extruder_plan_start = 0; // don't override the extruder plan with same extruder of the previous layer
    }
        
    insertPreheatCommand(prev_extruder_plan, time_after_extruder_plan_start, extruder, required_temp);
}

void LayerPlanBuffer::insertPreheatCommand_multiExtrusion(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    double required_temp = extruder_plan.required_temp;
    
    extruder_plan.insertCommand(0, extruder, required_temp, true); // just after the extruder switch, wait for the destination temperature to be reached
    
    double time_before_extruder_plan_to_insert = timeBeforeExtruderPlanToInsert(layers, layer_plan_idx, extruder_plan_idx);
    
    unsigned int extruder_plan_before_idx = extruder_plan_idx - 1;
    bool first_it = true; // Whether it's the first iteration of the for loop below
    for (unsigned int layer_idx = layer_plan_idx; int(layer_idx) >= 0; layer_idx--)
    {
        GCodePlanner& layer = *layers[layer_idx];
        if (!first_it)
        {
            extruder_plan_before_idx = layer.extruder_plans.size() - 1;
        }
        for ( ; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
        {
            ExtruderPlan& extruder_plan_before = layer.extruder_plans[extruder_plan_before_idx];
            assert (extruder_plan_before.extruder != extruder);
            
            double time_here = extruder_plan_before.estimates.getTotalTime();
            if (time_here > time_before_extruder_plan_to_insert)
            {
                insertPreheatCommand(extruder_plan_before, time_here - time_before_extruder_plan_to_insert, extruder, required_temp);
                return;
            }
            time_before_extruder_plan_to_insert -= time_here;
            
        }
        first_it = false;
    }
    
    // time_before_extruder_plan_to_insert falls before all plans in the buffer
    ExtruderPlan& first_extruder_plan = layers[0]->extruder_plans[0];
    first_extruder_plan.insertCommand(0, extruder, required_temp, false); // insert preheat command at verfy beginning of buffer
}

void LayerPlanBuffer::insertPreheatCommand(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
{   
    ExtruderPlan& extruder_plan = layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    double required_temp = extruder_plan.required_temp;
    
    
    ExtruderPlan* prev_extruder_plan = nullptr;
    if (extruder_plan_idx == 0)
    {
        if (layer_plan_idx == 0)
        { // the very first extruder plan
            for (int extruder_idx = 0; extruder_idx < getSettingAsCount("machine_extruder_count"); extruder_idx++)
            { // set temperature of the first nozzle, turn other nozzles down
                if (extruder_idx == extruder)
                {
//                     extruder_plan.insertCommand(0, extruder, required_temp, true);
                    // the first used extruder should already be set to the required temp in the start gcode
                }
                else 
                {
                    extruder_plan.insertCommand(0, extruder_idx, preheat_config.getStandbyTemp(extruder_idx), false);
                }
            }
            return;
        }
        prev_extruder_plan = &layers[layer_plan_idx - 1]->extruder_plans.back();
    }
    else 
    {
        prev_extruder_plan = &layers[layer_plan_idx]->extruder_plans[extruder_plan_idx - 1];
    }
    assert(prev_extruder_plan != nullptr);
    
    int prev_extruder = prev_extruder_plan->extruder;
    
    if (prev_extruder != extruder)
    { // set previous extruder to standby temperature
        prev_extruder_plan->insertCommand(prev_extruder_plan->paths.size(), prev_extruder, preheat_config.getStandbyTemp(prev_extruder), false);
    }
    
    if (prev_extruder == extruder)
    {
        if (preheat_config.usesFlowDependentTemp(extruder))
        {
            insertPreheatCommand_singleExtrusion(*prev_extruder_plan, extruder, required_temp);
        }
    }
    else 
    {
        insertPreheatCommand_multiExtrusion(layers, layer_plan_idx, extruder_plan_idx);
    }
    
}

void LayerPlanBuffer::insertPreheatCommands()
{
    if (buffer.back().extruder_plans.size() == 0 || (buffer.back().extruder_plans.size() == 1 && buffer.back().extruder_plans[0].paths.size() == 0))
    { // disregard empty layer
        buffer.pop_back();
        return;
    }
    
    std::vector<GCodePlanner*> layers;
    layers.reserve(buffer.size());
    for (GCodePlanner& layer_plan : buffer)
    {
        layers.push_back(&layer_plan);
    }
    
    unsigned int layer_idx = layers.size() - 1;

    // insert commands for all extruder plans on this layer
    GCodePlanner& layer_plan = *layers[layer_idx];
    for (unsigned int extruder_plan_idx = 0; extruder_plan_idx < layer_plan.extruder_plans.size(); extruder_plan_idx++)
    {
        ExtruderPlan& extruder_plan = layer_plan.extruder_plans[extruder_plan_idx];
        double time = extruder_plan.estimates.getTotalUnretractedTime();
        if (time <= 0.0 
            || extruder_plan.estimates.getMaterial() == 0.0 // extruder plan only consists of moves (when an extruder switch occurs at the beginning of a layer)
        )
        {
            continue;
        }
        double avg_flow = extruder_plan.estimates.getMaterial() / time; // TODO: subtract retracted travel time
        extruder_plan.required_temp = preheat_config.getTemp(extruder_plan.extruder, avg_flow);
        
        insertPreheatCommand(layers, layer_idx, extruder_plan_idx);
    }
}







} // namespace cura