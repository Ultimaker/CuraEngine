/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */

#include "LayerPlanBuffer.h"
#include "gcodeExport.h"
#include "utils/logoutput.h"
#include "FffProcessor.h"

namespace cura {



void LayerPlanBuffer::flush()
{
    if (buffer.size() > 0)
    {
        insertPreheatCommands(); // insert preheat commands of the very last layer
    }
    while (!buffer.empty())
    {
        buffer.front().writeGCode(gcode);
        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->flushGcode();
        }
        buffer.pop_front();
    }
    
}

void LayerPlanBuffer::insertPreheatCommand(ExtruderPlan& extruder_plan_before, double time_after_extruder_plan_start, int extruder, double temp)
{
    double acc_time = 0.0;
    for (unsigned int path_idx = extruder_plan_before.paths.size() - 1; int(path_idx) != -1 ; path_idx--)
    {
        GCodePath& path = extruder_plan_before.paths[path_idx];
        const double time_this_path = path.estimates.getTotalTime();
        acc_time += time_this_path;
        if (acc_time > time_after_extruder_plan_start)
        {
            const double time_before_path_end = acc_time - time_after_extruder_plan_start;
            bool wait = false;
            extruder_plan_before.insertCommand(path_idx, extruder, temp, wait, time_this_path - time_before_path_end);
            return;
        }
    }
    bool wait = false;
    unsigned int path_idx = 0;
    extruder_plan_before.insertCommand(path_idx, extruder, temp, wait); // insert at start of extruder plan if time_after_extruder_plan_start > extruder_plan.time
}

Preheat::WarmUpResult LayerPlanBuffer::timeBeforeExtruderPlanToInsert(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    double initial_print_temp = preheat_config.getInitialPrintTemp(extruder);
    if (initial_print_temp == 0)
    {
        initial_print_temp = extruder_plan.printing_temperature;
    }

    double in_between_time = 0.0;
    for (unsigned int extruder_plan_before_idx = extruder_plan_idx - 1; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
    { // find a previous extruder plan where the same extruder is used to see what time this extruder wasn't used
        ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_before_idx];
        if (extruder_plan.extruder == extruder)
        {
            Preheat::WarmUpResult warm_up = preheat_config.timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(in_between_time, extruder, initial_print_temp);
            warm_up.heating_time = std::min(in_between_time, warm_up.heating_time + extra_preheat_time);
            return warm_up;
        }
        in_between_time += extruder_plan.estimates.getTotalTime();
    }
    // The last extruder plan with the same extruder falls outside of the buffer
    // assume the nozzle has cooled down to strandby temperature already.
    Preheat::WarmUpResult warm_up;
    warm_up.total_time_window = in_between_time;
    warm_up.lowest_temperature = preheat_config.getStandbyTemp(extruder);
    constexpr bool during_printing = false;
    warm_up.heating_time = preheat_config.timeBeforeEndToInsertPreheatCommand_warmUp(warm_up.lowest_temperature, extruder, initial_print_temp, during_printing);
    if (warm_up.heating_time > in_between_time)
    {
        warm_up.heating_time = in_between_time;
        warm_up.lowest_temperature = in_between_time / preheat_config.getTimeToHeatup1Degree(extruder);
    }
    warm_up.heating_time = warm_up.heating_time + extra_preheat_time;
    return warm_up;
    
}

void LayerPlanBuffer::insertPreheatCommand_singleExtrusion(ExtruderPlan& prev_extruder_plan, int extruder, double required_temp)
{
    // time_before_extruder_plan_end is halved, so that at the layer change the temperature will be half way betewen the two requested temperatures
    double time_before_extruder_plan_end = 0.5 * preheat_config.timeBeforeEndToInsertPreheatCommand_warmUp(prev_extruder_plan.printing_temperature, extruder, required_temp, true);
    time_before_extruder_plan_end = std::min(prev_extruder_plan.estimates.getTotalTime(), time_before_extruder_plan_end);

    insertPreheatCommand(prev_extruder_plan, time_before_extruder_plan_end, extruder, required_temp);
}


void LayerPlanBuffer::handleStandbyTemp(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx, double standby_temp)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    for (unsigned int extruder_plan_before_idx = extruder_plan_idx - 2; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
    {
        if (extruder_plans[extruder_plan_before_idx]->extruder == extruder)
        {
            extruder_plans[extruder_plan_before_idx + 1]->prev_extruder_standby_temp = standby_temp;
            return;
        }
    }
    logWarning("Warning: Couldn't find previous extruder plan so as to set the standby temperature. Inserting temp command in earliest available layer.\n");
    ExtruderPlan& earliest_extruder_plan = *extruder_plans[0];
    constexpr bool wait = false;
    earliest_extruder_plan.insertCommand(0, extruder, standby_temp, wait);
}

void LayerPlanBuffer::insertPreheatCommand_multiExtrusion(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    double initial_print_temp = preheat_config.getInitialPrintTemp(extruder);
    double print_temp = extruder_plan.printing_temperature;
    double final_print_temp = preheat_config.getFinalPrintTemp(extruder);
    if (initial_print_temp == 0)
    {
        initial_print_temp = extruder_plan.printing_temperature;
    }
    
    Preheat::WarmUpResult heating_time_and_from_temp = timeBeforeExtruderPlanToInsert(extruder_plans, extruder_plan_idx);

    if (heating_time_and_from_temp.total_time_window < preheat_config.getMinimalTimeWindow(extruder))
    {
        handleStandbyTemp(extruder_plans, extruder_plan_idx, initial_print_temp);
        return; // don't insert preheat command and just stay on printing temperature
    }
    else
    {
        handleStandbyTemp(extruder_plans, extruder_plan_idx, heating_time_and_from_temp.lowest_temperature);
    }

    double time_before_extruder_plan_to_insert = heating_time_and_from_temp.heating_time;
    for (unsigned int extruder_plan_before_idx = extruder_plan_idx - 1; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
    {
        ExtruderPlan& extruder_plan_before = *extruder_plans[extruder_plan_before_idx];
        assert (extruder_plan_before.extruder != extruder);

        double time_here = extruder_plan_before.estimates.getTotalTime();
        if (time_here >= time_before_extruder_plan_to_insert)
        {
            insertPreheatCommand(extruder_plan_before, time_before_extruder_plan_to_insert, extruder, initial_print_temp);
            return;
        }
        time_before_extruder_plan_to_insert -= time_here;
    }

    // time_before_extruder_plan_to_insert falls before all plans in the buffer
    bool wait = false;
    unsigned int path_idx = 0;
    extruder_plans[0]->insertCommand(path_idx, extruder, initial_print_temp, wait); // insert preheat command at verfy beginning of buffer
}

void LayerPlanBuffer::insertPreheatCommand(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{   
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    
    
    ExtruderPlan* prev_extruder_plan = extruder_plans[extruder_plan_idx - 1];
    
    int prev_extruder = prev_extruder_plan->extruder;
    
    if (prev_extruder != extruder)
    { // set previous extruder to standby temperature
        extruder_plan.prev_extruder_standby_temp = preheat_config.getStandbyTemp(prev_extruder);
    }
    
    if (prev_extruder == extruder)
    {
        insertPreheatCommand_singleExtrusion(*prev_extruder_plan, extruder, extruder_plan.printing_temperature);
    }
    else 
    {
        insertPreheatCommand_multiExtrusion(extruder_plans, extruder_plan_idx);
    }
    
}

void LayerPlanBuffer::insertPreheatCommands()
{
    if (buffer.back().extruder_plans.size() == 0 || (buffer.back().extruder_plans.size() == 1 && buffer.back().extruder_plans[0].paths.size() == 0))
    { // disregard empty layer
        buffer.pop_back();
        return;
    }

    std::vector<ExtruderPlan*> extruder_plans;
    extruder_plans.reserve(buffer.size() * 2);
    for (GCodePlanner& layer_plan : buffer)
    {
        for (ExtruderPlan& extr_plan : layer_plan.extruder_plans)
        {
            extruder_plans.push_back(&extr_plan);
        }
    }


    // insert commands for all extruder plans on this layer
    GCodePlanner& layer_plan = buffer.back();
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
        extruder_plan.printing_temperature = preheat_config.getTemp(extruder_plan.extruder, avg_flow, extruder_plan.is_initial_layer);

        if (buffer.size() == 1 && extruder_plan_idx == 0)
        { // the very first extruder plan of the current meshgroup
            int extruder = extruder_plan.extruder;
            for (int extruder_idx = 0; extruder_idx < getSettingAsCount("machine_extruder_count"); extruder_idx++)
            { // set temperature of the first nozzle, turn other nozzles down
                if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
                {
                    // override values from GCodeExport::setInitialTemps
                    // the first used extruder should be set to the required temp in the start gcode
                    // see  FffGcodeWriter::processStartingCode
                    if (extruder_idx == extruder)
                    {
                        gcode.setInitialTemp(extruder_idx, extruder_plan.printing_temperature);
                    }
                    else 
                    {
                        gcode.setInitialTemp(extruder_idx, preheat_config.getStandbyTemp(extruder_idx));
                    }
                }
                else
                {
                    if (extruder_idx != extruder)
                    { // TODO: do we need to do this?
                        extruder_plan.prev_extruder_standby_temp = preheat_config.getStandbyTemp(extruder_idx);
                    }
                }
            }
            continue;
        }

        unsigned int overall_extruder_plan_idx = extruder_plans.size() - layer_plan.extruder_plans.size() + extruder_plan_idx;
        insertPreheatCommand(extruder_plans, overall_extruder_plan_idx);
    }
}

} // namespace cura