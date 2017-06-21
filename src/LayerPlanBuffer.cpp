/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */

#include "LayerPlanBuffer.h"
#include "gcodeExport.h"
#include "utils/logoutput.h"
#include "FffProcessor.h"

namespace cura {




void LayerPlanBuffer::setPreheatConfig(MeshGroup& settings)
{
    preheat_config.setConfig(settings);
}

void LayerPlanBuffer::push(LayerPlan& layer_plan)
{
    buffer.push_back(&layer_plan);
}

void LayerPlanBuffer::handle(LayerPlan& layer_plan, GCodeExport& gcode)
{
    push(layer_plan);
    LayerPlan* to_be_written = processBuffer();
    if (to_be_written)
    {
        to_be_written->writeGCode(gcode);
        delete to_be_written;
    }
}

LayerPlan* LayerPlanBuffer::processBuffer()
{
    if (buffer.empty())
    {
        return nullptr;
    }
    processFanSpeedLayerTime();
    if (buffer.size() >= 2)
    {
        addConnectingTravelMove(*--(--buffer.end()), *--buffer.end());
    }
    if (buffer.size() > 0)
    {
        insertTempCommands(); // insert preheat commands of the just completed layer plan (not the newly emplaced one)
    }
    if (buffer.size() > buffer_size)
    {
        LayerPlan* ret = buffer.front();
        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->flushGcode();
        }
        buffer.pop_front();
        return ret;
    }
    return nullptr;
}

void LayerPlanBuffer::flush()
{
    if (buffer.size() > 0)
    {
        insertTempCommands(); // insert preheat commands of the very last layer
    }
    while (!buffer.empty())
    {
        buffer.front()->writeGCode(gcode);
        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->flushGcode();
        }
        buffer.pop_front();
    }
}

void LayerPlanBuffer::addConnectingTravelMove(LayerPlan* prev_layer, const LayerPlan* newest_layer)
{
    std::optional<std::pair<Point, bool>> new_layer_destination_state = newest_layer->getFirstTravelDestinationState();

    if (!new_layer_destination_state)
    {
        logWarning("There are empty layers (or layers with empty extruder plans) in the print! Temperature control and cross layer travel moves might suffer.\n");
        return;
    }

    Point first_location_new_layer = new_layer_destination_state->first;

    assert(newest_layer->extruder_plans.front().extruder == prev_layer->extruder_plans.back().extruder);
    assert(newest_layer->extruder_plans.front().paths.size() > 0);
    assert(newest_layer->extruder_plans.front().paths[0].points.size() == 1);
    assert(newest_layer->extruder_plans.front().paths[0].points[0] == first_location_new_layer);


    // if the last planned position in the previous layer isn't the same as the first location of the new layer, travel to the new location
    if (!prev_layer->last_planned_position || *prev_layer->last_planned_position != first_location_new_layer)
    {
        prev_layer->setIsInside(new_layer_destination_state->second);
        prev_layer->addTravel(first_location_new_layer);
    }
}

void LayerPlanBuffer::processFanSpeedLayerTime()
{
    assert(buffer.size() > 0);
    auto newest_layer_it = --buffer.end();
    // Assume the print head is homed at the start of a meshgroup.
    // This introduces small inaccuracies for the naive layer time estimates of the first layer of the second meshgroup.
    // It's not that bad, though. They are naive estimates any way.
    Point starting_position(0, 0);
    if (buffer.size() >= 2)
    {
        auto prev_layer_it = newest_layer_it;
        prev_layer_it--;
        const LayerPlan* prev_layer = *prev_layer_it;
        starting_position = prev_layer->getLastPosition();
    }
    LayerPlan* newest_layer = *newest_layer_it;
    newest_layer->processFanSpeedAndMinimalLayerTime(starting_position);
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

Preheat::WarmUpResult LayerPlanBuffer::computeStandbyTempPlan(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    int extruder = extruder_plan.extruder;
    double initial_print_temp = extruder_plan.required_start_temperature;

    double in_between_time = 0.0; // the duration during which the extruder isn't used
    for (unsigned int extruder_plan_before_idx = extruder_plan_idx - 1; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
    { // find a previous extruder plan where the same extruder is used to see what time this extruder wasn't used
        ExtruderPlan& extruder_plan_before = *extruder_plans[extruder_plan_before_idx];
        if (extruder_plan_before.extruder == extruder)
        {
            double temp_before = preheat_config.getFinalPrintTemp(extruder);
            if (temp_before == 0)
            {
                temp_before = extruder_plan_before.extrusion_temperature.value_or(initial_print_temp);
            }
            constexpr bool during_printing = false;
            Preheat::WarmUpResult warm_up = preheat_config.getWarmUpPointAfterCoolDown(in_between_time, extruder, temp_before, preheat_config.getStandbyTemp(extruder), initial_print_temp, during_printing);
            warm_up.heating_time = std::min(in_between_time, warm_up.heating_time + extra_preheat_time);
            return warm_up;
        }
        in_between_time += extruder_plan_before.estimates.getTotalTime();
    }
    // The last extruder plan with the same extruder falls outside of the buffer
    // assume the nozzle has cooled down to strandby temperature already.
    Preheat::WarmUpResult warm_up;
    warm_up.total_time_window = in_between_time;
    warm_up.lowest_temperature = preheat_config.getStandbyTemp(extruder);
    constexpr bool during_printing = false;
    warm_up.heating_time = preheat_config.getTimeToGoFromTempToTemp(extruder, warm_up.lowest_temperature, initial_print_temp, during_printing);
    if (warm_up.heating_time > in_between_time)
    {
        warm_up.heating_time = in_between_time;
        warm_up.lowest_temperature = initial_print_temp - in_between_time / preheat_config.getTimeToHeatup1Degree(extruder, during_printing);
    }
    warm_up.heating_time = warm_up.heating_time + extra_preheat_time;
    return warm_up;
}

void LayerPlanBuffer::insertPreheatCommand_singleExtrusion(ExtruderPlan& prev_extruder_plan, int extruder, double required_temp)
{
    if (!gcode.getExtruderUsesTemp(extruder))
    {
        return;
    }
    // time_before_extruder_plan_end is halved, so that at the layer change the temperature will be half way betewen the two requested temperatures
    constexpr bool during_printing = true;
    const double prev_extrusion_temp = prev_extruder_plan.extrusion_temperature.value_or(prev_extruder_plan.required_start_temperature);
    double time_before_extruder_plan_end = 0.5 * preheat_config.getTimeToGoFromTempToTemp(extruder, prev_extrusion_temp, required_temp, during_printing);
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
    const int extruder = extruder_plan.extruder;
    if (!gcode.getExtruderUsesTemp(extruder))
    {
        return;
    }
    double initial_print_temp = extruder_plan.required_start_temperature;
    
    Preheat::WarmUpResult heating_time_and_from_temp = computeStandbyTempPlan(extruder_plans, extruder_plan_idx);

    if (heating_time_and_from_temp.total_time_window < preheat_config.getMinimalTimeWindow(extruder))
    {
        handleStandbyTemp(extruder_plans, extruder_plan_idx, initial_print_temp);
        return; // don't insert preheat command and just stay on printing temperature
    }
    else if (heating_time_and_from_temp.heating_time < heating_time_and_from_temp.total_time_window)
    { // only insert command to cool down to standby temperature if there is some time to cool before heating up again
        handleStandbyTemp(extruder_plans, extruder_plan_idx, heating_time_and_from_temp.lowest_temperature);
    }

    // handle preheat command
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

void LayerPlanBuffer::insertTempCommands(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
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
        insertPreheatCommand_singleExtrusion(*prev_extruder_plan, extruder, extruder_plan.required_start_temperature);
        prev_extruder_plan->extrusion_temperature_command = --prev_extruder_plan->inserts.end();
    }
    else 
    {
        insertPreheatCommand_multiExtrusion(extruder_plans, extruder_plan_idx);
        insertFinalPrintTempCommand(extruder_plans, extruder_plan_idx - 1);
        insertPrintTempCommand(extruder_plan);
    }
}

void LayerPlanBuffer::insertPrintTempCommand(ExtruderPlan& extruder_plan)
{
    const unsigned int extruder = extruder_plan.extruder;

    if (!extruder_plan.extrusion_temperature)
    {
        logWarning("Empty extruder plan detected! Discarding extrusion temperature command.\n");
        return;
    }

    const double print_temp = *extruder_plan.extrusion_temperature;
    if (!gcode.getExtruderUsesTemp(extruder))
    {
        return;
    }

    double heated_pre_travel_time = 0;
    if (preheat_config.getInitialPrintTemp(extruder) != 0)
    { // handle heating from initial_print_temperature to printing_tempreature
        unsigned int path_idx;
        for (path_idx = 0; path_idx < extruder_plan.paths.size(); path_idx++)
        {
            GCodePath& path = extruder_plan.paths[path_idx];
            heated_pre_travel_time += path.estimates.getTotalTime();
            if (!path.isTravelPath())
            {
                break;
            }
        }
        bool wait = false;
        extruder_plan.insertCommand(path_idx, extruder, print_temp, wait);
    }
    extruder_plan.heated_pre_travel_time = heated_pre_travel_time;
}

void LayerPlanBuffer::insertFinalPrintTempCommand(std::vector<ExtruderPlan*>& extruder_plans, unsigned int last_extruder_plan_idx)
{
    ExtruderPlan& last_extruder_plan = *extruder_plans[last_extruder_plan_idx];
    const int extruder = last_extruder_plan.extruder;
    if (!gcode.getExtruderUsesTemp(extruder))
    {
        return;
    }

    double final_print_temp = preheat_config.getFinalPrintTemp(extruder);
    if (final_print_temp == 0)
    {
        return;
    }

    double heated_post_travel_time = 0; // The time after the last extrude move toward the end of the extruder plan during which the nozzle is stable at the final print temperature
    { // compute heated_post_travel_time
        unsigned int path_idx;
        for (path_idx = last_extruder_plan.paths.size() - 1; int(path_idx) >= 0; path_idx--)
        {
            GCodePath& path = last_extruder_plan.paths[path_idx];
            if (!path.isTravelPath())
            {
                break;
            }
            heated_post_travel_time += path.estimates.getTotalTime();
        }
    }

    double time_window = 0; // The time window within which the nozzle needs to heat from the initial print temp to the printing temperature and then back to the final print temp; i.e. from the first to the last extrusion move with this extruder
    double weighted_average_extrusion_temp = 0; // The average of the normal extrusion temperatures of the extruder plans (which might be different due to flow dependent temp or due to initial layer temp) Weighted by time
    std::optional<double> initial_print_temp; // The initial print temp of the first extruder plan with this extruder
    { // compute time window and print temp statistics
        double heated_pre_travel_time = -1; // The time before the first extrude move from the start of the extruder plan during which the nozzle is stable at the initial print temperature
        for (unsigned int prev_extruder_plan_idx = last_extruder_plan_idx; (int)prev_extruder_plan_idx >= 0; prev_extruder_plan_idx--)
        {
            ExtruderPlan& prev_extruder_plan = *extruder_plans[prev_extruder_plan_idx];
            if (prev_extruder_plan.extruder != extruder)
            {
                break;
            }
            double prev_extruder_plan_time = prev_extruder_plan.estimates.getTotalTime();
            time_window += prev_extruder_plan_time;
            heated_pre_travel_time = prev_extruder_plan.heated_pre_travel_time;

            if (prev_extruder_plan.estimates.getTotalUnretractedTime() > 0)
            { // handle temp statistics
                weighted_average_extrusion_temp += prev_extruder_plan.extrusion_temperature.value_or(prev_extruder_plan.required_start_temperature) * prev_extruder_plan_time;
                initial_print_temp = prev_extruder_plan.required_start_temperature;
            }
        }
        assert(time_window != 0.0);
        weighted_average_extrusion_temp /= time_window;
        time_window -= heated_pre_travel_time + heated_post_travel_time;
        assert(heated_pre_travel_time != -1 && "heated_pre_travel_time must have been computed; there must have been an extruder plan!");
    }

    if (!initial_print_temp)
    { // none of the extruder plans had unretracted moves
        logWarning("Unneccesary extruder switch detected! Discarding final print temperature commands.\n");
        return;
    }

    assert((time_window >= 0 || last_extruder_plan.estimates.getMaterial() == 0) && "Time window should always be positive if we actually extrude");

    //          ,layer change                                                                                   .
    //          :     ,precool command                   ,layer change                                          .
    //          : ____:                                  :     ,precool command                                 .
    //          :/    \                             _____:_____:                                                .
    //     _____/      \                           /           \                                                .
    //    /             \                         /             \                                               .
    //   /                                       /                                                              .
    //  /                                       /                                                               .
    //                                                                                                          .
    // approximate ^               by                   ^                                                       .
    // This approximation is quite ok since it only determines where to insert the precool temp command,
    // which means the stable temperature of the previous extruder plan and the stable temperature of the next extruder plan couldn't be reached
    constexpr bool during_printing = true;
    Preheat::CoolDownResult warm_cool_result = preheat_config.getCoolDownPointAfterWarmUp(time_window, extruder, *initial_print_temp, weighted_average_extrusion_temp, final_print_temp, during_printing);
    double cool_down_time = warm_cool_result.cooling_time;
    assert(cool_down_time >= 0);

    // find extruder plan in which to insert cooling command
    ExtruderPlan* precool_extruder_plan = &last_extruder_plan;
    {
        for (unsigned int precool_extruder_plan_idx = last_extruder_plan_idx; (int)precool_extruder_plan_idx >= 0; precool_extruder_plan_idx--)
        {
            precool_extruder_plan = extruder_plans[precool_extruder_plan_idx];
            if (precool_extruder_plan->extrusion_temperature_command)
            { // the precool command ends up before the command to go to the print temperature of the next extruder plan, so remove that print temp command
                precool_extruder_plan->inserts.erase(*precool_extruder_plan->extrusion_temperature_command);
            }
            double time_here = precool_extruder_plan->estimates.getTotalTime();
            if (cool_down_time < time_here)
            {
                break;
            }
            cool_down_time -= time_here;
        }
    }

    // at this point cool_down_time is what time is left if cool down time of extruder plans after precool_extruder_plan (up until last_extruder_plan) are already taken into account

    { // insert temp command in precool_extruder_plan
        double extrusion_time_seen = 0;
        unsigned int path_idx;
        for (path_idx = precool_extruder_plan->paths.size() - 1; int(path_idx) >= 0; path_idx--)
        {
            GCodePath& path = precool_extruder_plan->paths[path_idx];
            extrusion_time_seen += path.estimates.getTotalTime();
            if (extrusion_time_seen >= cool_down_time)
            {
                break;
            }
        }
        bool wait = false;
        double time_after_path_start = extrusion_time_seen - cool_down_time;
        precool_extruder_plan->insertCommand(path_idx, extruder, final_print_temp, wait, time_after_path_start);
    }
}


void LayerPlanBuffer::insertTempCommands()
{
    if (buffer.back()->extruder_plans.size() == 0 || (buffer.back()->extruder_plans.size() == 1 && buffer.back()->extruder_plans[0].paths.size() == 0))
    { // disregard empty layer
        buffer.pop_back();
        return;
    }

    std::vector<ExtruderPlan*> extruder_plans;
    extruder_plans.reserve(buffer.size() * 2);
    for (LayerPlan* layer_plan : buffer)
    {
        for (ExtruderPlan& extr_plan : layer_plan->extruder_plans)
        {
            extruder_plans.push_back(&extr_plan);
        }
    }


    // insert commands for all extruder plans on this layer
    LayerPlan& layer_plan = *buffer.back();
    for (unsigned int extruder_plan_idx = 0; extruder_plan_idx < layer_plan.extruder_plans.size(); extruder_plan_idx++)
    {
        unsigned int overall_extruder_plan_idx = extruder_plans.size() - layer_plan.extruder_plans.size() + extruder_plan_idx;
        ExtruderPlan& extruder_plan = layer_plan.extruder_plans[extruder_plan_idx];
        int extruder = extruder_plan.extruder;
        double time = extruder_plan.estimates.getTotalUnretractedTime();
        double avg_flow;
        if (time > 0.0)
        {
            avg_flow = extruder_plan.estimates.getMaterial() / time;
        }
        else
        {
            assert(extruder_plan.estimates.getMaterial() == 0.0 && "No extrusion time should mean no material usage!");
            logWarning("Empty extruder plans detected! Temperature control might suffer.\n");
            avg_flow = 0.0;
        }

        double print_temp = preheat_config.getTemp(extruder, avg_flow, extruder_plan.is_initial_layer);
        double initial_print_temp = preheat_config.getInitialPrintTemp(extruder);
        if (initial_print_temp == 0.0 // user doesn't want to use initial print temp feature
            || !extruder_used_in_meshgroup[extruder] // prime blob uses print temp rather than initial print temp
            || (overall_extruder_plan_idx > 0 && extruder_plans[overall_extruder_plan_idx - 1]->extruder == extruder  // prev plan has same extruder ..
                && extruder_plans[overall_extruder_plan_idx - 1]->estimates.getTotalUnretractedTime() > 0.0) // and prev extruder plan already heated to printing temperature
        )
        {
            extruder_plan.required_start_temperature = print_temp;
            extruder_used_in_meshgroup[extruder] = true;
        }
        else
        {
            extruder_plan.required_start_temperature = initial_print_temp;
            extruder_plan.extrusion_temperature = print_temp;
        }
        assert(extruder_plan.required_start_temperature != -1 && "extruder_plan.required_start_temperature should now have been set");

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
                        gcode.setInitialTemp(extruder_idx, extruder_plan.extrusion_temperature.value_or(extruder_plan.required_start_temperature));
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

        insertTempCommands(extruder_plans, overall_extruder_plan_idx);
    }
}

} // namespace cura