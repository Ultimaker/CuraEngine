// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "LayerPlanBuffer.h"

#include <spdlog/spdlog.h>

#include "Application.h" //To flush g-code through the communication channel.
#include "ExtruderTrain.h"
#include "FffProcessor.h"
#include "LayerPlan.h"
#include "Slice.h"
#include "communication/Communication.h" //To flush g-code through the communication channel.
#include "gcodeExport.h"

namespace cura
{


constexpr Duration LayerPlanBuffer::extra_preheat_time_;

void LayerPlanBuffer::push(LayerPlan& layer_plan)
{
    buffer_.push_back(&layer_plan);
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
    if (buffer_.empty())
    {
        return nullptr;
    }
    processFanSpeedLayerTime();
    if (buffer_.size() >= 2)
    {
        addConnectingTravelMove(*--(--buffer_.end()), *--buffer_.end());
    }
    if (buffer_.size() > 0)
    {
        insertTempCommands(); // insert preheat commands of the just completed layer plan (not the newly emplaced one)
    }
    if (buffer_.size() > buffer_size_)
    {
        LayerPlan* ret = buffer_.front();
        Application::getInstance().communication_->flushGCode();
        buffer_.pop_front();
        return ret;
    }
    return nullptr;
}

void LayerPlanBuffer::flush()
{
    Application::getInstance()
        .communication_->flushGCode(); // If there was still g-code in a layer, flush that as a separate layer. Don't want to group them together accidentally.
    if (buffer_.size() > 0)
    {
        insertTempCommands(); // insert preheat commands of the very last layer
    }
    while (! buffer_.empty())
    {
        buffer_.front()->writeGCode(gcode_);
        Application::getInstance().communication_->flushGCode();
        delete buffer_.front();
        buffer_.pop_front();
    }
}

void LayerPlanBuffer::addConnectingTravelMove(LayerPlan* prev_layer, const LayerPlan* newest_layer)
{
    std::optional<std::pair<Point2LL, bool>> new_layer_destination_state = newest_layer->getFirstTravelDestinationState();

    if (! new_layer_destination_state)
    {
        spdlog::warn("Layer {} is empty (or it has empty extruder plans). Temperature control and cross layer travel moves might suffer!", newest_layer->layer_nr_);
        return;
    }

    Point2LL first_location_new_layer = new_layer_destination_state->first;

    assert(newest_layer->extruder_plans_.front().paths_[0].points.size() == 1);
    assert(newest_layer->extruder_plans_.front().paths_[0].points[0].toPoint2LL() == first_location_new_layer);

    // if the last planned position in the previous layer isn't the same as the first location of the new layer, travel to the new location
    if (! prev_layer->last_planned_position_ || *prev_layer->last_planned_position_ != first_location_new_layer)
    {
        const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
        const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[prev_layer->extruder_plans_.back().extruder_nr_].settings_;
        prev_layer->setIsInside(new_layer_destination_state->second);

        const bool travel_retract_before_outer_wall = mesh_group_settings.get<bool>("travel_retract_before_outer_wall");
        const bool retract_at_layer_change = extruder_settings.get<bool>("retract_at_layer_change");
        bool next_mesh_retract_before_outer_wall = false;
        std::shared_ptr<const SliceMeshStorage> first_printed_mesh = newest_layer->findFirstPrintedMesh();
        if (! retract_at_layer_change && first_printed_mesh && travel_retract_before_outer_wall)
        {
            // Check whether we are moving toving towards an outer wall and it should be retracted
            const Settings& mesh_settings = first_printed_mesh->settings;
            const InsetDirection inset_direction = mesh_settings.get<InsetDirection>("inset_direction");
            const size_t wall_line_count = mesh_settings.get<size_t>("wall_line_count");

            next_mesh_retract_before_outer_wall = inset_direction == InsetDirection::OUTSIDE_IN || wall_line_count == 1;
        }
        const bool force_retract = retract_at_layer_change || next_mesh_retract_before_outer_wall;
        prev_layer->final_travel_z_ = newest_layer->z_;
        GCodePath& path = prev_layer->addTravel(first_location_new_layer, force_retract);
        if (force_retract && ! path.retract)
        {
            // addTravel() won't use retraction if the travel distance is less than retraction minimum travel setting
            // so to avoid blobs when moving to the new layer height, which can occur if the z-axis speed is very slow,
            // we force the path to use retraction
            path.retract = true;
        }
    }

    // If not using travel-specific jerk and acceleration, the layer plan needs to know the jerk/acc of the first extrusion move of the next layer.
    prev_layer->next_layer_acc_jerk_ = newest_layer->first_extrusion_acc_jerk_;
}

void LayerPlanBuffer::processFanSpeedLayerTime()
{
    assert(buffer_.size() > 0);
    auto newest_layer_it = --buffer_.end();
    // Assume the print head is homed at the start of a mesh group.
    // This introduces small inaccuracies for the naive layer time estimates of the first layer of the second mesh group.
    // It's not that bad, though. They are naive estimates any way.
    Point2LL starting_position(0, 0);
    if (buffer_.size() >= 2)
    {
        auto prev_layer_it = newest_layer_it;
        prev_layer_it--;
        const LayerPlan* prev_layer = *prev_layer_it;
        starting_position = prev_layer->getLastPlannedPositionOrStartingPosition();
    }
    LayerPlan* newest_layer = *newest_layer_it;
    newest_layer->processFanSpeedAndMinimalLayerTime(starting_position);
}


void LayerPlanBuffer::insertPreheatCommand(ExtruderPlan& extruder_plan_before, const Duration time_before_extruder_plan_end, const size_t extruder_nr, const Temperature temp)
{
    Duration acc_time = 0.0;
    for (unsigned int path_idx = extruder_plan_before.paths_.size() - 1; int(path_idx) != -1; path_idx--)
    {
        GCodePath& path = extruder_plan_before.paths_[path_idx];
        acc_time += path.estimates.getTotalTime();
        if (acc_time >= time_before_extruder_plan_end)
        {
            constexpr bool wait = false;
            extruder_plan_before.insertCommand(NozzleTempInsert{ .path_idx = path_idx,
                                                                 .extruder = extruder_nr,
                                                                 .temperature = temp,
                                                                 .wait = wait,
                                                                 .time_after_path_start = acc_time - time_before_extruder_plan_end });
            return;
        }
    }
    constexpr bool wait = false;
    constexpr size_t path_idx = 0;
    extruder_plan_before.insertCommand(NozzleTempInsert{ .path_idx = path_idx,
                                                         .extruder = extruder_nr,
                                                         .temperature = temp,
                                                         .wait = wait }); // insert at start of extruder plan if time_after_extruder_plan_start > extruder_plan.time
}

Preheat::WarmUpResult LayerPlanBuffer::computeStandbyTempPlan(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    size_t extruder = extruder_plan.extruder_nr_;
    Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder].settings_;
    double initial_print_temp = extruder_plan.required_start_temperature_;

    Duration in_between_time = 0.0_s; // the duration during which the extruder isn't used
    for (size_t extruder_plan_before_idx = extruder_plan_idx - 1; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
    { // find a previous extruder plan where the same extruder is used to see what time this extruder wasn't used
        ExtruderPlan& extruder_plan_before = *extruder_plans[extruder_plan_before_idx];
        if (extruder_plan_before.extruder_nr_ == extruder)
        {
            Temperature temp_before = extruder_settings.get<Temperature>("material_final_print_temperature");
            if (temp_before == 0)
            {
                temp_before = extruder_plan_before.extrusion_temperature_.value_or(initial_print_temp);
            }
            constexpr bool during_printing = false;
            Preheat::WarmUpResult warm_up = preheat_config_.getWarmUpPointAfterCoolDown(
                in_between_time,
                extruder,
                temp_before,
                extruder_settings.get<Temperature>("material_standby_temperature"),
                initial_print_temp,
                during_printing);
            warm_up.heating_time = std::min(in_between_time, warm_up.heating_time + extra_preheat_time_);
            return warm_up;
        }
        in_between_time += extruder_plan_before.estimates_.getTotalTime();
    }
    // The last extruder plan with the same extruder falls outside of the buffer
    // assume the nozzle has cooled down to strandby temperature already.
    Preheat::WarmUpResult warm_up;
    warm_up.total_time_window = in_between_time;
    warm_up.lowest_temperature = extruder_settings.get<Temperature>("material_standby_temperature");
    constexpr bool during_printing = false;
    warm_up.heating_time = preheat_config_.getTimeToGoFromTempToTemp(extruder, warm_up.lowest_temperature, initial_print_temp, during_printing);
    if (warm_up.heating_time > in_between_time)
    {
        warm_up.heating_time = in_between_time;
        warm_up.lowest_temperature = initial_print_temp - in_between_time * extruder_settings.get<Temperature>("machine_nozzle_heat_up_speed");
    }
    warm_up.heating_time = warm_up.heating_time + extra_preheat_time_;
    return warm_up;
}

void LayerPlanBuffer::insertPreheatCommand_singleExtrusion(ExtruderPlan& prev_extruder_plan, const size_t extruder_nr, const Temperature required_temp)
{
    if (! Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }
    // time_before_extruder_plan_end is halved, so that at the layer change the temperature will be half way betewen the two requested temperatures
    constexpr bool during_printing = true;
    const double prev_extrusion_temp = prev_extruder_plan.extrusion_temperature_.value_or(prev_extruder_plan.required_start_temperature_);
    double time_before_extruder_plan_end = 0.5 * preheat_config_.getTimeToGoFromTempToTemp(extruder_nr, prev_extrusion_temp, required_temp, during_printing);
    time_before_extruder_plan_end = std::min(prev_extruder_plan.estimates_.getTotalTime(), time_before_extruder_plan_end);

    insertPreheatCommand(prev_extruder_plan, time_before_extruder_plan_end, extruder_nr, required_temp);
}


void LayerPlanBuffer::handleStandbyTemp(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx, double standby_temp)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    size_t extruder = extruder_plan.extruder_nr_;
    for (size_t extruder_plan_before_idx = extruder_plan_idx - 2; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
    {
        if (extruder_plans[extruder_plan_before_idx]->extruder_nr_ == extruder)
        {
            extruder_plans[extruder_plan_before_idx + 1]->prev_extruder_standby_temp_ = standby_temp;
            return;
        }
    }
    spdlog::warn("Couldn't find previous extruder plan so as to set the standby temperature. Inserting temp command in earliest available layer.");
    ExtruderPlan& earliest_extruder_plan = *extruder_plans[0];
    constexpr bool wait = false;
    earliest_extruder_plan.insertCommand(NozzleTempInsert{ .path_idx = 0, .extruder = extruder, .temperature = standby_temp, .wait = wait });
}

void LayerPlanBuffer::insertPreheatCommand_multiExtrusion(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    const size_t extruder = extruder_plan.extruder_nr_;
    const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder].settings_;
    if (! extruder_settings.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }
    double initial_print_temp = extruder_plan.required_start_temperature_;

    Preheat::WarmUpResult heating_time_and_from_temp = computeStandbyTempPlan(extruder_plans, extruder_plan_idx);

    if (heating_time_and_from_temp.total_time_window < extruder_settings.get<Duration>("machine_min_cool_heat_time_window"))
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
        assert(extruder_plan_before.extruder_nr_ != extruder);

        double time_here = extruder_plan_before.estimates_.getTotalTime();
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
    extruder_plans[0]->insertCommand(
        NozzleTempInsert{ .path_idx = path_idx, .extruder = extruder, .temperature = initial_print_temp, .wait = wait }); // insert preheat command at verfy beginning of buffer
}

void LayerPlanBuffer::insertTempCommands(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
{
    ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
    const size_t extruder = extruder_plan.extruder_nr_;

    ExtruderPlan* prev_extruder_plan = extruder_plans[extruder_plan_idx - 1];

    const size_t prev_extruder = prev_extruder_plan->extruder_nr_;

    if (prev_extruder != extruder)
    { // set previous extruder to standby temperature
        const Settings& previous_extruder_settings = Application::getInstance().current_slice_->scene.extruders[prev_extruder].settings_;
        extruder_plan.prev_extruder_standby_temp_ = previous_extruder_settings.get<Temperature>("material_standby_temperature");
    }

    if (prev_extruder == extruder)
    {
        insertPreheatCommand_singleExtrusion(*prev_extruder_plan, extruder, extruder_plan.required_start_temperature_);
        prev_extruder_plan->extrusion_temperature_command_ = --prev_extruder_plan->inserts_.end();
    }
    else if (Application::getInstance().current_slice_->scene.extruders[extruder].settings_.get<bool>("machine_extruders_share_heater"))
    {
        // extruders share a heater so command the previous extruder to change to the temperature required for this extruder
        insertPreheatCommand_singleExtrusion(*prev_extruder_plan, prev_extruder, extruder_plan.required_start_temperature_);
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
    if (! extruder_plan.extrusion_temperature_)
    {
        spdlog::warn("Empty extruder plan detected! Discarding extrusion temperature command.");
        return;
    }
    const double print_temp = *extruder_plan.extrusion_temperature_;

    const unsigned int extruder = extruder_plan.extruder_nr_;
    const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder].settings_;
    if (! extruder_settings.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }

    double heated_pre_travel_time = 0;
    if (extruder_settings.get<Temperature>("material_initial_print_temperature") != 0)
    { // handle heating from initial_print_temperature to printing_tempreature
        unsigned int path_idx;
        for (path_idx = 0; path_idx < extruder_plan.paths_.size(); path_idx++)
        {
            GCodePath& path = extruder_plan.paths_[path_idx];
            heated_pre_travel_time += path.estimates.getTotalTime();
            if (! path.isTravelPath())
            {
                break;
            }
        }
        bool wait = false;
        extruder_plan.insertCommand(NozzleTempInsert{ .path_idx = path_idx, .extruder = extruder, .temperature = print_temp, .wait = wait });
    }
    extruder_plan.heated_pre_travel_time_ = heated_pre_travel_time;
}

void LayerPlanBuffer::insertFinalPrintTempCommand(std::vector<ExtruderPlan*>& extruder_plans, unsigned int last_extruder_plan_idx)
{
    ExtruderPlan& last_extruder_plan = *extruder_plans[last_extruder_plan_idx];
    const size_t extruder = last_extruder_plan.extruder_nr_;
    const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder].settings_;
    if (! extruder_settings.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }

    const Temperature final_print_temp = extruder_settings.get<Temperature>("material_final_print_temperature");
    if (final_print_temp == 0)
    {
        return;
    }

    double heated_post_travel_time = 0; // The time after the last extrude move toward the end of the extruder plan during which the nozzle is stable at the final print temperature
    { // compute heated_post_travel_time
        unsigned int path_idx;
        for (path_idx = last_extruder_plan.paths_.size() - 1; int(path_idx) >= 0; path_idx--)
        {
            GCodePath& path = last_extruder_plan.paths_[path_idx];
            if (! path.isTravelPath())
            {
                break;
            }
            heated_post_travel_time += path.estimates.getTotalTime();
        }
    }

    double time_window = 0; // The time window within which the nozzle needs to heat from the initial print temp to the printing temperature and then back to the final print temp;
                            // i.e. from the first to the last extrusion move with this extruder
    double weighted_average_extrusion_temp = 0; // The average of the normal extrusion temperatures of the extruder plans (which might be different due to flow dependent temp or
                                                // due to initial layer temp) Weighted by time
    std::optional<double> initial_print_temp; // The initial print temp of the first extruder plan with this extruder
    { // compute time window and print temp statistics
        double heated_pre_travel_time
            = -1; // The time before the first extrude move from the start of the extruder plan during which the nozzle is stable at the initial print temperature
        for (unsigned int prev_extruder_plan_idx = last_extruder_plan_idx; (int)prev_extruder_plan_idx >= 0; prev_extruder_plan_idx--)
        {
            ExtruderPlan& prev_extruder_plan = *extruder_plans[prev_extruder_plan_idx];
            if (prev_extruder_plan.extruder_nr_ != extruder)
            {
                break;
            }
            double prev_extruder_plan_time = prev_extruder_plan.estimates_.getTotalTime();
            time_window += prev_extruder_plan_time;
            heated_pre_travel_time = prev_extruder_plan.heated_pre_travel_time_;

            if (prev_extruder_plan.estimates_.getTotalUnretractedTime() > 0)
            { // handle temp statistics
                weighted_average_extrusion_temp += prev_extruder_plan.extrusion_temperature_.value_or(prev_extruder_plan.required_start_temperature_) * prev_extruder_plan_time;
                initial_print_temp = prev_extruder_plan.required_start_temperature_;
            }
        }
        if (time_window <= 0.0) // There was a move in this plan but it was length 0.
        {
            spdlog::warn("Unnecessary extruder switch detected! SliceDataStorage::getExtrudersUsed should probably be updated.");
            return;
        }
        weighted_average_extrusion_temp /= time_window;
        time_window -= heated_pre_travel_time + heated_post_travel_time;
        assert(heated_pre_travel_time != -1 && "heated_pre_travel_time must have been computed; there must have been an extruder plan!");
    }

    if (! initial_print_temp)
    { // none of the extruder plans had unretracted moves
        spdlog::warn("Unnecessary extruder switch detected! Discarding final print temperature commands.");
        return;
    }

    assert((time_window >= -0.001 || last_extruder_plan.estimates_.material == 0) && "Time window should always be positive if we actually extrude");

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
    Preheat::CoolDownResult warm_cool_result
        = preheat_config_.getCoolDownPointAfterWarmUp(time_window, extruder, *initial_print_temp, weighted_average_extrusion_temp, final_print_temp, during_printing);
    double cool_down_time = warm_cool_result.cooling_time;
    assert(cool_down_time >= 0);

    // find extruder plan in which to insert cooling command
    ExtruderPlan* precool_extruder_plan = &last_extruder_plan;
    {
        for (unsigned int precool_extruder_plan_idx = last_extruder_plan_idx; (int)precool_extruder_plan_idx >= 0; precool_extruder_plan_idx--)
        {
            precool_extruder_plan = extruder_plans[precool_extruder_plan_idx];
            if (precool_extruder_plan->extrusion_temperature_command_)
            { // the precool command ends up before the command to go to the print temperature of the next extruder plan, so remove that print temp command
                precool_extruder_plan->inserts_.erase(*precool_extruder_plan->extrusion_temperature_command_);
            }
            double time_here = precool_extruder_plan->estimates_.getTotalTime();
            if (cool_down_time < time_here)
            {
                break;
            }
            cool_down_time -= time_here;
        }
    }

    // at this point cool_down_time is what time is left if cool down time of extruder plans after precool_extruder_plan (up until last_extruder_plan) are already taken into
    // account

    { // insert temp command in precool_extruder_plan
        double extrusion_time_seen = 0;
        unsigned int path_idx;
        for (path_idx = precool_extruder_plan->paths_.size() - 1; int(path_idx) >= 0; path_idx--)
        {
            GCodePath& path = precool_extruder_plan->paths_[path_idx];
            extrusion_time_seen += path.estimates.getTotalTime();
            if (extrusion_time_seen >= cool_down_time)
            {
                break;
            }
        }
        bool wait = false;
        double time_after_path_start = extrusion_time_seen - cool_down_time;
        precool_extruder_plan->insertCommand(
            NozzleTempInsert{ .path_idx = path_idx, .extruder = extruder, .temperature = final_print_temp, .wait = wait, .time_after_path_start = time_after_path_start });
    }
}

void LayerPlanBuffer::insertTempCommands()
{
    if (buffer_.back()->extruder_plans_.size() == 0 || (buffer_.back()->extruder_plans_.size() == 1 && buffer_.back()->extruder_plans_[0].paths_.size() == 0))
    { // disregard empty layer
        buffer_.pop_back();
        return;
    }

    std::vector<ExtruderPlan*> extruder_plans; // sorted in print order
    extruder_plans.reserve(buffer_.size() * 2);
    for (LayerPlan* layer_plan : buffer_)
    {
        for (ExtruderPlan& extr_plan : layer_plan->extruder_plans_)
        {
            extruder_plans.push_back(&extr_plan);
        }
    }

    // insert commands for all extruder plans on this layer
    Scene& scene = Application::getInstance().current_slice_->scene;
    LayerPlan& layer_plan = *buffer_.back();
    for (size_t extruder_plan_idx = 0; extruder_plan_idx < layer_plan.extruder_plans_.size(); extruder_plan_idx++)
    {
        const size_t overall_extruder_plan_idx = extruder_plans.size() - layer_plan.extruder_plans_.size() + extruder_plan_idx;
        ExtruderPlan& extruder_plan = layer_plan.extruder_plans_[extruder_plan_idx];
        size_t extruder = extruder_plan.extruder_nr_;
        const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder].settings_;
        Duration time = extruder_plan.estimates_.getTotalUnretractedTime();
        if (time <= 0.0)
        {
            assert(extruder_plan.estimates_.material == 0.0 && "No extrusion time should mean no material usage!");
        }

        Temperature print_temp = preheat_config_.getTemp(extruder, extruder_plan.is_initial_layer_);
        Temperature initial_print_temp = extruder_settings.get<Temperature>("material_initial_print_temperature");

        if (extruder_plan.temperature_factor_ > 0) // force lower printing temperatures due to minimum layer time
        {
            print_temp = print_temp * (1 - extruder_plan.temperature_factor_) + extruder_plan.temperature_factor_ * extruder_settings.get<Temperature>("cool_min_temperature");
            initial_print_temp = std::min(initial_print_temp, print_temp);
        }

        if (initial_print_temp == 0.0 // user doesn't want to use initial print temp feature
            || extruder_settings.get<bool>("machine_extruders_share_heater") // ignore initial print temps when extruders share a heater
            || (! extruder_used_in_meshgroup_[extruder] && extruder_settings.get<bool>("prime_blob_enable")) // prime blob uses print temp rather than initial print temp
            || (overall_extruder_plan_idx > 0 && extruder_plans[overall_extruder_plan_idx - 1]->extruder_nr_ == extruder // prev plan has same extruder ..
                && extruder_plans[overall_extruder_plan_idx - 1]->estimates_.getTotalUnretractedTime() > 0.0) // and prev extruder plan already heated to printing temperature
        )
        {
            extruder_plan.required_start_temperature_ = print_temp;
            extruder_used_in_meshgroup_[extruder] = true;
        }
        else
        {
            extruder_plan.required_start_temperature_ = initial_print_temp;
            extruder_plan.extrusion_temperature_ = print_temp;
        }
        assert(extruder_plan.required_start_temperature_ != -1 && "extruder_plan.required_start_temperature should now have been set");

        if (buffer_.size() == 1 && extruder_plan_idx == 0)
        { // the very first extruder plan of the current meshgroup
            for (size_t extruder_idx = 0; extruder_idx < scene.extruders.size(); extruder_idx++)
            { // set temperature of the first nozzle, turn other nozzles down
                const Settings& other_extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder_idx].settings_;
                if (scene.current_mesh_group == scene.mesh_groups.begin()) // First mesh group.
                {
                    // override values from GCodeExport::setInitialTemps
                    // the first used extruder should be set to the required temp in the start gcode
                    // see  FffGcodeWriter::processStartingCode
                    if (extruder_idx == extruder)
                    {
                        gcode_.setInitialTemp(extruder_idx, extruder_plan.extrusion_temperature_.value_or(extruder_plan.required_start_temperature_));
                    }
                    else
                    {
                        gcode_.setInitialTemp(extruder_idx, other_extruder_settings.get<Temperature>("material_standby_temperature"));
                    }
                }
                else
                {
                    if (extruder_idx != extruder)
                    { // TODO: do we need to do this?
                        extruder_plan.prev_extruder_standby_temp_ = other_extruder_settings.get<Temperature>("material_standby_temperature");
                    }
                }
            }
            continue;
        }

        insertTempCommands(extruder_plans, overall_extruder_plan_idx);
    }
}

} // namespace cura
