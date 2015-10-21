#ifndef LAYER_PLAN_BUFFER_H
#define LAYER_PLAN_BUFFER_H

#include <list>

#include "settings.h"
#include "commandSocket.h"

#include "gcodeExport.h"
#include "gcodePlanner.h"
#include "MeshGroup.h"

#include "Preheat.h"

namespace cura 
{

class LayerPlanBuffer : SettingsMessenger
{
    CommandSocket* command_socket;
    
    GCodeExport& gcode;
    
    Preheat preheat_config;
    
public:
    std::list<GCodePlanner> buffer;
    
    LayerPlanBuffer(SettingsBaseVirtual* settings, CommandSocket* command_socket, GCodeExport& gcode)
    : SettingsMessenger(settings)
    , command_socket(command_socket)
    , gcode(gcode)
    { }
    
    void setPreheatConfig(MeshGroup& settings)
    {
        preheat_config.setConfig(settings);
    }
    
    template<typename... Args>
    GCodePlanner& emplace_back(Args&&... constructor_args)
    {
        if (buffer.size() > 0)
        {
            insertPreheatCommands();
        }
        buffer.emplace_back(constructor_args...);
        if (buffer.size() > 2)
        {
            buffer.front().writeGCode(gcode, getSettingBoolean("cool_lift_head"), buffer.front().getLayerNr() > 0 ? getSettingInMicrons("layer_height") : getSettingInMicrons("layer_height_0"));
            if (command_socket)
                command_socket->sendGCodeLayer();
            buffer.pop_front();
        }
        return buffer.back();
    }
    
    void flush()
    {
        if (buffer.size() > 0)
        {
            insertPreheatCommands();
        }
        for (GCodePlanner& layer_plan : buffer)
        {
            layer_plan.processFanSpeedAndMinimalLayerTime();
            layer_plan.writeGCode(gcode, getSettingBoolean("cool_lift_head"), layer_plan.getLayerNr() > 0 ? getSettingInMicrons("layer_height") : getSettingInMicrons("layer_height_0"));
            if (command_socket)
                command_socket->sendGCodeLayer();
        }
    }
    
    /*!
     * 
     * \param extruder_plan_before An extruder plan before the extruder plan for which the temperature is computed, in which to insert the preheat command
     * \param time_after_extruder_plan_start The time after the start of the extruder plan, before which to insert the preheat command
     * \param extruder The extruder for which to set the temperature
     * \param temp The temperature of the preheat command
     */
    void insertPreheatCommand(ExtruderPlan& extruder_plan_before, double time_after_extruder_plan_start, int extruder, double temp)
    {
        double acc_time = 0.0;
        for (unsigned int path_idx = 0; path_idx < extruder_plan_before.paths.size(); path_idx++)
        {
            GCodePath& path = extruder_plan_before.paths[path_idx];
            acc_time += path.estimates.getTotalTime();
            if (acc_time > time_after_extruder_plan_start)
            {
//                 logError("Inserting %f\t seconds too early!\n", acc_time - time_after_extruder_plan_start);
//                 std::cerr << "INSERT " << temp << "*C for nozzle " << extruder <<" at " << path_idx << " of " << extruder_plan_before.paths.size() << std::endl;
                // TODO: do the actual insert!
                return;
            }
        }
        
//         std::cerr << "INSERT " << temp << "*C for nozzle " << extruder <<" at end " << std::endl;
        // TODO: do the actual insert!
        // = special insert after all extruder plans
    }
    
    double getTimeUntilSameExtruderIsMet(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
    {
        int extruder = layers[layer_plan_idx]->extruder_plans[extruder_plan_idx].extruder;
        unsigned int extruder_plan_before_idx = extruder_plan_idx - 1;
        bool first_it = true;
        double total_time = 0.0;
        for (unsigned int layer_idx = layers.size() - 1; int(layer_idx) >= 0; layer_idx++)
        {
            GCodePlanner& layer = *layers[layer_idx];
            if (first_it)
            {
                extruder_plan_before_idx = layer.extruder_plans.size() - 1;
                first_it = false;
            }
            for ( ; int(extruder_plan_before_idx) >= 0; extruder_plan_before_idx--)
            {
                ExtruderPlan& extruder_plan = layer.extruder_plans[extruder_plan_before_idx];
                if (extruder_plan.extruder == extruder)
                {
                    return total_time;
                }
                total_time += extruder_plan.estimates.getTotalTime();
            }
        }
        return total_time; // in case there is no previous extruder plan with the same extruder nr in the buffer
    }
    
    /*!
     * 
     * \param layers The layers of the buffer, moved to a temporary vector (from lower to upper layers)
     * \param layer_plan_idx The index of the layer plan for which to generate a preheat command
     * \param extruder_plan_idx The index of the extruder plan in the layer corresponding to @p layer_plan_idx for which to generate the preheat command
     */
    void insertPreheatCommand(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
    {
        ExtruderPlan& extruder_plan = layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
        int extruder = extruder_plan.extruder;
        double required_temp = extruder_plan.required_temp;
        
        
        ExtruderPlan* prev_extruder_plan = nullptr;
        if (extruder_plan_idx == 0)
        {
            if (layer_plan_idx == 0)
            {
                insertPreheatCommand(extruder_plan, 0, extruder, required_temp);
                return;
            }
            prev_extruder_plan = &layers[layer_plan_idx - 1]->extruder_plans.back();
        }
        else 
        {
            prev_extruder_plan = &layers[layer_plan_idx]->extruder_plans[extruder_plan_idx - 1];
        }
        
        if (prev_extruder_plan->extruder == extruder)
        {
            // time_before_extruder_plan_end is halved, so that at the layer change the temperature will be half way betewen the two requested temperatures
            double time_before_extruder_plan_end = 0.5 * preheat_config.timeBeforeEndToInsertPreheatCommand_warmUp(prev_extruder_plan->required_temp, extruder, required_temp);
            double time_after_extruder_plan_start = prev_extruder_plan->estimates.getTotalTime() - time_before_extruder_plan_end;
            if (time_after_extruder_plan_start < 0)
            {
                time_after_extruder_plan_start = 0; // don't override the extruder plan with same extruder of the previous layer
            }
                
            insertPreheatCommand(*prev_extruder_plan, time_after_extruder_plan_start, extruder, required_temp);
            return;
        }
        // else 
        double time_until_same_extruder_is_met = getTimeUntilSameExtruderIsMet(layers, layer_plan_idx, extruder_plan_idx);
        
        double time_before_extruder_plan_to_insert = preheat_config.timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(time_until_same_extruder_is_met, extruder, required_temp);
        
        unsigned int extruder_plan_before_idx = extruder_plan_idx - 1;
        bool first_it = true;
        for (unsigned int layer_idx = layers.size() - 1; int(layer_idx) >= 0; layer_idx++)
        {
            GCodePlanner& layer = *layers[layer_idx];
            if (first_it)
            {
                extruder_plan_before_idx = layer.extruder_plans.size() - 1;
                first_it = false;
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
        }
    }

    void insertPreheatCommands()
    {
        std::vector<GCodePlanner*> layers;
        for (GCodePlanner& layer_plan : buffer)
        {
            // TODO: disregards empty layers!
            layers.push_back(&layer_plan);
        }
        for (unsigned int layer_idx = 0; layer_idx < layers.size(); layer_idx++)
        {
            GCodePlanner& layer_plan = *layers[layer_idx];
            for (unsigned int extruder_plan_idx = 0; extruder_plan_idx < layer_plan.extruder_plans.size(); extruder_plan_idx++)
            {
                ExtruderPlan& extruder_plan = layer_plan.extruder_plans[extruder_plan_idx];
                if (extruder_plan.required_temp == 0 || extruder_plan.preheat_command_inserted)
                {
                    continue;
                }
                double time = extruder_plan.estimates.getTotalTime();
                if (time <= 0.0)
                {
                    continue;
                }
                double avg_flow = extruder_plan.estimates.material / time; // TODO: subtract retracted travel time
                extruder_plan.required_temp = preheat_config.getTemp(extruder_plan.extruder, avg_flow);
                
                insertPreheatCommand(layers, layer_idx, extruder_plan_idx);
            }
        }
    }
};



} // namespace cura

#endif // LAYER_PLAN_BUFFER_H