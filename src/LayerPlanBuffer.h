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
    
    class iterator
    {
        std::vector<GCodePlanner*>& layers;
    public:
        unsigned int layer_plan_idx;
        unsigned int extruder_plan_idx;
        iterator(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
        : layers(layers)
        , layer_plan_idx(layer_plan_idx)
        , extruder_plan_idx(extruder_plan_idx)
        { }
        
        iterator& operator=(const iterator& b)
        {
//             layers = b.layers;
            layer_plan_idx = b.layer_plan_idx;
            extruder_plan_idx = b.extruder_plan_idx;
            return *this;
        }
        
        iterator operator++(int) // int so that we declare postfix
        { 
            extruder_plan_idx++;
            if (extruder_plan_idx == layers[layer_plan_idx]->extruder_plans.size())
            {
                extruder_plan_idx = 0;
                layer_plan_idx++;    
            }
            return *this; 
        }
        iterator operator--(int) // int so that we declare postfix
        {
            if (extruder_plan_idx == 0)
            {
                layer_plan_idx--;
                if (int(layer_plan_idx) >= 0)
                {
                    extruder_plan_idx = layers[layer_plan_idx]->extruder_plans.size();
                } // otherwise keep arbitrary extruder_plan_idx
            }
            extruder_plan_idx--;
            return *this;
        }
        ExtruderPlan& operator*()
        {
            return layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
        }
        ExtruderPlan* operator->()
        {
            return &layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
        }
        bool operator==(const iterator b) 
        { 
            if (int(layer_plan_idx) > 0)
            {
                return layer_plan_idx == b.layer_plan_idx && extruder_plan_idx == b.extruder_plan_idx && &layers == &b.layers; 
            }
            else 
            {
                return int(layer_plan_idx) < 0 && int(b.layer_plan_idx) < 0;
            }
        }
        bool operator!=(const iterator b) { return !(*this == b); }
    };
    
    iterator begin(std::vector<GCodePlanner*>& layers)
    {
        return iterator(layers, 0, 0);
    }
    
    
    iterator start(std::vector<GCodePlanner*>& layers)
    {
        return iterator(layers, -1, 0);
    }
    
    iterator end(std::vector<GCodePlanner*>& layers)
    {
        return iterator(layers, buffer.size(), 0);
    }
    
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
                std::cerr << "INSERT " << temp << "*C for nozzle " << extruder <<" at " << path_idx << std::endl;
                // TODO: do the actual insert!
                return;
            }
        }
        
        std::cerr << "INSERT " << temp << "*C for nozzle " << extruder <<" at " << 0 << std::endl;
        // TODO: do the actual insert!
        logError("Warning! Couldn't preheat in time.\n");
    }
    
    /*!
     * 
     * \param layers The layers of the buffer, moved to a temporary vector (from lower to upper layers)
     * \param layer_plan_idx The index of the layer plan for which to generate a preheat command
     * \param extruder_plan_idx The index of the extruder plan in the layer corresponding to @p layer_plan_idx for which to generate the preheat command
     */
    void insertPreheatCommand_OLD(std::vector<GCodePlanner*>& layers, unsigned int layer_plan_idx, unsigned int extruder_plan_idx)
    {
        ExtruderPlan& extruder_plan = layers[layer_plan_idx]->extruder_plans[extruder_plan_idx];
        
        if (extruder_plan.preheat_command_inserted)
        {
            return;
        }
        
        int extruder = extruder_plan.extruder;

        TimeMaterialEstimates extruder_plan_estimates = extruder_plan.estimates;

        if (layer_plan_idx == 0 && extruder_plan_idx == 0)
        { // for the very first extruder plan on the first layer (only after the first layer will the buffer consist merely of one layer)
            double time_after_extruder_plan_start = 0;
            double avg_flow = extruder_plan_estimates.material / extruder_plan_estimates.extrude_time;
            std::cerr << "AVERAGE FLOW = " << avg_flow << std::endl;
            double temp = preheat_config.getTemp(extruder, avg_flow);
            insertPreheatCommand(extruder_plan, time_after_extruder_plan_start, extruder, temp);
            return;
        }
        
        iterator last_extruder_plan_different_extruder(layers, layer_plan_idx, extruder_plan_idx);
        last_extruder_plan_different_extruder--;

        // check whether this extruder plan needs to be latched with the last one of the prev layer

        if (extruder_plan_idx == 0 && layer_plan_idx > 0)
        {
            GCodePlanner& prev_layer = *layers[layer_plan_idx - 1];
            ExtruderPlan& prev_extruder_plan = prev_layer.extruder_plans.back();
            if (prev_layer.extruder_plans.size() == 1 && layers[layer_plan_idx]->extruder_plans.size() == 1) 
            { // two full single extrusion layers...
                
                double time_after_extruder_plan_start = 0000; // prev_extruder_plan.estimates.extrude_time + prev_extruder_plan.estimates.travel_time - preheat_config.timeBeforeEndToInsertPreheatCommand_warmUp();
                double avg_flow = extruder_plan_estimates.material / extruder_plan_estimates.extrude_time;
                std::cerr << "AVERAGE FLOW = " << avg_flow << std::endl;
                double temp = preheat_config.getTemp(extruder, avg_flow);
                insertPreheatCommand(prev_extruder_plan, time_after_extruder_plan_start, extruder, temp);
                return;
            }
            if (prev_extruder_plan.extruder == extruder_plan.extruder)
            {
                extruder_plan_estimates += prev_extruder_plan.estimates;
                last_extruder_plan_different_extruder.layer_plan_idx = layer_plan_idx - 1; 
                last_extruder_plan_different_extruder.extruder_plan_idx = layers[layer_plan_idx - 1]->extruder_plans.size() - 1; // start from the one before the last extruder plan of the previous layer
                last_extruder_plan_different_extruder--;
            }
        }

        double avg_flow = extruder_plan_estimates.material / extruder_plan_estimates.extrude_time;
        std::cerr << "AVERAGE FLOW = " << avg_flow << std::endl;

        // find time in between two extruder plans of the same extruder
        double time_in_between = 0.0;
        iterator prev_extruder_plan_same_extruder = begin(layers); // the reverse-iterator equivalent of end()
        for (iterator extruder_plan_before_it = last_extruder_plan_different_extruder; extruder_plan_before_it != start(layers); extruder_plan_before_it--)
        {
            if (extruder_plan_before_it->extruder == extruder)
            {
                prev_extruder_plan_same_extruder = extruder_plan_before_it;
                break;
            }
            time_in_between += extruder_plan_before_it->estimates.extrude_time + extruder_plan_before_it->estimates.travel_time;
//             if (extruder_plan_before_it == begin(layers))
//             {
//                 break;
//             }
        }
        
        double temp = preheat_config.getTemp(extruder, avg_flow);
        double time_before_extruder_plan_end = preheat_config.timeBeforeEndToInsertPreheatCommand_coolDownWarmUp(time_in_between, extruder, temp);
        
        // insert preheat command in the right place in the right extruder plan
        for (iterator extruder_plan_before_it = last_extruder_plan_different_extruder; extruder_plan_before_it != prev_extruder_plan_same_extruder && extruder_plan_before_it != start(layers); extruder_plan_before_it--)
        {
            double extruder_plan_time = extruder_plan_before_it->estimates.extrude_time + extruder_plan_before_it->estimates.travel_time;
            if (extruder_plan_time > time_before_extruder_plan_end)
            {
                insertPreheatCommand(*extruder_plan_before_it, extruder_plan_time - time_before_extruder_plan_end, extruder, temp);
                return;
            }
            time_before_extruder_plan_end -= extruder_plan_time;
        }
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
        
        /*
        std::vector<GCodePlanner*> layers;
        for (GCodePlanner& layer_plan : buffer)
        {
            // TODO: disregards empty layers!
            layers.push_back(&layer_plan);
        }
        
        
        unsigned int last_layer_idx = layers.size() - 1;
        // process all extruder plans of the last layer except the last one, and process the last one of the previous layer
        for (unsigned int extruder_plan_idx = 0; extruder_plan_idx < layers[last_layer_idx]->extruder_plans.size() - 1; ++extruder_plan_idx)
        {
            insertPreheatCommand(layers, last_layer_idx, extruder_plan_idx);
        }
        unsigned int second_last_layer_idx = last_layer_idx - 1;
        if (int(second_last_layer_idx) >= 0)
        {
            insertPreheatCommand(layers, second_last_layer_idx, layers[second_last_layer_idx]->extruder_plans.size() - 1);
        }
        */
    }
    /*
    class iterator
    {
    public:
        std::list<GCodePlanner>::iterator layer_plan_it;
        unsigned int path_idx;
        iterator(std::list<GCodePlanner>::iterator layer_plan_it, unsigned int path_idx)
        : layer_plan_it(layer_plan_it)
        , path_idx(path_idx)
        { }
        
        iterator operator++(int) // int so that we declare postfix
        { 
            path_idx++;
            if (path_idx == layer_plan_it->paths.size())
            {
                path_idx = 0;
                layer_plan_it++;    
            }
            return *this; 
        }
        iterator operator--(int) // int so that we declare postfix
        {
            if (path_idx == 0)
            {
                layer_plan_it--;
                path_idx = layer_plan_it->paths.size();
            }
            path_idx--;
            return *this;
        }
        GCodePath& operator*()
        {
            return layer_plan_it->paths[path_idx];
        }
        GCodePath* operator->()
        {
            return &layer_plan_it->paths[path_idx];
        }
        bool operator==(const iterator b) { return layer_plan_it == b.layer_plan_it && path_idx == b.path_idx; }
        bool operator!=(const iterator b) { return !(*this == b); }
    };
    
    iterator begin()
    {
        return iterator(buffer.begin(), 0);
    }
    
    iterator end()
    {
        return iterator(buffer.end(), 0);
    }
    */
    /*
    struct ExtruderPlanZone
    {
        iterator start;
        iterator end;
        int extruder;
        
        ExtruderPlanZone(LayerPlanBuffer& buffer, int extruder, iterator start)
        : start(start)
        , end(buffer.end())
        , extruder(extruder)
        { }
    };
    */
    /*!
     * Retrieves the zones of the plan which are printed with a single extruder
     * \param zones (output) The resulting zones
     * \return returns the index of the first zone in the current layer, i.e. the middle (or first) layer in the buffer
     */
    /*
    int findExtruderPlanZones(std::vector<ExtruderPlanZone>& zones)
    {
        assert(buffer.size() >= 2);
        std::list<GCodePlanner>::iterator next_layer = --buffer.end();
        std::list<GCodePlanner>::iterator this_layer = --next_layer;
        int first_zone_in_this_layer = -1;
        zones.emplace_back(*this, begin()->extruder, begin());
        for (iterator it = begin(); it != end(); it++)
        {
            if (it->getExtrusionMM3perMM() == 0 && it->extruder != zones.back().extruder)
            {
                zones.back().end = it;
                zones.emplace_back(*this, it->extruder, it);
                if (it.layer_plan_it == this_layer)
                {
                    if (first_zone_in_this_layer == -1)
                    {
                        first_zone_in_this_layer = zones.size() - 1;
                    }
                }
                else if (it.layer_plan_it == next_layer)
                {
                    break;
                }
            }
        }
        
    }
    
    void insertPreheatCommands()
    {
        std::vector<ExtruderPlanZone> zones;
        int first_zone_in_this_layer = findExtruderPlanZones(zones);
        for (unsigned int zone_idx = static_cast<unsigned int>(first_zone_in_this_layer); zone_idx < zones.size(); zone_idx++)
        {
            int extruder = zones[zone_idx].extruder;
            
            // TODO: find time and material of this zone
            
            
            unsigned int prev_zone_idx;
            for (prev_zone_idx = zone_idx - 1; int(prev_zone_idx) >= 0; prev_zone_idx--)
            {
                if (zones[prev_zone_idx].extruder == extruder)
                {
                    break;
                }
            }
            unsigned int start_zone_other_extruders = prev_zone_idx + 1;
            
            // TODO: add up all times of the extruder zones from [start_zone_other_extruders] to [zone_idx]
        }
    }
    */
    /*!
     * Inserts pre-heating commands for the middle layer
     */
    /*
    void insertPreheatCommand()
    {
        // temp
        
        //
        GCodePlanner& prev_layer = buffer.front();
        GCodePlanner& middle_layer = *(buffer.begin()++);
        GCodePlanner& next_layer = buffer.back();
        
        unsigned int current_extruder = middle_layer.paths[0].extruder; // first extruder on the middle layer in the buffer
        
        unsigned int current_extruder_start_idx = 0;
        GCodePlanner* current_extruder_start_layer = &buffer.front();
        
        for (unsigned int path_idx = prev_layer.paths.size() - 1; int(path_idx) > 0; path_idx--)
        {
            if (prev_layers.paths[path_idx].extruder != current_extruder)
            {
                if (path_idx == prev_layer.paths.size() - 1)
                {
                    break;
                }
                else 
                {
                    current_extruder_start_idx = path_idx + 1;
                    current_extruder_start_layer = &prev_layer;
                    break;
                }
            }
        }
        unsigned int current_extruder_end_idx = -1;
        GCodePlanner* current_extruder_end_layer = &buffer.front();
        
        for (unsigned int path_idx = 0; path_idx < middle_layer.paths.size(); path_idx++)
        {
            if (middle_layer
            
        }
        
        
        
        
        unsigned int next_extruder_end_idx = 0;
        GCodePlanner* next_extruder_end_layer = &buffer.front();
        
        
    }
    */


};



} // namespace cura

#endif // LAYER_PLAN_BUFFER_H