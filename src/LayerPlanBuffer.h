#ifndef LAYER_PLAN_BUFFER_H
#define LAYER_PLAN_BUFFER_H

#include <list>

#include "settings.h"
#include "commandSocket.h"

#include "gcodeExport.h"
#include "gcodePlanner.h"

namespace cura 
{

class LayerPlanBuffer : SettingsBaseVirtual
{
    CommandSocket* command_socket;
    
    GCodeExport& gcode;
    
public:
    std::list<GCodePlanner> buffer;
    
    LayerPlanBuffer(SettingsBaseVirtual* settings, CommandSocket* command_socket, GCodeExport& gcode)
    : SettingsBaseVirtual(settings)
    , command_socket(command_socket)
    , gcode(gcode)
    { }
    
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
    void flush()
    {
        for (GCodePlanner& layer_plan : buffer)
        {
            layer_plan.writeGCode(gcode, getSettingBoolean("cool_lift_head"), layer_plan.getLayerNr() > 0 ? getSettingInMicrons("layer_height") : getSettingInMicrons("layer_height_0"));
            if (command_socket)
                command_socket->sendGCodeLayer();
        }
    }
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