/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef LAYER_PLAN_BUFFER_H
#define LAYER_PLAN_BUFFER_H

#include <list>

#include "settings/settings.h"
#include "commandSocket.h"

#include "gcodeExport.h"
#include "gcodePlanner.h"
#include "MeshGroup.h"

#include "Preheat.h"

namespace cura 
{

class LayerPlanBuffer : SettingsMessenger
{
    GCodeExport& gcode;
    
    Preheat preheat_config; //!< the nozzle and material temperature settings for each extruder train.
    
    static constexpr unsigned int buffer_size = 5; // should be as low as possible while still allowing enough time in the buffer to heat up from standby temp to printing temp // TODO: hardcoded value
    // this value should be higher than 1, cause otherwise each layer is viewed as the first layer and no temp commands are inserted.

    static constexpr const double extra_preheat_time = 1.0; //!< Time to start heating earlier than computed to avoid accummulative discrepancy between actual heating times and computed ones.

public:
    std::list<GCodePlanner> buffer; //!< The buffer containing several layer plans (GCodePlanner) before writing them to gcode.
    
    LayerPlanBuffer(SettingsBaseVirtual* settings, GCodeExport& gcode)
    : SettingsMessenger(settings)
    , gcode(gcode)
    { }
    
    void setPreheatConfig(MeshGroup& settings)
    {
        preheat_config.setConfig(settings);
    }
    
    /*!
     * Place a new layer plan (GcodePlanner) by constructing it with the given arguments.
     * Pop back the oldest layer plan is it exceeds the buffer size and write it to gcode.
     */
    template<typename... Args>
    GCodePlanner& emplace_back(Args&&... constructor_args)
    {
        if (buffer.size() > 0)
        {
            insertPreheatCommands(); // insert preheat commands of the just completed layer plan (not the newly emplaced one)
        }
        buffer.emplace_back(constructor_args...);
        if (buffer.size() > buffer_size)
        {
            buffer.front().writeGCode(gcode);
            if (CommandSocket::isInstantiated())
            {
                CommandSocket::getInstance()->flushGcode();
            }
            buffer.pop_front();
        }
        return buffer.back();
    }
    
    /*!
     * Write all remaining layer plans (GCodePlanner) to gcode and empty the buffer.
     */
    void flush();
    
    /*!
     * Insert the preheat command for @p extruder into @p extruder_plan_before
     * 
     * \param extruder_plan_before An extruder plan before the extruder plan for which the temperature is computed, in which to insert the preheat command
     * \param time_before_extruder_plan_end The time before the end of the extruder plan, before which to insert the preheat command
     * \param extruder The extruder for which to set the temperature
     * \param temp The temperature of the preheat command
     */
    void insertPreheatCommand(ExtruderPlan& extruder_plan_before, double time_before_extruder_plan_end, int extruder, double temp);
    
    /*!
     * Compute the time needed to preheat, based either on the time the extruder has been on standby 
     * or based on the temp of the previous extruder plan which has the same extruder nr.
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param extruder_plan_idx The index of the extruder plan in \p extruder_plans for which to find the preheat time needed
     * \return the time needed to preheat and the temperature from which heating starts
     */
    Preheat::WarmUpResult timeBeforeExtruderPlanToInsert(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx);
    
    /*!
     * For two consecutive extruder plans of the same extruder (so on different layers), 
     * preheat the extruder to the temperature corresponding to the average flow of the second extruder plan.
     * 
     * The preheat commands are inserted such that the middle of the temperature change coincides with the start of the next layer.
     * 
     * \param prev_extruder_plan The former extruder plan (of the former layer)
     * \param extruder The extruder for which too set the temperature
     * \param required_temp The required temperature for the second extruder plan
     */
    void insertPreheatCommand_singleExtrusion(ExtruderPlan& prev_extruder_plan, int extruder, double required_temp);
    
    /*!
     * Insert the preheat command for an extruder plan which is preceded by an extruder plan with a different extruder.
     * Find the time window in which this extruder hasn't been used
     * and compute at what time the preheat command needs to be inserted.
     * Then insert the preheat command in the right extruder plan.
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param extruder_plan_idx The index of the extruder plan in \p extruder_plans for which to find the preheat time needed
     */
    void insertPreheatCommand_multiExtrusion(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx);
    
    /*!
     * Insert the preheat command for the extruder plan corersponding to @p extruder_plan_idx of the layer corresponding to @p layer_plan_idx.
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param extruder_plan_idx The index of the extruder plan in \p extruder_plans for which to generate the preheat command
     */
    void insertPreheatCommand(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx);

    /*!
     * Insert the preheat commands for the last added layer (unless that layer was empty)
     */
    void insertPreheatCommands();
private:
    /*!
     * Reconfigure the standby temperature during which we didn't print with this extruder.
     * Find the previous extruder plan with the same extruder as layers[layer_plan_idx].extruder_plans[extruder_plan_idx]
     * Set the prev_extruder_standby_temp in the next extruder plan
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param extruder_plan_idx The index of the extruder plan in \p extruder_plans before which to reconfigure the standby temperature
     * \param standby_temp The temperature to which to cool down when the extruder is in standby mode.
     */
    void handleStandbyTemp(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx, double standby_temp);
};



} // namespace cura

#endif // LAYER_PLAN_BUFFER_H