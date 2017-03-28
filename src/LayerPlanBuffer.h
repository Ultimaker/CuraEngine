/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef LAYER_PLAN_BUFFER_H
#define LAYER_PLAN_BUFFER_H

#include <list>

#include "settings/settings.h"
#include "commandSocket.h"

#include "gcodeExport.h"
#include "LayerPlan.h"
#include "MeshGroup.h"

#include "Preheat.h"

namespace cura 
{

/*!
 * Class for buffering multiple layer plans (\ref LayerPlan) / extruder plans within those layer plans, so that temperature commands can be inserted in earlier layer plans.
 * 
 * This class handles where to insert temperature commands for:
 * - initial layer temperature
 * - flow dependent temperature
 * - starting to heat up from the standby temperature
 * - initial printing temperature | printing temperature | final printing temperature
 * 
 * \image html assets/precool.png "Temperature Regulation" width=10cm
 * \image latex assets/precool.png "Temperature Regulation" width=10cm
 * 
 */
class LayerPlanBuffer : SettingsMessenger
{
    GCodeExport& gcode;
    
    Preheat preheat_config; //!< the nozzle and material temperature settings for each extruder train.
    
    static constexpr unsigned int buffer_size = 5; // should be as low as possible while still allowing enough time in the buffer to heat up from standby temp to printing temp // TODO: hardcoded value
    // this value should be higher than 1, cause otherwise each layer is viewed as the first layer and no temp commands are inserted.

    static constexpr const double extra_preheat_time = 1.0; //!< Time to start heating earlier than computed to avoid accummulative discrepancy between actual heating times and computed ones.

    std::vector<bool> extruder_used_in_meshgroup; //!< For each extruder whether it has already been planned once in this meshgroup. This is used to see whether we should heat to the initial_print_temp or to the extrusion_temperature

    /*!
     * The buffer containing several layer plans (LayerPlan) before writing them to gcode.
     * 
     * The front is the lowest/oldest layer.
     * The back is the highest/newest layer.
     */
    std::list<LayerPlan*> buffer;
public:
    LayerPlanBuffer(SettingsBaseVirtual* settings, GCodeExport& gcode)
    : SettingsMessenger(settings)
    , gcode(gcode)
    , extruder_used_in_meshgroup(MAX_EXTRUDERS, false)
    { }

    void setPreheatConfig(MeshGroup& settings);

    /*!
     * Push a new layer plan into the buffer
     */
    void push(LayerPlan& layer_plan);

    /*!
     * Push a new layer onto the buffer and handle the buffer.
     * Write a layer to gcode if it is popped out of the buffer.
     * 
     * \param layer_plan The layer to handle
     * \param gcode The exporter with which to write a layer to gcode if the buffer is too large after pushing the new layer.
     */
    void handle(LayerPlan& layer_plan, GCodeExport& gcode);

    /*!
     * Process all layers in the buffer
     * This inserts the temperature commands to start warming for a given layer in earlier layers;
     * the fan speeds and layer time settings of the most recently pushed layer are processed;
     * the correctly combing travel move between the last added layer and the layer before is added.
     * 
     * Pop out the earliest layer in the buffer if the buffer size is exceeded
     * \return A nullptr or the popped gcode_layer
     */
    LayerPlan* processBuffer();

    /*!
     * Write all remaining layer plans (LayerPlan) to gcode and empty the buffer.
     */
    void flush();

private:
    /*!
     * Add the travel move to properly travel from the end location of the previous layer to the starting location of the next
     * 
     * \param prev_layer The layer before the just added layer, to which to add the combing travel move.
     * \param newest_layer The newly added layer, with a non-combing travel move as first path.
     */
    void addConnectingTravelMove(LayerPlan* prev_layer, const LayerPlan* newest_layer);

    /*!
     * Apply fan speed and correct extrusion flow for minimal layer time settings of the last layer plan in the buffer.
     */
    void processFanSpeedLayerTime();

    /*!
     * Insert a preheat command for @p extruder into @p extruder_plan_before
     * 
     * \param extruder_plan_before An extruder plan before the extruder plan for which the temperature is computed, in which to insert the preheat command
     * \param time_before_extruder_plan_end The time before the end of the extruder plan, before which to insert the preheat command
     * \param extruder The extruder for which to set the temperature
     * \param temp The temperature of the preheat command
     */
    void insertPreheatCommand(ExtruderPlan& extruder_plan_before, double time_before_extruder_plan_end, int extruder, double temp);

    /*!
     * Compute the time needed to preheat from standby to required (initial) printing temperature at the start of an extruder plan,
     * based on the time the extruder has been on standby.
     * 
     * Also computes the temperature to which we cool before starting to heat agian.
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param extruder_plan_idx The index of the extruder plan in \p extruder_plans for which to find the preheat time needed
     * \return the time needed to preheat and the temperature from which heating starts
     */
    Preheat::WarmUpResult computeStandbyTempPlan(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx);

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
     * Insert temperature commands related to the extruder plan corersponding to @p extruder_plan_idx
     * and the extruder plan before:
     * 
     * In case the extruder plan before has the same extruder:
     * - gradually change printing temperature around the layer change (\ref LayerPlanBuffer::insertPreheatCommand_singleExtrusion)
     * 
     * In case the previous extruder plan is a different extruder
     * - insert preheat command from standby to initial temp in the extruder plan(s) before (\ref LayerPlanBuffer::insertPreheatCommand_multiExtrusion)
     * - insert the final print temp command of the previous extruder plan (\ref LayerPlanBuffer::insertFinalPrintTempCommand)
     * - insert the normal extrusion temp command for the current extruder plan (\ref LayerPlanBuffer::insertPrintTempCommand)
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param extruder_plan_idx The index of the extruder plan in \p extruder_plans for which to generate the preheat command
     */
    void insertTempCommands(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx);

    /*!
     * Insert the temperature command to heat from the initial print temperature to the printing temperature
     * 
     * The temperature command is insert at the start of the very first extrusion move
     * 
     * \param extruder_plan The extruder plan in which to insert the heat up command
     */
    void insertPrintTempCommand(ExtruderPlan& extruder_plan);

    /*!
     * Insert the temp command to start cooling from the printing temperature to the final print temp
     * 
     * The print temp is inserted before the last extrusion move of the extruder plan corresponding to \p last_extruder_plan_idx
     * 
     * The command is inserted at a timed offset before the end of the last extrusion move
     * 
     * \param extruder_plans The extruder plans in the buffer, moved to a temporary vector (from lower to upper layers)
     * \param last_extruder_plan_idx The index of the last extruder plan in \p extruder_plans with the same extruder as previous extruder plans
     */
    void insertFinalPrintTempCommand(std::vector<ExtruderPlan*>& extruder_plans, unsigned int last_extruder_plan_idx);

    /*!
     * Insert the preheat commands for the last added layer (unless that layer was empty)
     */
    void insertTempCommands();

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