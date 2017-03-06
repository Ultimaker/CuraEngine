#ifndef GCODE_WRITER_H
#define GCODE_WRITER_H


#include <fstream>
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/NoCopy.h"
#include "utils/polygonUtils.h"
#include "sliceDataStorage.h"
#include "raft.h"
#include "infill.h"
#include "bridge.h"
#include "pathOrderOptimizer.h"
#include "gcodePlanner.h"
#include "gcodeExport.h"
#include "commandSocket.h"
#include "PrimeTower.h"
#include "FanSpeedLayerTime.h"
#include "PrintFeature.h"


#include "LayerPlanBuffer.h"


namespace cura 
{

/*!
 * Secondary stage in Fused Filament Fabrication processing: The generated polygons are used in the gcode generation.
 * Some polygons in the SliceDataStorage signify areas which are to be filled with parallel lines, 
 * while other polygons signify the contours which should be printed.
 * 
 * The main function of this class is FffGcodeWriter::writeGCode().
 */
class FffGcodeWriter : public SettingsMessenger, NoCopy
{
    friend class FffProcessor; // cause WireFrame2Gcode uses the member [gcode] (TODO)
private:
    int max_object_height; //!< The maximal height of all previously sliced meshgroups, used to avoid collision when moving to the next meshgroup to print.

    /*
     * Buffer for all layer plans (of type GCodePlanner)
     * 
     * The layer plans are buffered so that we can start heating up a nozzle several layers before it needs to be used.
     * Another reason is to perform Auto Temperature.
     */
    LayerPlanBuffer layer_plan_buffer; 

    /*!
     * The class holding the current state of the gcode being written.
     * 
     * It holds information such as the last written position etc.
     */
    GCodeExport gcode;

    /*!
     * The gcode file to write to when using CuraEngine as command line tool.
     */
    std::ofstream output_file;

    /*!
     * Whether the skirt or brim polygons have been processed into planned paths
     * for each extruder train.
     */
    bool skirt_brim_is_processed[MAX_EXTRUDERS];

    std::vector<std::vector<unsigned int>> mesh_order_per_extruder; //!< For each extruder, the cyclic order of the meshes (the first element is not the starting element per se)

    /*!
     * For each extruder whether priming has already been planned
     */
    bool extruder_prime_is_planned[MAX_EXTRUDERS];

    std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder; //!< The settings used relating to minimal layer time and fan speeds. Configured for each extruder.

    Point last_position_planned; //!< The position of the head before planning the next layer
    int current_extruder_planned; //!< The extruder train in use before planning the next layer
    bool is_inside_mesh_layer_part; //!< Whether the last position was inside a layer part (used in combing)
public:
    FffGcodeWriter(SettingsBase* settings_)
    : SettingsMessenger(settings_)
    , max_object_height(0)
    , layer_plan_buffer(this, gcode)
    , extruder_prime_is_planned {} // initialize all values in array with [false]
    , last_position_planned(no_point)
    , current_extruder_planned(0) // changed somewhere early in FffGcodeWriter::writeGCode
    , is_inside_mesh_layer_part(false)
    {
    }

    /*!
     * Set the target to write gcode to: to a file.
     * 
     * Used when CuraEngine is used as command line tool.
     * 
     * \param filename The filename of the file to which to write the gcode.
     */
    bool setTargetFile(const char* filename)
    {
        output_file.open(filename);
        if (output_file.is_open())
        {
            gcode.setOutputStream(&output_file);
            return true;
        }
        return false;
    }

    /*!
     * Set the target to write gcode to: an output stream.
     * 
     * Used when CuraEngine is NOT used as command line tool.
     * 
     * \param stream The stream to write gcode to.
     */
    void setTargetStream(std::ostream* stream)
    {
        gcode.setOutputStream(stream);
    }

    /*!
     * Get the total extruded volume for a specific extruder in mm^3
     * 
     * Retractions and unretractions don't contribute to this.
     * 
     * \param extruder_nr The extruder number for which to get the total netto extruded volume
     * \return total filament printed in mm^3
     */
    double getTotalFilamentUsed(int extruder_nr)
    {
        return gcode.getTotalFilamentUsed(extruder_nr);
    }

    /*!
     * Get the total estimated print time in seconds
     * 
     * \return total print time in seconds
     */
    double getTotalPrintTime()
    {
        return gcode.getTotalPrintTime();
    }

    /*!
     * Write all the gcode for the current meshgroup.
     * This is the primary function of this class.
     * 
     * \param[in] storage The data storage from which to get the polygons to print and the areas to fill.
     * \param timeKeeper The stop watch to see how long it takes for each of the stages in the slicing process.
     */
    void writeGCode(SliceDataStorage& storage, TimeKeeper& timeKeeper);

private:
    /*!
     * Set the FffGcodeWriter::fan_speed_layer_time_settings by retrieving all settings from the global/per-meshgroup settings.
     * 
     * \param[out] storage The data storage to which to save the configuration
     */
    void setConfigFanSpeedLayerTime(SliceDataStorage& storage);

    /*!
     * Create and set the SliceDataStorage::coasting_config for each extruder.
     * 
     * \param[out] storage The data storage to which to save the configuration
     */
    void setConfigCoasting(SliceDataStorage& storage);

    /*!
     * Set the retraction config globally, per extruder and per mesh.
     * 
     * \param[out] storage The data storage to which to save the configurations
     */
    void setConfigRetraction(SliceDataStorage& storage);
    
    /*!
     * Initialize the GcodePathConfig config parameters which don't change over
     * all layers, for each feature.
     * 
     * The features are: skirt or brim, support and for each mesh: outer wall,
     * inner walls, skin, infill (and combined infill).
     * 
     * \param[out] storage The data storage to which to save the configurations.
     */
    void initConfigs(SliceDataStorage& storage);

    /*!
     * Get the extruder with which to start the print.
     * 
     * Generally this is the adhesion_extruder_nr, but in case the platform adhesion type is none,
     * the extruder with lowest number which is used on the first layer is used as initial extruder.
     * 
     * \param[in] storage where to get settings from.
     */
    unsigned int getStartExtruder(SliceDataStorage& storage);

    /*!
     * Set temperatures and perform initial priming.
     * 
     * Write a stub header if CuraEngine is in command line tool mode. (Cause writing the header afterwards would entail moving all gcode down.)
     * 
     * \param[in] storage where the slice data is stored.
     * \param[in] start_extruder_nr The extruder with which to start the print.
     */
    void processStartingCode(SliceDataStorage& storage, const unsigned int start_extruder_nr);

    /*!
     * Move up and over the already printed meshgroups to print the next meshgroup.
     * 
     * \param[in] storage where the slice data is stored.
     */
    void processNextMeshGroupCode(SliceDataStorage& storage);
    
    /*!
     * Add raft layer plans onto the FffGcodeWriter::layer_plan_buffer
     * 
     * \param[in,out] storage where the slice data is stored.
     * \param total_layers The total number of layers.
     */
    void processRaft(SliceDataStorage& storage, unsigned int total_layers);

    /*!
     * Convert the polygon data of a layer into a layer plan on the FffGcodeWriter::layer_plan_buffer
     * 
     * In case of negative layer numbers, create layers only containing the data from
     * the helper parts (support etc) to fill up the gap between the raft and the model.
     * 
     * \param[in] storage where the slice data is stored.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param total_layers The total number of layers.
     */
    void processLayer(SliceDataStorage& storage, int layer_nr, unsigned int total_layers);

    /*!
     * Whether the extruders need to be primed separately just before they are used.
     * 
     * \return whether the extruders need to be primed separately just before they are used
     */
    bool getExtrudersNeedPrimeDuringFirstLayer();

    /*!
     * Plan priming of all used extruders which haven't been primed yet
     * \param[in] storage where the slice data is stored.
     * \param layer_plan The initial planning of the g-code of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void ensureAllExtrudersArePrimed(SliceDataStorage& storage, GCodePlanner& layer_plan, const int layer_nr);

    /*!
     * Add the skirt or the brim to the layer plan \p gcodeLayer.
     * 
     * \param Storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the g-code of the layer.
     * \param extruder_nr The extruder train for which to process the skirt or
     * brim.
     */
    void processSkirtBrim(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int extruder_nr);
    
    /*!
     * Adds the ooze shield to the layer plan \p gcodeLayer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processOozeShield(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    /*!
     * Adds the draft protection screen to the layer plan \p gcodeLayer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processDraftShield(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);

    /*!
     * Calculate in which order to plan the extruders
     * 
     * \param[in] storage where the slice data is stored.
     * \param current_extruder The current extruder with which we last printed
     * \return A vector of pairs of extruder numbers coupled with the mesh indices ordered on print order for that extruder.
     */
    std::vector<int> calculateExtruderOrder(SliceDataStorage& storage, int current_extruder);

    /*!
     * Calculate in which order to plan the meshes of a specific extruder
     * 
     * \param[in] storage where the slice data is stored.
     * \param extruder_nr The extruder for which to determine the order
     * \return A vector of pairs of extruder numbers coupled with the mesh indices ordered on print order for that extruder.
     */
    std::vector<unsigned int> calculateMeshOrder(SliceDataStorage& storage, int extruder_nr);

    /*!
     * Add a single layer from a single mesh-volume to the layer plan \p gcodeLayer in mesh surface mode.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh to add to the layer plan \p gcodeLayer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshLayerToGCode_meshSurfaceMode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
    /*!
     * Add the open polylines from a single layer from a single mesh-volume to the layer plan \p gcodeLayer for mesh the surface modes.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshOpenPolyLinesToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr);
    
    /*!
     * Add a single layer from a single mesh-volume to the layer plan \p gcode_layer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh to add to the layer plan \p gcode_layer.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr);

    /*!
     * Add a single part from a given layer of a mesh-volume to the layer plan \p gcode_layer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh to add to the layer plan \p gcode_layer.
     * \param part The part to add
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshPartToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, SliceLayerPart& part, GCodePlanner& gcode_layer, int layer_nr);
    
    /*!
     * Add thicker (multiple layers) sparse infill for a given part in a layer plan.
     * 
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_line_distance The distance between the infill lines
     * \param infill_overlap The distance by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     */
    void processMultiLayerInfill(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int fillAngle); 
    
    /*!
     * Add normal sparse infill for a given part in a layer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_line_distance The distance between the infill lines
     * \param infill_overlap The distance by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     */
    void processSingleLayerInfill(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int fillAngle);
    
    /*!
     * Generate the insets for the walls of a given layer part.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param z_seam_type dir3ective for where to start the outer paerimeter of a part
     * \param z_seam_pos The location near where to start the outer inset in case \p z_seam_type is 'back'
     */
    void processInsets(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type, Point z_seam_pos);
    
    
    /*!
     * Add the gcode of the top/bottom skin of the given part and of the perimeter gaps.
     * 
     * Perimter gaps are generated for skin outlines and printed while the skin fill of the skin part is printed.
     * Perimeter gaps between the walls are added to the gcode afterwards.
     * 
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param skin_overlap The distance by which the skin overlaps with the wall insets and the distance by which the perimeter gaps overlap with adjacent print features.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     */
    void processSkinAndPerimeterGaps(cura::GCodePlanner& gcode_layer, cura::SliceMeshStorage* mesh, cura::SliceLayerPart& part, unsigned int layer_nr, int skin_overlap, int infill_angle);

    /*!
     * Add the support to the layer plan \p gcodeLayer of the current layer for all support parts with the given \p extruder_nr.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \return whether any support was added to the layer plan
     */
    bool addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int extruder_nr);
    /*!
     * Add the support lines/walls to the layer plan \p gcodeLayer of the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \return whether any support infill was added to the layer plan
     */
    bool addSupportInfillToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    /*!
     * Add the support skins to the layer plan \p gcodeLayer of the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \return whether any support skin was added to the layer plan
     */
    bool addSupportRoofsToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    
    /*!
     * Change to a new extruder, and add the prime tower instructions if the new extruder is different from the last.
     * 
     * On layer 0 this function adds the skirt for the nozzle it switches to, instead of the prime tower.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param extruder_nr The extruder to which to switch
     */
    void setExtruder_addPrime(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int extruder_nr);
    
    /*!
     * Add the prime tower gcode for the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param prev_extruder The current extruder with which we last printed.
     */
    void addPrimeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prev_extruder);
    
    /*!
     * Add the end gcode and set all temperatures to zero.
     */
    void finalize();

    /*!
     * Calculate for each layer in the mesh the index of the vertex that is considered to be the seam
     * \param mesh The mesh containing the layers to be spiralized
     */
    void findLayerSeamsForSpiralize(SliceMeshStorage& mesh);
};

}//namespace cura

#endif // GCODE_WRITER_H
