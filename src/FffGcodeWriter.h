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
     * Layer number of the last layer in which a prime tower has been printed per extruder train.  
     * 
     * This is recorded per extruder to account for a prime tower per extruder, instead of the mixed prime tower.
     */
    int last_prime_tower_poly_printed[MAX_EXTRUDERS]; 

    /*!
     * Whether the skirt or brim polygons have been processed into planned paths
     * for each extruder train.
     */
    bool skirt_brim_is_processed[MAX_EXTRUDERS];

    std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder; //!< The settings used relating to minimal layer time and fan speeds. Configured for each extruder.

    Point last_position_planned; //!< The position of the head before planning the next layer
    int current_extruder_planned; //!< The extruder train in use before planning the next layer
    bool is_inside_mesh_layer_part; //!< Whether the last position was inside a layer part (used in combing)
public:
    FffGcodeWriter(SettingsBase* settings_)
    : SettingsMessenger(settings_)
    , layer_plan_buffer(this, gcode)
    , last_position_planned(no_point)
    , current_extruder_planned(0) // changed somewhere early in FffGcodeWriter::writeGCode
    , is_inside_mesh_layer_part(false)
    {
        max_object_height = 0;
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
     * Set temperatures and perform initial priming.
     * 
     * Write a stub header if CuraEngine is in command line tool mode. (Cause writing the header afterwards would entail moving all gcode down.)
     * 
     * \param[in] storage where the slice data is stored.
     */
    void processStartingCode(SliceDataStorage& storage);

    /*!
     * Move up and over the already printed meshgroups to print the next meshgroup.
     * 
     * \param[in] storage where the slice data is stored.
     */
    void processNextMeshGroupCode(SliceDataStorage& storage);
    
    /*!
     * Add raft layer plans onto the FffGcodeWriter::layer_plan_buffer
     * 
     * \param[in] storage where the slice data is stored.
     * \param total_layers The total number of layers.
     */
    void processRaft(SliceDataStorage& storage, unsigned int total_layers);
    
    /*!
     * Convert the polygon data of a layer into a layer plan on the FffGcodeWriter::layer_plan_buffer
     * 
     * \param[in] storage where the slice data is stored.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param total_layers The total number of layers.
     * \param has_raft Whether a raft is used for this print.
     */
    void processLayer(SliceDataStorage& storage, unsigned int layer_nr, unsigned int total_layers, bool has_raft);
    
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
     * Calculate in which order to print the meshes.
     * 
     * \param[in] storage where the slice data is stored.
     * \param current_extruder The current extruder with which we last printed
     * \return A vector of mesh indices ordered on print order.
     */
    std::vector<unsigned int> calculateMeshOrder(SliceDataStorage& storage, int current_extruder);
        
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
     * Add a single layer from a single mesh-volume to the layer plan \p gcodeLayer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh to add to the layer plan \p gcodeLayer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
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
     */
    void processInsets(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type);
    
    
    /*!
     * Add the gcode of the top/bottom skin of the given part.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param skin_overlap The distance by which the skin overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     */
    void processSkin(cura::GCodePlanner& gcode_layer, cura::SliceMeshStorage* mesh, cura::SliceLayerPart& part, unsigned int layer_nr, int skin_overlap, int infill_angle);
    
    /*!
     * Add the support to the layer plan \p gcodeLayer of the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param extruder_nr_before The extruder number at the start of the layer (before other print parts aka the rest)
     * \param before_rest Whether the function has been called before adding the rest to the layer plan \p gcodeLayer, or after.
     */
    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int extruder_nr_before, bool before_rest);
    /*!
     * Add the support lines/walls to the layer plan \p gcodeLayer of the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void addSupportInfillToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    /*!
     * Add the support roofs to the layer plan \p gcodeLayer of the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void addSupportRoofsToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    
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
};

}//namespace cura

#endif // GCODE_WRITER_H
