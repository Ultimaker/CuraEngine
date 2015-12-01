#ifndef GCODE_WRITER_H
#define GCODE_WRITER_H


#include <fstream>
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"
#include "raft.h"
#include "infill.h"
#include "bridge.h"
#include "pathOrderOptimizer.h"
#include "gcodePlanner.h"
#include "gcodeExport.h"
#include "commandSocket.h"
#include "utils/polygonUtils.h"
#include "PrimeTower.h"
#include "FanSpeedLayerTime.h"


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
class FffGcodeWriter : public SettingsMessenger
{
    friend class FffProcessor; // cause WireFrame2Gcode uses the member [gcode] (TODO)
private:
    int max_object_height;
    int meshgroup_number; //!< used for sequential printing of objects
    
    LayerPlanBuffer layer_plan_buffer;
    
    GCodeExport gcode;
    CommandSocket* command_socket;
    std::ofstream output_file;
    
    /*!
     * Layer number of the last layer in which a prime tower has been printed per extruder train.  
     * 
     * This is recorded per extruder to account for a prime tower per extruder, instead of the mixed prime tower.
     */
    int last_prime_tower_poly_printed[MAX_EXTRUDERS]; 
    
    FanSpeedLayerTimeSettings fan_speed_layer_time_settings;
    
    Point last_position_planned; //!< The position of the head before planning the next layer
    int current_extruder_planned; //!< The extruder train in use before planning the next layer
public:
    FffGcodeWriter(SettingsBase* settings_)
    : SettingsMessenger(settings_)
    , layer_plan_buffer(this, command_socket, gcode)
    , last_position_planned(no_point)
    , current_extruder_planned(0) // TODO: make configurable
    {
        meshgroup_number = 1;
        max_object_height = 0;
        command_socket = NULL;
    }
    void resetFileNumber()
    {
        meshgroup_number = 1;
    }

    void setCommandSocket(CommandSocket* socket)
    {
        command_socket = socket;
    }

    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons, int line_width)
    {
        if (command_socket)
            command_socket->sendPolygons(type, layer_nr, polygons, line_width);
    }
        
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
    
    void setTargetStream(std::ostream* stream)
    {
        gcode.setOutputStream(stream);
    }
    
    double getTotalFilamentUsed(int e)
    {
        return gcode.getTotalFilamentUsed(e);
    }

    double getTotalPrintTime()
    {
        return gcode.getTotalPrintTime();
    }

    void writeGCode(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
private:
    void setConfigFanSpeedLayerTime();
    
    void setConfigCoasting(SliceDataStorage& storage);

    //Setup the retraction parameters.
    void setConfigRetraction(SliceDataStorage& storage);
    
    /*!
     * initialize GcodePathConfig config parameters which don't change over all layers
     */
    void initConfigs(SliceDataStorage& storage);
    
    /*!
     * Set temperatures and perform initial priming.
     * \param storage Input: where the slice data is stored.
     */
    void processStartingCode(SliceDataStorage& storage);

    /*!
     * Move up and over the just printed model to print the next model.
     * \param storage Input: where the slice data is stored.
     */
    void processNextMeshGroupCode(SliceDataStorage& storage);
    
    /*!
     * Add raft gcode.
     * \param storage Input: where the slice data is stored.
     * \param total_layers The total number of layers.
     */
    void processRaft(SliceDataStorage& storage, unsigned int total_layers);
    
    /*!
     * Add a layer to the gcode.
     * \param storage Input: where the slice data is stored.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param total_layers The total number of layers.
     * \param has_raft Whether a raft is used for this print.
     */
    void processLayer(SliceDataStorage& storage, unsigned int layer_nr, unsigned int total_layers, bool has_raft);
    
    /*!
     * Add the skirt to the gcode.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param extruder_nr The extrudewr train for which to process the skirt
     */
    void processSkirt(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int extruder_nr);
    
    /*!
     * Adds the ooze shield to the print.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processOozeShield(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    /*!
     * Adds the draft protection screen to the print.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processDraftShield(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    /*!
     * Calculate in which order to print the meshes.
     * \param storage Input: where the slice data is stored.
     * \param current_extruder The current extruder with which we last printed
     * \return A vector of mesh indices ordered on print order.
     */
    std::vector<unsigned int> calculateMeshOrder(SliceDataStorage& storage, int current_extruder);
        
    /*!
     * Add a single layer from a single mesh-volume to the GCode in mesh surface mode.
     * \param storage Input: where the slice data is stored.
     * \param mesh The mesh to add to the gcode.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshLayerToGCode_meshSurfaceMode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
    /*!
     * Add the open polylines from a single layer from a single mesh-volume to the GCode for mesh surface mode.
     * \param storage Input: where the slice data is stored.
     * \param mesh The mesh for which to add to the gcode.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshOpenPolyLinesToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr);
    
    /*!
     * Add a single layer from a single mesh-volume to the GCode.
     * \param storage Input: where the slice data is stored.
     * \param mesh The mesh to add to the gcode.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
    /*!
     * Add thicker (multiple layers) sparse infill for a given part in a layer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the gcode.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_line_distance The distance between the infill lines
     * \param infill_overlap The fraction of the extrusion width by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     * \param extrusionWidth extrusionWidth
     */
    void processMultiLayerInfill(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, double infill_overlap, int fillAngle, int extrusionWidth); 
    
    /*!
     * Add normal sparse infill for a given part in a layer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the gcode.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_line_distance The distance between the infill lines
     * \param infill_overlap The fraction of the extrusion width by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     * \param extrusionWidth extrusionWidth
     */
    void processSingleLayerInfill(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, double infill_overlap, int fillAngle, int extrusionWidth);
    
    /*!
     * Generate the insets for the walls of a given layer part.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the gcode.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param z_seam_type dir3ective for where to start the outer paerimeter of a part
     */
    void processInsets(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type);
    
    
    /*!
     * Add the gcode of the top/bottom skin of the given part.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the gcode.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     * \param infill_overlap The fraction of the extrusion width by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     * \param extrusionWidth extrusionWidth
     */
    void processSkin(cura::GCodePlanner& gcode_layer, cura::SliceMeshStorage* mesh, cura::SliceLayerPart& part, unsigned int layer_nr, double infill_overlap, int infill_angle, int extrusion_width);
    
    /*!
     * Add the support to the gcode of the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param extruder_nr_before The extruder number at the start of the layer (before other print parts aka the rest)
     * \param before_rest Whether the function has been called before adding the rest to the gcode, or after.
     */
    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int extruder_nr_before, bool before_rest);
    /*!
     * Add the support lines/walls to the gcode of the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void addSupportInfillToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    /*!
     * Add the support roofs to the gcode of the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void addSupportRoofsToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    
    /*!
     * Change to a new extruder, and add the prime tower instructions if the new extruder is different from the last.
     * 
     * On layer 0 this function adds the skirt for the nozzle it switches to, instead of the prime tower.
     * 
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param extruder_nr The extruder to which to switch
     */
    void setExtruder_addPrime(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int extruder_nr);
    
    /*!
     * Add the prime tower gcode for the current layer.
     * \param storage Input: where the slice data is stored.
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
