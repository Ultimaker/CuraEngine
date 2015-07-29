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

namespace cura 
{

/*!
 * Secondary stage in Fused Filament Fabrication processing: The generated polygons are used in the gcode generation.
 * Some polygons in the SliceDataStorage signify areas which are to be filled with parallel lines, 
 * while other polygons signify the contours which should be printed.
 * 
 * The main function of this class is FffGcodeWriter::writeGCode().
 */
class FffGcodeWriter : public SettingsBase
{
    friend class fffProcessor; // cause WireFrame2Gcode uses the member [gcode] (TODO)
private:
    int max_object_height;
    int file_number; //!< used for sequential printing of objects
    GCodeExport gcode;
    CoastingConfig coasting_config;
    CommandSocket* command_socket;
    std::ofstream output_file;
public:
    FffGcodeWriter(SettingsBase* settings_)
    : SettingsBase(settings_)
    {
        file_number = 1;
        max_object_height = 0;
        command_socket = NULL;
    }
    void resetFileNumber()
    {
        file_number = 1;
    }

    void setCommandSocket(CommandSocket* socket)
    {
        command_socket = socket;
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
    void setConfigCoasting();

    //Setup the retraction parameters.
    void setConfigRetraction(SliceDataStorage& storage);
    
    void setConfigSkirt(SliceDataStorage& storage, int layer_thickness);
    
    void setConfigSupport(SliceDataStorage& storage, int layer_thickness);
    
    void setConfigInsets(SliceMeshStorage& mesh, int layer_thickness);
    
    void setConfigSkin(SliceMeshStorage& mesh, int layer_thickness);
    
    void setConfigInfill(SliceMeshStorage& mesh, int layer_thickness);
    
    /*!
     * Set temperatures and perform initial priming.
     * \param storage Input: where the slice data is stored.
     */
    void processStartingCode(SliceDataStorage& storage);

    /*!
     * Move up and over the just printed model to print the next model.
     * \param storage Input: where the slice data is stored.
     */
    void processNextPrintObjectCode(SliceDataStorage& storage);
    
    /*!
     * Add raft gcode.
     * \param storage Input: where the slice data is stored.
     * \param totalLayers The total number of layers.
     */
    void processRaft(SliceDataStorage& storage, unsigned int totalLayers);
    
    /*!
     * Add a layer to the gcode.
     * \param storage Input: where the slice data is stored.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param totalLayers The total number of layers.
     * \param has_raft Whether a raft is used for this print.
     */
    void processLayer(SliceDataStorage& storage, unsigned int layer_nr, unsigned int totalLayers, bool has_raft);
    
    /*!
     * Interpolate between the initial layer speeds and the eventual speeds.
     * \param storage Input: where the slice data is stored.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processInitialLayersSpeedup(SliceDataStorage& storage, unsigned int layer_nr);
    
    /*!
     * Add the skirt to the gcode.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processSkirt(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
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
     * \return A vector of meshes ordered on print order.
     */
    std::vector<SliceMeshStorage*> calculateMeshOrder(SliceDataStorage& storage, int current_extruder);
        
    /*!
     * Add a single layer from a single mesh-volume to the GCode in magic polygon mode.
     * \param storage Input: where the slice data is stored.
     * \param mesh The mesh to add to the gcode.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * 
     */
    void addMeshLayerToGCode_magicPolygonMode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
    
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
     * \param sparse_infill_line_distance The distance between the infill lines
     * \param infill_overlap The fraction of the extrusion width by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     * \param extrusionWidth extrusionWidth
     */
    void processMultiLayerInfill(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, int sparse_infill_line_distance, double infill_overlap, int fillAngle, int extrusionWidth); 
    
    /*!
     * Add normal sparse infill for a given part in a layer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the gcode.
     * \param part The part for which to create gcode
     * \param sparse_infill_line_distance The distance between the infill lines
     * \param infill_overlap The fraction of the extrusion width by which the infill overlaps with the wall insets.
     * \param fillAngle The angle in the XY plane at which the infill is generated.
     * \param extrusionWidth extrusionWidth
     */
    void processSingleLayerInfill(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, int sparse_infill_line_distance, double infill_overlap, int fillAngle, int extrusionWidth);
    
    /*!
     * Generate the insets for the walls of a given layer part.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the gcode.
     * \param part The part for which to create gcode
     * \param layer_nr The current layer number.
     */
    void processInsets(GCodePlanner& gcodeLayer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr);
    
    
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
    void processSkin(cura::GCodePlanner& gcode_layer, cura::SliceMeshStorage* mesh, cura::SliceLayerPart& part, unsigned int layer_nr, double infill_overlap, int fill_angle, int extrusion_width);
    
    /*!
     * Add the support to the gcode of the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param before_rest Whether the function has been called before adding the rest to the gcode, or after.
     */
    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, bool before_rest);
    /*!
     * Add the support lines/walls to the gcode of the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void addSupportLinesToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    /*!
     * Add the support roofs to the gcode of the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void addSupportRoofsToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    
    /*!
     * Add the wipe tower gcode for the current layer.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param prev_extruder The current extruder with which we last printed.
     */
    void addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prev_extruder);
    
    /*!
     * Finish the layer by applying speed corrections for minimal layer times and determine the fanSpeed.
     * \param storage Input: where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param layer_nr The index of the layer to write the gcode of.
     */
    void processFanSpeedAndMinimalLayerTime(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    /*!
     * Add the end gcode and set all temperatures to zero.
     */
    void finalize();
};

}//namespace cura

#endif // GCODE_WRITER_H
