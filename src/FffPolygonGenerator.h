#ifndef FFF_AREA_GENERATOR_H
#define FFF_AREA_GENERATOR_H


#include "MeshGroup.h"
#include "utils/polygonUtils.h"
#include "utils/NoCopy.h"
#include "utils/gettime.h"
#include "settings.h"
#include "sliceDataStorage.h"
#include "commandSocket.h"
#include "PrintFeature.h"

namespace cura
{

/*!
 * Primary stage in Fused Filament Fabrication processing: Polygons are generated.
 * The model is sliced and each slice consists of polygons representing the outlines: the boundaries between inside and outside the object.
 * After slicing, the layers are processed; for example the wall insets are generated, and the areas which are to be filled with support and infill, which are all represented by polygons.
 * In this stage nothing other than areas and circular paths are generated, which are both represented by polygons.
 * No infill lines or support pattern etc. is generated.
 * 
 * The main function of this class is FffPolygonGenerator::generateAreas().
 */
class FffPolygonGenerator : public SettingsMessenger, NoCopy
{
private:
    CommandSocket* command_socket;
public:
    /*!
     * Basic constructor; doesn't set the FffAreaGenerator::command_socket .
     */
    FffPolygonGenerator(SettingsBase* settings_)
    : SettingsMessenger(settings_)
    , command_socket(nullptr)
    {
    }
    
    /*!
     * Set the FffAreaGenerator::command_socket
     */
    void setCommandSocket(CommandSocket* socket)
    {
        command_socket = socket;
    }
    

    /*!
     * Slice the \p object, process the outline information into inset perimeter polygons, support area polygons, etc. 
     * 
     * \param object The object to slice.
     * \param timeKeeper Object which keeps track of timings of each stage.
     * \param storage Output parameter: where the outlines are stored. See SliceLayerPart::outline.
     */
    bool generateAreas(SliceDataStorage& storage, MeshGroup* object, TimeKeeper& timeKeeper);
  
private:
    /*!
     * Send polygons over the command socket, if there is one.
     * \param type The type of polygon to send
     * \param layer_nr The layer number at which the polygons occur
     * \param polygons The polygons to be sent
     */
    void sendPolygons(PrintFeatureType type, int layer_nr, Polygons& polygons, int line_width)
    {
        if (command_socket)
            command_socket->sendPolygons(type, layer_nr, polygons, line_width);
    }
    
    /*!
     * Slice the \p object and store the outlines in the \p storage.
     * 
     * \param object The object to slice.
     * \param timeKeeper Object which keeps track of timings of each stage.
     * \param storage Output parameter: where the outlines are stored. See SliceLayerPart::outline.
     * 
     * \return Whether the process succeeded (always true).
     */
    bool sliceModel(MeshGroup* object, TimeKeeper& timeKeeper, SliceDataStorage& storage); /// slices the model

    /*!
     * Processes the outline information as stored in the \p storage: generates inset perimeter polygons, support area polygons, etc. 
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param timeKeeper Object which keeps track of timings of each stage.
     */
    void slices2polygons(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
    /*!
     * Remove all bottom layers which are empty.
     * \param storage Input and Ouput parameter: stores all layers
     * \param layer_height The height of each layer
     * \param total_layers The total number of layers
     */
    void removeEmptyFirstLayers(SliceDataStorage& storage, int layer_height, unsigned int total_layers);
    
    /*!
     * Generate the inset polygons which form the walls.
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param layer_nr The layer for which to generate the insets.
     */
    void processInsets(SliceDataStorage& storage, unsigned int layer_nr);

    /*!
     * Generate the outline of the ooze shield.
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param total_layers The total number of layers 
     */
    void processOozeShield(SliceDataStorage& storage, unsigned int total_layers);
    
    /*!
     * Generate the skin areas.
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param layer_nr The layer for which to generate the skin areas.
     */
    void processSkinsAndInfill(SliceDataStorage& storage, unsigned int layer_nr); 

    /*!
     * Generate the polygons where the draft screen should be.
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param total_layers The total number of layers 
     */
    void processDraftShield(SliceDataStorage& storage, unsigned int total_layers);
    /*!
     * Generate the skirt/brim/raft areas/insets.
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     */
    void processPlatformAdhesion(SliceDataStorage& storage);
    
    
    
    /*!
     * Make the outer wall 'fuzzy'
     */
    void processFuzzyWalls(SliceMeshStorage& mesh);
    
    

    
};
}//namespace cura
#endif // FFF_AREA_GENERATOR_H
