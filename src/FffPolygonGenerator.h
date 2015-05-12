#ifndef FFF_AREA_GENERATOR_H
#define FFF_AREA_GENERATOR_H


#include "modelFile/modelFile.h"
#include "utils/polygonUtils.h"
#include "utils/gettime.h"
#include "settings.h"
#include "sliceDataStorage.h"
#include "commandSocket.h"

namespace cura
{

/*!
 * Primary stage in Fused Filament Fabrication processing: Polygons are generated.
 * The model is sliced and each slice consists of polygons representing the outlines: the boundaries between inside and outside the object.
 * After slicing, the layers are processed; for example the wall insets are generated, and the areas which are to be filled with support and infill, which are all represented by polygons.
 * In this stage nothing other than areas and circular paths are generated, which are both represented by polygons.
 * No infill lines or support pattern etc. is generated.
 */
class FffPolygonGenerator : public SettingsBase
{
private:
    CommandSocket* commandSocket;
public:
    /*!
     * Basic constructor; doesn't set the FffAreaGenerator::commandSocket .
     */
    FffPolygonGenerator(SettingsBase* settings_)
    : SettingsBase(settings_)
    , commandSocket(nullptr)
    {
    }
    
    /*!
     * Set the FffAreaGenerator::commandSocket
     */
    void setCommandSocket(CommandSocket* socket)
    {
        commandSocket = socket;
    }
  
private:
    /*!
     * Send polygons over the command socket, if there is one.
     * \param type The type of polygon to send
     * \param layer_nr The layer number at which the polygons occur
     * \param polygons The polygons to be sent
     */
    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
    {
        if (commandSocket)
            commandSocket->sendPolygons(type, layer_nr, polygons);
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
    bool sliceModel(PrintObject* object, TimeKeeper& timeKeeper, SliceDataStorage& storage); /// slices the model

    /*!
     * Processes the outline information as stored in the \p storage: generates inset perimeter polygons, support area polygons, etc. 
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param timeKeeper Object which keeps track of timings of each stage.
     */
    void slices2polygons(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
    void slices2polygons_magicPolygonMode(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
    void removeEmptyFirstLayers(SliceDataStorage& storage, int layer_height, unsigned int totalLayers);
    
    void processInsets(SliceDataStorage& storage, unsigned int layer_nr);

    void processOozeShield(SliceDataStorage& storage, unsigned int totalLayers);
    
    void processSkins(SliceDataStorage& storage, unsigned int layer_nr); 

    void processWipeTower(SliceDataStorage& storage, unsigned int totalLayers);
    
    void processPlatformAdhesion(SliceDataStorage& storage);
    
    



public:
    /*!
     * Slice the \p object, process the outline information into inset perimeter polygons, support area polygons, etc. 
     * 
     * \param object The object to slice.
     * \param timeKeeper Object which keeps track of timings of each stage.
     * \param storage Output parameter: where the outlines are stored. See SliceLayerPart::outline.
     */
    bool generateAreas(SliceDataStorage& storage, PrintObject* object, TimeKeeper& timeKeeper);
    
};
} // namespace cura
#endif // FFF_AREA_GENERATOR_H