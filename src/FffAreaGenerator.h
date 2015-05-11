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
 * Primary stage in Fused Filament Fabrication processing: Areas are generated.
 * Each layer in the model consists of areas, 
 */
class FffAreaGenerator
{
private:
    CommandSocket* commandSocket;
    SettingsBase& settings; //!< Reference to the global settings
public:
    /*!
     * Basic constructor; doesn't set the FffAreaGenerator::commandSocket .
     */
    FffAreaGenerator(SettingsBase& settings_)
    : settings(settings_)
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
     */
    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
    {
        if (commandSocket)
            commandSocket->sendPolygons(type, layer_nr, polygons);
    }
    
    bool sliceModel(SliceDataStorage& storage, PrintObject* object, TimeKeeper& timeKeeper); /// slices the model

    void slices2areas(SliceDataStorage& storage, TimeKeeper& timeKeeper);
public:
    bool generateAreas(SliceDataStorage& storage, PrintObject* object, TimeKeeper& timeKeeper);
    
};
} // namespace cura
#endif // FFF_AREA_GENERATOR_H