#ifndef FFF_AREA_GENERATOR_H
#define FFF_AREA_GENERATOR_H


#include "settings.h"
#include "sliceDataStorage.h"
#include "FffGcodeWriter.h"
#include "FffAreaGenerator.h"
#include "commandSocket.h"
#include "Weaver.h"
#include "Wireframe2gcode.h"
#include "utils/polygonUtils.h"

namespace cura
{


class FffAreaGenerator
{
private:
    CommandSocket* commandSocket;
    SettingsBase& settings;
public:
    FffAreaGenerator(SettingsBase& settings_)
    : settings(settings_)
    {
    }

    void setCommandSocket(CommandSocket* socket)
    {
        commandSocket = socket;
    }
  
    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
    {
        if (commandSocket)
            commandSocket->sendPolygons(type, layer_nr, polygons);
    }
    
    bool prepareModel(SliceDataStorage& storage, PrintObject* object, TimeKeeper& timeKeeper); /// slices the model

    void processSliceData(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
};
} // namespace cura
#endif // FFF_AREA_GENERATOR_H