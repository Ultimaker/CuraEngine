#ifndef COMMAND_SOCKET_H
#define COMMAND_SOCKET_H

#include "utils/socket.h"
#include "utils/polygon.h"
#include "settings.h"

#include <memory>

#include "Cura.pb.h"

namespace cura {

class fffProcessor;
class CommandSocket
{
public:
    CommandSocket(fffProcessor* processor);

    void connect(const std::string& ip, int port);

    void handleObjectList(Cura::ObjectList* list);
    void handleSettingList(Cura::SettingList* list);
    
    void sendLayerInfo(int layer_nr, int32_t z, int32_t height);
    void sendPolygons(cura::PolygonType type, int layer_nr, cura::Polygons& polygons, int line_width);
    void sendProgress(float amount);
    void sendPrintTime();
    void sendPrintMaterialForObject(int index, int extruder_nr, float material_amount);

    void beginSendSlicedObject();
    void endSendSlicedObject();

    void beginGCode();
    void sendGCodeLayer();
    void sendGCodePrefix(std::string prefix);

private:
    class Private;
    const std::unique_ptr<Private> d;
};

}//namespace cura

#endif//COMMAND_SOCKET_H
