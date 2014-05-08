#ifndef COMMAND_SOCKET_H
#define COMMAND_SOCKET_H

#include "utils/socket.h"
#include "utils/polygon.h"
#include "settings.h"

namespace cura {

class fffProcessor;
class CommandSocket
{
private:
    ClientSocket socket;
public:
    CommandSocket(int portNr);
    
    void handleIncommingData(ConfigSettings* config, fffProcessor* processor);
    
    void sendPolygons(const char* name, int layerNr, int32_t z, Polygons& polygons);
    void sendProgress(float amount);
};

}//namespace cura

#endif//COMMAND_SOCKET_H
