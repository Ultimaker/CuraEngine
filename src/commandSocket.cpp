#include "utils/logoutput.h"
#include "commandSocket.h"
#include "fffProcessor.h"

namespace cura {

const static int CMD_SETTING = 0x0001;
const static int CMD_MATRIX = 0x0002;
const static int CMD_PROCESS_MESH = 0x0003;
const static int CMD_START_MESH = 0x1000;
const static int CMD_START_VOLUME = 0x1001;
const static int CMD_VOLUME_VERTEX_POSITION = 0x1002;
const static int CMD_VOLUME_VERTEX_NORMAL = 0x1003;
const static int CMD_FINISHED = 0x9000;

CommandSocket::CommandSocket(int portNr)
{
    socket.connectTo("127.0.0.1", portNr);
}

void CommandSocket::handleIncommingData(ConfigSettings* config, fffProcessor* processor)
{
    SimpleModel* model = NULL;
    SimpleVolume* volume = NULL;
    
    while(true)
    {
        int dataSize = socket.recvNr();
        dataSize -= 4;
        if (dataSize < 0)
            break;
        int command = socket.recvNr();
        switch(command)
        {
        case CMD_SETTING:
            {
                char buffer[dataSize+1];
                buffer[dataSize] = '\0';
                socket.recvAll(buffer, dataSize);
                char* value = strchr(buffer, '=');
                if (value)
                    config->setSetting(buffer, value+1);
            }
            break;
        case CMD_START_MESH:
            if (model)
                delete model;
            model = new SimpleModel();
            volume = NULL;
            break;
        case CMD_START_VOLUME:
            if (model)
            {
                model->volumes.push_back(SimpleVolume());
                volume = &model->volumes[model->volumes.size()-1];
            }
            break;
        case CMD_VOLUME_VERTEX_POSITION:
            if (volume)
            {
                int faceCount = dataSize / 4 / 3 / 3;
                logError("Reading %i faces\n", faceCount);
                for(int n=0; n<faceCount; n++)
                {
                    FPoint3 fv[3];
                    socket.recvAll(fv, 4 * 3 * 3);
                    Point3 v[3];
                    v[0] = config->matrix.apply(fv[0]);
                    v[1] = config->matrix.apply(fv[1]);
                    v[2] = config->matrix.apply(fv[2]);
                    volume->addFace(v[0], v[1], v[2]);
                }
            }else{
                for(int n=0; n<dataSize; n++)
                    socket.recvAll(&command, 1);
            }
            break;
        case CMD_PROCESS_MESH:
            if (model)
            {
                processor->processModel(model);
                model = NULL;
            }
            break;
        case CMD_FINISHED:
            socket.close();
            break;
        default:
            logError("Unknown command: %04x (%i)\n", command, dataSize);
            for(int n=0; n<dataSize; n++)
                socket.recvAll(&command, 1);
            break;
        }
    }
    if (model)
        delete model;
}

void CommandSocket::sendPolygons(const char* name, int layerNr, int32_t z, Polygons& polygons)
{
}

}//namespace cura
