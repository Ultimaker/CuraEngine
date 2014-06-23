#include "utils/logoutput.h"
#include "commandSocket.h"
#include "fffProcessor.h"

namespace cura {

#define CURA_IDENTIFIER "CuraEngine"
const static int CMD_REQUEST_IDENTIFIER = 0x00100000;
const static int CMD_IDENTIFIER_REPLY = 0x00100001;
const static int CMD_REQUEST_VERSION = 0x00100002;
const static int CMD_VERSION_REPLY = 0x00100003;

const static int CMD_SETTING = 0x00100004;
const static int CMD_MATRIX = 0x00300002;
const static int CMD_PROCESS_MESH = 0x00300000;
const static int CMD_OBJECT_LIST = 0x00200000;
const static int CMD_MESH_LIST = 0x00200001;
const static int CMD_VERTEX_LIST = 0x00200002;
const static int CMD_NORMAL_LIST = 0x00200003;
const static int CMD_FINISHED = 0x00300003;

const static int CMD_PROGRESS_REPORT = 0x00300001;

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
        int command = socket.recvInt32();
        int dataSize = socket.recvInt32();
        if (dataSize < 0)
            break;
        switch(command)
        {
        case CMD_REQUEST_IDENTIFIER:
            socket.sendInt32(CMD_IDENTIFIER_REPLY);
            socket.sendInt32(strlen(CURA_IDENTIFIER) + 1);
            socket.sendAll(CURA_IDENTIFIER, strlen(CURA_IDENTIFIER) + 1);
            break;
        case CMD_SETTING:
            {
                char buffer[dataSize+1];
                buffer[dataSize] = '\0';
                socket.recvAll(buffer, dataSize);
                char* value = (buffer + strlen(buffer)) + 1;
                if ((value - buffer) < dataSize)
                {
                    config->setSetting(buffer, value);
                }
            }
            break;
        case CMD_MATRIX:
            {
                for(int x=0; x<3; x++)
                    for(int y=0; y<3; y++)
                        config->matrix.m[x][y] = socket.recvFloat32();
            }
            break;
        case CMD_OBJECT_LIST:
            socket.recvInt32(); //Number of following CMD_MESH_LIST commands
            if (model)
                delete model;
            model = new SimpleModel();
            volume = NULL;
            break;
        case CMD_MESH_LIST:
            socket.recvInt32(); //Number of following CMD_?_LIST commands that fill this mesh with data
            if (model)
            {
                model->volumes.push_back(SimpleVolume());
                volume = &model->volumes[model->volumes.size()-1];
            }
            break;
        case CMD_VERTEX_LIST:
            if (volume)
            {
                Point3 offset(config->objectPosition.X, config->objectPosition.Y, 0);
                int faceCount = dataSize / 4 / 3 / 3;
                logError("Reading %i faces\n", faceCount);
                logError("%i %i\n", config->objectPosition.X, config->objectPosition.Y);
                for(int n=0; n<faceCount; n++)
                {
                    FPoint3 fv[3];
                    socket.recvAll(fv, 4 * 3 * 3);
                    Point3 v[3];
                    v[0] = config->matrix.apply(fv[0]) + offset;
                    v[1] = config->matrix.apply(fv[1]) + offset;
                    v[2] = config->matrix.apply(fv[2]) + offset;
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
    //TODO
}

void CommandSocket::sendProgress(float amount)
{
    socket.sendInt32(CMD_PROGRESS_REPORT);
    socket.sendInt32(4);
    socket.sendFloat32(amount);
}

}//namespace cura
