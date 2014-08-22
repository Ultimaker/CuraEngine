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
const static int CMD_OBJECT_COUNT = 0x00300003;
const static int CMD_OBJECT_LIST = 0x00200000;
const static int CMD_MESH_LIST = 0x00200001;
const static int CMD_VERTEX_LIST = 0x00200002;
const static int CMD_NORMAL_LIST = 0x00200003;
const static int CMD_PROCESS_MESH = 0x00300000;

const static int CMD_PROGRESS_REPORT = 0x00300001;
const static int CMD_OBJECT_PRINT_TIME = 0x00300004;
const static int CMD_OBJECT_PRINT_MATERIAL = 0x00300005;
const static int CMD_LAYER_INFO = 0x00300007;
const static int CMD_POLYGON = 0x00300006;

CommandSocket::CommandSocket(int portNr)
{
    socket.connectTo("127.0.0.1", portNr);
    object_count = 1;
    current_object_number = 0;
}

void CommandSocket::handleIncommingData(fffProcessor* processor)
{
    std::vector<PrintObject*> object_list;
    PrintObject* object = NULL;
    Mesh* mesh = NULL;
    FMatrix3x3 matrix;
    float total_print_time = 0.0;
    float total_material_amount[MAX_EXTRUDERS];
    for(int n=0; n<MAX_EXTRUDERS; n++)
        total_material_amount[n] = 0.0;
    
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
                    //processor->getSetting(buffer);
                    if (mesh)
                        mesh->setSetting(buffer, value);
                    else if (object)
                        object->setSetting(buffer, value);
                    else
                        processor->setSetting(buffer, value);
                }
            }
            break;
        case CMD_MATRIX:
            {
                for(int x=0; x<3; x++)
                    for(int y=0; y<3; y++)
                        matrix.m[x][y] = socket.recvFloat32();
            }
            break;
        case CMD_OBJECT_COUNT:
            object_count = socket.recvInt32();
            current_object_number = 0;
            break;
        case CMD_OBJECT_LIST:
            socket.recvInt32(); //Number of following CMD_MESH_LIST commands
            if (object)
            {
                object->finalize();
                object_list.push_back(object);
            }
            object = new PrintObject(processor);
            mesh = NULL;
            break;
        case CMD_MESH_LIST:
            socket.recvInt32(); //Number of following CMD_?_LIST commands that fill this mesh with data
            if (object)
            {
                object->meshes.emplace_back(object);
                mesh = &object->meshes[object->meshes.size()-1];
            }
            break;
        case CMD_VERTEX_LIST:
            if (mesh)
            {
                int faceCount = dataSize / 4 / 3 / 3;
                logError("Reading %i faces\n", faceCount);
                for(int n=0; n<faceCount; n++)
                {
                    FPoint3 fv[3];
                    socket.recvAll(fv, 4 * 3 * 3);
                    Point3 v[3];
                    v[0] = matrix.apply(fv[0]);
                    v[1] = matrix.apply(fv[1]);
                    v[2] = matrix.apply(fv[2]);
                    mesh->addFace(v[0], v[1], v[2]);
                }
                mesh->finish();
            }else{
                for(int n=0; n<dataSize; n++)
                    socket.recvAll(&command, 1);
            }
            break;
        case CMD_PROCESS_MESH:
            if (object)
            {
                object->finalize();
                for(PrintObject* obj : object_list)
                    for(Mesh& m : obj->meshes)
                        object->meshes.push_back(m);
                processor->processModel(object);
                for(PrintObject* obj : object_list)
                    delete obj;
                object_list.clear();
                sendPrintTimeForObject(current_object_number, processor->getTotalPrintTime() - total_print_time);
                total_print_time = processor->getTotalPrintTime();
                for(int n=0; n<MAX_EXTRUDERS; n++)
                {
                    if (processor->getTotalFilamentUsed(n) != total_material_amount[n])
                        sendPrintMaterialForObject(current_object_number, n, processor->getTotalFilamentUsed(n) - total_material_amount[n]);
                    total_material_amount[n] = processor->getTotalFilamentUsed(n);
                }
                current_object_number++;
                if (current_object_number >= object_count)
                    socket.close();
                delete object;
                object = NULL;
                mesh = NULL;
            }
            break;
        default:
            logError("Unknown command: %04x (%i)\n", command, dataSize);
            for(int n=0; n<dataSize; n++)
                socket.recvAll(&command, 1);
            break;
        }
    }
    if (object)
        delete object;
}

void CommandSocket::sendLayerInfo(int layer_nr, int32_t z, int32_t height)
{
    socket.sendInt32(CMD_LAYER_INFO);
    socket.sendInt32(4 * 4);
    socket.sendInt32(current_object_number);
    socket.sendInt32(layer_nr);
    socket.sendInt32(z);
    socket.sendInt32(height);
}

void CommandSocket::sendPolygons(const char* name, int layer_nr, Polygons& polygons)
{
    int size = (strlen(name) + 1) + 3 * 4 + polygons.size() * 4;
    for(unsigned int n=0; n<polygons.size(); n++)
        size += polygons[n].size() * sizeof(Point);
    if (polygons.size() < 1)
        return;

    socket.sendInt32(CMD_POLYGON);
    socket.sendInt32(size);
    socket.sendAll(name, strlen(name) + 1);
    socket.sendInt32(current_object_number);
    socket.sendInt32(layer_nr);
    socket.sendInt32(polygons.size());
    for(unsigned int n=0; n<polygons.size(); n++)
    {
        socket.sendInt32(polygons[n].size());
        socket.sendAll(polygons[n].data(), polygons[n].size() * sizeof(Point));
    }
}

void CommandSocket::sendProgress(float amount)
{
    socket.sendInt32(CMD_PROGRESS_REPORT);
    socket.sendInt32(4);
    socket.sendFloat32(float(current_object_number) / float(object_count) + amount / float(object_count));
}

void CommandSocket::sendPrintTimeForObject(int index, float print_time)
{
    socket.sendInt32(CMD_OBJECT_PRINT_TIME);
    socket.sendInt32(8);
    socket.sendInt32(index);
    socket.sendFloat32(print_time);
}

void CommandSocket::sendPrintMaterialForObject(int index, int extruder_nr, float print_time)
{
    socket.sendInt32(CMD_OBJECT_PRINT_MATERIAL);
    socket.sendInt32(12);
    socket.sendInt32(index);
    socket.sendInt32(extruder_nr);
    socket.sendFloat32(print_time);
}

}//namespace cura
