#include "utils/logoutput.h"
#include "commandSocket.h"
#include "fffProcessor.h"

#include <thread>

#include <Arcus/Socket.h>

namespace cura {

#define BYTES_PER_FLOAT 4
#define FLOATS_PER_VECTOR 3
#define VECTORS_PER_FACE 3

class CommandSocket::Private
{
public:
    Private()
        : processor(nullptr)
        , socket(nullptr)
        , object_count(0)
        , current_object_number(0)
        , currentSlicedObject(nullptr)
        , slicedObjects(0)
    { }

    fffProcessor* processor;

    Arcus::Socket* socket;

    int object_count;
    int current_object_number;

    std::shared_ptr<Cura::SlicedObjectList> slicedObjectList;
    Cura::SlicedObject* currentSlicedObject;
    int slicedObjects;
    std::vector<long> objectIds;

    std::string tempGCodeFile;
};

CommandSocket::CommandSocket(fffProcessor* processor)
    : d(new Private)
{
    d->processor = processor;
    d->processor->setCommandSocket(this);
}

void CommandSocket::connect(const std::string& ip, int port)
{
    d->socket = new Arcus::Socket();
    d->socket->registerMessageType(1, &Cura::ObjectList::default_instance());
    d->socket->registerMessageType(2, &Cura::SlicedObjectList::default_instance());
    d->socket->registerMessageType(3, &Cura::Progress::default_instance());
    d->socket->registerMessageType(4, &Cura::GCode::default_instance());
    d->socket->registerMessageType(5, &Cura::ObjectPrintTime::default_instance());

    d->socket->connect(ip, port);

    while(d->socket->state() != Arcus::SocketState::Closed && d->socket->state() != Arcus::SocketState::Error)
    {
        Arcus::MessagePtr message = d->socket->takeNextMessage();

        Cura::ObjectList* objectList = dynamic_cast<Cura::ObjectList*>(message.get());
        if(objectList)
        {
            handleObjectList(objectList);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    logError("%s\n", d->socket->errorString().data());
}

void CommandSocket::handleObjectList(Cura::ObjectList* list)
{
    FMatrix3x3 matrix;
    d->object_count = 0;
    d->objectIds.clear();

    auto printObject = new PrintObject{d->processor};
    for(auto object : list->objects())
    {
        printObject->meshes.emplace_back(printObject); //Construct a new mesh and put it into printObject's mesh list.
        Mesh& mesh = printObject->meshes.back();

        int bytesPerFace = BYTES_PER_FLOAT * FLOATS_PER_VECTOR * VECTORS_PER_FACE;
        int faceCount = object.vertices().size() / bytesPerFace;
        for(int i = 0; i < faceCount; ++i)
        {
            //TODO: Apply matrix
            std::string data = object.vertices().substr(i * bytesPerFace, bytesPerFace);
            const FPoint3* floatVerts = reinterpret_cast<const FPoint3*>(data.data());

            Point3 verts[3];
            verts[0] = matrix.apply(floatVerts[0]);
            verts[1] = matrix.apply(floatVerts[1]);
            verts[2] = matrix.apply(floatVerts[2]);
            mesh.addFace(verts[0], verts[1], verts[2]);
        }

        d->object_count++;
        d->objectIds.push_back(object.id());
        mesh.finish();
    }
    printObject->finalize();

    //TODO: Support all-at-once/one-at-a-time printing
    d->processor->processModel(printObject);

    delete printObject;
}

void CommandSocket::sendLayerInfo(int layer_nr, int32_t z, int32_t height)
{
//     socket.sendInt32(CMD_LAYER_INFO);
//     socket.sendInt32(4 * 4);
//     socket.sendInt32(current_object_number);
//     socket.sendInt32(layer_nr);
//     socket.sendInt32(z);
//     socket.sendInt32(height);
}

void CommandSocket::sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
{
    if(!d->currentSlicedObject)
        return;

    Cura::Layer* layer = d->currentSlicedObject->add_layers();
    layer->set_id(layer_nr);

    for(unsigned int i = 0; i < polygons.size(); ++i)
    {
        Cura::Polygon* p = layer->add_polygons();
        p->set_type(static_cast<Cura::Polygon_Type>(type));
        std::string polydata;
        polydata.append(reinterpret_cast<const char*>(polygons[i].data()), polygons[i].size() * sizeof(Point));
        p->set_points(polydata);
    }
}

void CommandSocket::sendProgress(float amount)
{
    auto message = std::make_shared<Cura::Progress>();
    message->set_amount(amount);
    d->socket->sendMessage(message);
}

void CommandSocket::sendPrintTimeForObject(int index, float print_time)
{
    auto message = std::make_shared<Cura::ObjectPrintTime>();
    message->set_id(index);
    message->set_time(print_time);
    d->socket->sendMessage(message);
}

void CommandSocket::sendPrintMaterialForObject(int index, int extruder_nr, float print_time)
{
//     socket.sendInt32(CMD_OBJECT_PRINT_MATERIAL);
//     socket.sendInt32(12);
//     socket.sendInt32(index);
//     socket.sendInt32(extruder_nr);
//     socket.sendFloat32(print_time);
}

void CommandSocket::beginSendSlicedObject()
{
    if(!d->slicedObjectList)
    {
        d->slicedObjectList = std::make_shared<Cura::SlicedObjectList>();
    }

    d->currentSlicedObject = d->slicedObjectList->add_objects();
    d->currentSlicedObject->set_id(d->objectIds[d->slicedObjects]);
}

void CommandSocket::endSendSlicedObject()
{
    d->slicedObjects++;
    if(d->slicedObjects >= d->object_count)
    {
        d->socket->sendMessage(d->slicedObjectList);
        d->slicedObjects = 0;
        d->slicedObjectList.reset();
    }
}

void CommandSocket::beginGCode()
{
    //TODO: This is a massive hack to work around the fact that GCodeExport can only deal with
    //FILE* at the moment. We really should rewrite that, but effort...
#ifdef __GNUC__
#warning This is a very very ugly hack
#endif
    d->tempGCodeFile = tmpnam(nullptr);
    d->processor->setTargetFile(d->tempGCodeFile.c_str());
}

void CommandSocket::endGCode()
{
    auto message = std::make_shared<Cura::GCode>();
    message->set_filename(d->tempGCodeFile);
    d->socket->sendMessage(message);
}

}//namespace cura
