#include "utils/logoutput.h"
#include "commandSocket.h"
#include "fffProcessor.h"

#include <thread>
#include <cinttypes>

#include <Arcus/Socket.h>

#ifdef _WIN32
#include <windows.h>
#endif

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

    Cura::Layer* getLayerById(int id);

    fffProcessor* processor;

    Arcus::Socket* socket;

    int object_count;
    int current_object_number;

    std::shared_ptr<Cura::SlicedObjectList> slicedObjectList;
    Cura::SlicedObject* currentSlicedObject;
    int slicedObjects;
    std::vector<int64_t> objectIds;

    std::string tempGCodeFile;
    std::ostringstream gcode_output_stream;

    std::shared_ptr<PrintObject> objectToSlice;
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
    d->socket->registerMessageType(4, &Cura::GCodeLayer::default_instance());
    d->socket->registerMessageType(5, &Cura::ObjectPrintTime::default_instance());
    d->socket->registerMessageType(6, &Cura::SettingList::default_instance());
    d->socket->registerMessageType(7, &Cura::GCodePrefix::default_instance());

    d->socket->connect(ip, port);

    while(d->socket->state() != Arcus::SocketState::Closed && d->socket->state() != Arcus::SocketState::Error)
    {
        if(d->objectToSlice)
        {
            //TODO: Support all-at-once/one-at-a-time printing
            d->processor->processModel(d->objectToSlice.get());
            d->objectToSlice.reset();
            d->processor->resetFileNumber();

            sendPrintTime();
        }

        Arcus::MessagePtr message = d->socket->takeNextMessage();

        Cura::SettingList* settingList = dynamic_cast<Cura::SettingList*>(message.get());
        if(settingList)
        {
            handleSettingList(settingList);
        }

        Cura::ObjectList* objectList = dynamic_cast<Cura::ObjectList*>(message.get());
        if(objectList)
        {
            handleObjectList(objectList);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        if(!d->socket->errorString().empty()) {
            logError("%s\n", d->socket->errorString().data());
            d->socket->clearError();
        }
    }
}

void CommandSocket::handleObjectList(Cura::ObjectList* list)
{
    FMatrix3x3 matrix;
    d->object_count = 0;
    d->objectIds.clear();

    d->objectToSlice = std::make_shared<PrintObject>(d->processor);
    for(auto object : list->objects())
    {
        d->objectToSlice->meshes.emplace_back(d->objectToSlice.get()); //Construct a new mesh and put it into printObject's mesh list.
        Mesh& mesh = d->objectToSlice->meshes.back();

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

        for(auto setting : object.settings())
        {
            mesh.setSetting(setting.name(), setting.value());
        }

        d->objectIds.push_back(object.id());
        mesh.finish();
    }
    d->object_count++;
    d->objectToSlice->finalize();
}

void CommandSocket::handleSettingList(Cura::SettingList* list)
{
    for(auto setting : list->settings())
    {
        d->processor->setSetting(setting.name(), setting.value());
    }
}

void CommandSocket::sendLayerInfo(int layer_nr, int32_t z, int32_t height)
{
    if(!d->currentSlicedObject)
    {
        return;
    }

    Cura::Layer* layer = d->getLayerById(layer_nr);
    layer->set_height(z);
    layer->set_thickness(height);
}

void CommandSocket::sendPolygons(PolygonType type, int layer_nr, Polygons& polygons, int line_width)
{
    if(!d->currentSlicedObject)
        return;

    Cura::Layer* layer = d->getLayerById(layer_nr);

    for(unsigned int i = 0; i < polygons.size(); ++i)
    {
        Cura::Polygon* p = layer->add_polygons();
        p->set_type(static_cast<Cura::Polygon_Type>(type));
        std::string polydata;
        polydata.append(reinterpret_cast<const char*>(polygons[i].data()), polygons[i].size() * sizeof(Point));
        p->set_points(polydata);
        p->set_line_width(line_width);
    }
}

void CommandSocket::sendProgress(float amount)
{
    auto message = std::make_shared<Cura::Progress>();
    message->set_amount(amount);
    d->socket->sendMessage(message);
}

void CommandSocket::sendPrintTime()
{
    auto message = std::make_shared<Cura::ObjectPrintTime>();
    message->set_time(d->processor->getTotalPrintTime());
    message->set_material_amount(d->processor->getTotalFilamentUsed(0));
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
        d->currentSlicedObject = nullptr;
    }
}

void CommandSocket::beginGCode()
{
    d->processor->setTargetStream(&d->gcode_output_stream);
}

void CommandSocket::sendGCodeLayer()
{
    auto message = std::make_shared<Cura::GCodeLayer>();
    message->set_id(d->objectIds[0]);
    message->set_data(d->gcode_output_stream.str());
    d->socket->sendMessage(message);
    
    d->gcode_output_stream.str("");
}

void CommandSocket::sendGCodePrefix(std::string prefix)
{
    auto message = std::make_shared<Cura::GCodePrefix>();
    message->set_data(prefix);
    d->socket->sendMessage(message);
}

Cura::Layer* CommandSocket::Private::getLayerById(int id)
{
    auto itr = std::find_if(currentSlicedObject->mutable_layers()->begin(), currentSlicedObject->mutable_layers()->end(), [id](Cura::Layer& l) { return l.id() == id; });

    Cura::Layer* layer = nullptr;
    if(itr != currentSlicedObject->mutable_layers()->end())
    {
        layer = &(*itr);
    }
    else
    {
        layer = currentSlicedObject->add_layers();
        layer->set_id(id);
    }

    return layer;
}

}//namespace cura
