#include "utils/logoutput.h"
#include "commandSocket.h"
#include "fffProcessor.h"
#include "Progress.h"

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
        , current_sliced_object(nullptr)
        , sliced_objects(0)
    { }

    fffProcessor* processor;

    Arcus::Socket* socket;

    int object_count;
    int current_object_number;

    std::shared_ptr<Cura::SlicedObjectList> sliced_object_list;
    Cura::SlicedObject* current_sliced_object;
    int sliced_objects;
    std::vector<int64_t> object_ids;

    std::string temp_gcode_file;
    std::ostringstream gcode_output_stream;

    std::shared_ptr<PrintObject> object_to_slice;
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
        if(d->object_to_slice)
        {
            //TODO: Support all-at-once/one-at-a-time printing
            d->processor->processModel(d->object_to_slice.get());
            d->object_to_slice.reset();
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
    d->object_ids.clear();

    d->object_to_slice = std::make_shared<PrintObject>(d->processor);
    for(auto object : list->objects())
    {
        d->object_to_slice->meshes.emplace_back(d->object_to_slice.get()); //Construct a new mesh and put it into printObject's mesh list.
        Mesh& mesh = d->object_to_slice->meshes.back();

        int bytes_per_face = BYTES_PER_FLOAT * FLOATS_PER_VECTOR * VECTORS_PER_FACE;
        int face_count = object.vertices().size() / bytes_per_face;
        for(int i = 0; i < face_count; ++i)
        {
            //TODO: Apply matrix
            std::string data = object.vertices().substr(i * bytes_per_face, bytes_per_face);
            const FPoint3* float_vertices = reinterpret_cast<const FPoint3*>(data.data());

            Point3 verts[3];
            verts[0] = matrix.apply(float_vertices[0]);
            verts[1] = matrix.apply(float_vertices[1]);
            verts[2] = matrix.apply(float_vertices[2]);
            mesh.addFace(verts[0], verts[1], verts[2]);
        }

        for(auto setting : object.settings())
        {
            mesh.setSetting(setting.name(), setting.value());
        }

        d->object_ids.push_back(object.id());
        mesh.finish();
    }
    d->object_count++;
    d->object_to_slice->finalize();
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
//     socket.sendInt32(CMD_LAYER_INFO);
//     socket.sendInt32(4 * 4);
//     socket.sendInt32(current_object_number);
//     socket.sendInt32(layer_nr);
//     socket.sendInt32(z);
//     socket.sendInt32(height);
}

void CommandSocket::sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
{
    if(!d->current_sliced_object)
        return;

    Cura::Layer* layer = d->current_sliced_object->add_layers();
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

void CommandSocket::sendProgressStage(Progress::Stage stage)
{
    // TODO
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
    if(!d->sliced_object_list)
    {
        d->sliced_object_list = std::make_shared<Cura::SlicedObjectList>();
    }

    d->current_sliced_object = d->sliced_object_list->add_objects();
    d->current_sliced_object->set_id(d->object_ids[d->sliced_objects]);
}

void CommandSocket::endSendSlicedObject()
{
    d->sliced_objects++;
    if(d->sliced_objects >= d->object_count)
    {
        d->socket->sendMessage(d->sliced_object_list);
        d->sliced_objects = 0;
        d->sliced_object_list.reset();
        d->current_sliced_object = nullptr;
    }
}

void CommandSocket::beginGCode()
{
    d->processor->setTargetStream(&d->gcode_output_stream);
}

void CommandSocket::sendGCodeLayer()
{
    auto message = std::make_shared<Cura::GCodeLayer>();
    message->set_id(d->object_ids[0]);
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

}//namespace cura
