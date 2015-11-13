#include "utils/logoutput.h"
#include "commandSocket.h"
#include "FffProcessor.h"
#include "Progress.h"

#include <thread>
#include <cinttypes>

#include <Arcus/Socket.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR(x) 
// std::cerr << x;

namespace cura {

#define BYTES_PER_FLOAT 4
#define FLOATS_PER_VECTOR 3
#define VECTORS_PER_FACE 3
    
class CommandSocket::Private
{
public:
    Private()
        : socket(nullptr)
        , object_count(0)
        , current_sliced_object(nullptr)
        , sliced_objects(0)
        , current_layer_count(0)
        , current_layer_offset(0)
    { }

    cura::proto::Layer* getLayerById(int id);

    Arcus::Socket* socket;
    
    // Number of objects that need to be sliced
    int object_count;
    
    // Message that holds a list of sliced objects
    std::shared_ptr<cura::proto::SlicedObjectList> sliced_object_list;
    
    // Message that holds the currently sliced object (to be added to sliced_object_list)
    cura::proto::SlicedObject* current_sliced_object;
    
    // Number of sliced objects for this sliced object list
    int sliced_objects;

    // Number of layers sent to the front end so far
    // Used for incrementing the current layer in one at a time mode
    int current_layer_count;
    int current_layer_offset;
    
    // Ids of the sliced objects
    std::vector<int64_t> object_ids;

    std::string temp_gcode_file;
    std::ostringstream gcode_output_stream;
    
    // Print object that olds one or more meshes that need to be sliced. 
    std::vector< std::shared_ptr<MeshGroup> > objects_to_slice;
};

CommandSocket::CommandSocket()
    : d(new Private)
{
    FffProcessor::getInstance()->setCommandSocket(this);
}

void CommandSocket::connect(const std::string& ip, int port)
{
    d->socket = new Arcus::Socket();
    //d->socket->registerMessageType(1, &Cura::ObjectList::default_instance());
    d->socket->registerMessageType(1, &cura::proto::Slice::default_instance());
    d->socket->registerMessageType(2, &cura::proto::SlicedObjectList::default_instance());
    d->socket->registerMessageType(3, &cura::proto::Progress::default_instance());
    d->socket->registerMessageType(4, &cura::proto::GCodeLayer::default_instance());
    d->socket->registerMessageType(5, &cura::proto::ObjectPrintTime::default_instance());
    d->socket->registerMessageType(6, &cura::proto::SettingList::default_instance());
    d->socket->registerMessageType(7, &cura::proto::GCodePrefix::default_instance());

    d->socket->connect(ip, port);
    
    // Start & continue listening as long as socket is not closed and there is no error.
    while(d->socket->state() != Arcus::SocketState::Closed && d->socket->state() != Arcus::SocketState::Error)
    {
        //If there is an object to slice, do so.
        if(d->objects_to_slice.size())
        {
            FffProcessor::getInstance()->resetFileNumber();
            for(auto object : d->objects_to_slice)
            {
                if(!FffProcessor::getInstance()->processMeshGroup(object.get()))
                {
                    logError("Slicing mesh group failed!");
                }
            }
            d->objects_to_slice.clear();
            FffProcessor::getInstance()->finalize();
            sendGCodeLayer();
            sendPrintTime();
            //TODO: Support all-at-once/one-at-a-time printing
            //d->processor->processModel(d->object_to_slice.get());
            //d->object_to_slice.reset();
            //d->processor->resetFileNumber();

            //sendPrintTime();
        }
        
        // Actually start handling messages.
        Arcus::MessagePtr message = d->socket->takeNextMessage();
        cura::proto::SettingList* setting_list = dynamic_cast<cura::proto::SettingList*>(message.get());
        if(setting_list)
        {
            handleSettingList(setting_list);
        }

        /*cura::proto::ObjectList* object_list = dynamic_cast<cura::proto::ObjectList*>(message.get());
        if(object_list)
        {
            handleObjectList(object_list);
        }*/
        
        cura::proto::Slice* slice = dynamic_cast<cura::proto::Slice*>(message.get());
        if(slice)
        {
            // Reset object counts
            d->object_count = 0;
            d->object_ids.clear();
            for(auto object : slice->object_lists())
            {
                handleObjectList(&object);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        if(!d->socket->errorString().empty()) 
        {
            logError("%s\n", d->socket->errorString().data());
            d->socket->clearError();
        }
    }
}

void CommandSocket::handleObjectList(cura::proto::ObjectList* list)
{
    FMatrix3x3 matrix;
    //d->object_count = 0;
    //d->object_ids.clear();
    d->objects_to_slice.push_back(std::make_shared<MeshGroup>(FffProcessor::getInstance()));
    MeshGroup* object_to_slice = d->objects_to_slice.back().get();
    for(auto object : list->objects())
    {
        DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("solid Cura_out\n");
        object_to_slice->meshes.push_back(object_to_slice); //Construct a new mesh (with object_to_slice as settings parent object) and put it into MeshGroup's mesh list.
        Mesh& mesh = object_to_slice->meshes.back();

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
            
            
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("  facet normal -1 0 0\n");
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("    outer loop\n");
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("      vertex "<<INT2MM(verts[0].x) <<" " << INT2MM(verts[0].y) <<" " << INT2MM(verts[0].z) << "\n");
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("      vertex "<<INT2MM(verts[1].x) <<" " << INT2MM(verts[1].y) <<" " << INT2MM(verts[1].z) << "\n");
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("      vertex "<<INT2MM(verts[2].x) <<" " << INT2MM(verts[2].y) <<" " << INT2MM(verts[2].z) << "\n");
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("    endloop\n");
            DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("  endfacet\n");
        }
        DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("endsolid Cura_out\n");
        for(auto setting : object.settings())
        {
            mesh.setSetting(setting.name(), setting.value());
        }

        d->object_ids.push_back(object.id());
        mesh.finish();
    }

    for(auto setting : list->settings())
    {
        object_to_slice->setSetting(setting.name(), setting.value());
    }

    d->object_count++;
    object_to_slice->finalize();
}

void CommandSocket::handleSettingList(cura::proto::SettingList* list)
{
    for(auto setting : list->settings())
    {
        FffProcessor::getInstance()->setSetting(setting.name(), setting.value());
    }
}

void CommandSocket::sendLayerInfo(int layer_nr, int32_t z, int32_t height)
{
    if(!d->current_sliced_object)
    {
        return;
    }
    
    cura::proto::Layer* layer = d->getLayerById(layer_nr);
    layer->set_height(z);
    layer->set_thickness(height);
}

void CommandSocket::sendPolygons(PolygonType type, int layer_nr, Polygons& polygons, int line_width)
{
    if(!d->current_sliced_object)
        return;
    
    if (polygons.size() == 0)
        return;

    cura::proto::Layer* layer = d->getLayerById(layer_nr);

    for(unsigned int i = 0; i < polygons.size(); ++i)
    {
        cura::proto::Polygon* p = layer->add_polygons();
        p->set_type(static_cast<cura::proto::Polygon_Type>(type));
        std::string polydata;
        polydata.append(reinterpret_cast<const char*>(polygons[i].data()), polygons[i].size() * sizeof(Point));
        p->set_points(polydata);
        p->set_line_width(line_width);
    }
}

void CommandSocket::sendProgress(float amount)
{
    auto message = std::make_shared<cura::proto::Progress>();
    amount /= d->object_count;
    amount += d->sliced_objects * (1. / d->object_count);
    message->set_amount(amount);
    d->socket->sendMessage(message);
}

void CommandSocket::sendProgressStage(Progress::Stage stage)
{
    // TODO
}

void CommandSocket::sendPrintTime()
{
    auto message = std::make_shared<cura::proto::ObjectPrintTime>();
    message->set_time(FffProcessor::getInstance()->getTotalPrintTime());
    message->set_material_amount(FffProcessor::getInstance()->getTotalFilamentUsed(0));
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
        d->sliced_object_list = std::make_shared<cura::proto::SlicedObjectList>();
    }

    d->current_sliced_object = d->sliced_object_list->add_objects();
    d->current_sliced_object->set_id(d->object_ids[d->sliced_objects]);
}

void CommandSocket::endSendSlicedObject()
{
    d->sliced_objects++;
    d->current_layer_offset = d->current_layer_count;
    std::cout << "End sliced object called. sliced objects " << d->sliced_objects << " object count: " << d->object_count << std::endl;

    std::cout << "current layer count" << d->current_layer_count << std::endl;
    std::cout << "current layer offset" << d->current_layer_offset << std::endl;

    if(d->sliced_objects >= d->object_count)
    {
        d->socket->sendMessage(d->sliced_object_list);
        d->sliced_objects = 0;
        d->current_layer_count = 0;
        d->current_layer_offset = 0;
        d->sliced_object_list.reset();
        d->current_sliced_object = nullptr;
    }
}

void CommandSocket::beginGCode()
{
    FffProcessor::getInstance()->setTargetStream(&d->gcode_output_stream);
}

void CommandSocket::sendGCodeLayer()
{
    auto message = std::make_shared<cura::proto::GCodeLayer>();
    message->set_id(d->object_ids[0]);
    message->set_data(d->gcode_output_stream.str());
    d->socket->sendMessage(message);
    
    d->gcode_output_stream.str("");
}

void CommandSocket::sendGCodePrefix(std::string prefix)
{
    auto message = std::make_shared<cura::proto::GCodePrefix>();
    message->set_data(prefix);
    d->socket->sendMessage(message);
}

cura::proto::Layer* CommandSocket::Private::getLayerById(int id)
{
    id += current_layer_offset;

    auto itr = std::find_if(current_sliced_object->mutable_layers()->begin(), current_sliced_object->mutable_layers()->end(), [id](cura::proto::Layer& l) { return l.id() == id; });

    cura::proto::Layer* layer = nullptr;
    if(itr != current_sliced_object->mutable_layers()->end())
    {
        layer = &(*itr);
    }
    else
    {
        layer = current_sliced_object->add_layers();
        layer->set_id(id);
        current_layer_count++;
    }

    return layer;
}

}//namespace cura
