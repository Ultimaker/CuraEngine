#include "utils/logoutput.h"
#include "commandSocket.h"
#include "FffProcessor.h"
#include "progress/Progress.h"

#include <thread>
#include <cinttypes>

#ifdef ARCUS
#include <Arcus/Socket.h>
#include <Arcus/SocketListener.h>
#include <Arcus/Error.h>
#endif

#include <string> // stoi

#ifdef _WIN32
#include <windows.h>
#endif

#include "settings/SettingRegistry.h" // loadExtruderJSONsettings

#define DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR(x) 

// std::cerr << x;

namespace cura {

#define BYTES_PER_FLOAT 4
#define FLOATS_PER_VECTOR 3
#define VECTORS_PER_FACE 3

CommandSocket* CommandSocket::instance = nullptr; // instantiate instance

#ifdef ARCUS
class Listener : public Arcus::SocketListener
{
public:
    void stateChanged(Arcus::SocketState::SocketState newState) override
    {
    }

    void messageReceived() override
    {
    }

    void error(const Arcus::Error & error) override
    {
        if (error.getErrorCode() == Arcus::ErrorCode::Debug)
        {
            log("%s\n", error.toString().c_str());
        }
        else
        {
            logError("%s\n", error.toString().c_str());
        }
    }
};

class CommandSocket::Private
{
public:
    Private()
        : socket(nullptr)
        , object_count(0)
        , sliced_objects(0)
        , current_layer_count(0)
        , current_layer_offset(0)
    { }

    std::shared_ptr<cura::proto::Layer> getLayerById(int id);

    Arcus::Socket* socket;
    
    // Number of objects that need to be sliced
    int object_count;

    // Number of sliced objects for this sliced object list
    int sliced_objects;

    // Number of layers sent to the front end so far
    // Used for incrementing the current layer in one at a time mode
    int current_layer_count;
    int current_layer_offset;

    std::string temp_gcode_file;
    std::ostringstream gcode_output_stream;
    
    // Print object that olds one or more meshes that need to be sliced. 
    std::vector< std::shared_ptr<MeshGroup> > objects_to_slice;

    std::unordered_map<int, std::shared_ptr<cura::proto::Layer>> sliced_layers;
};
#endif

CommandSocket::CommandSocket()
#ifdef ARCUS
    : private_data(new Private)
#endif
{
#ifdef ARCUS
#endif
}

CommandSocket* CommandSocket::getInstance()
{
    return instance;
}

void CommandSocket::instantiate()
{
    instance = new CommandSocket();
}

bool CommandSocket::isInstantiated()
{
    return instance != nullptr;
}


void CommandSocket::connect(const std::string& ip, int port)
{
#ifdef ARCUS
    private_data->socket = new Arcus::Socket();
    private_data->socket->addListener(new Listener());

    //private_data->socket->registerMessageType(1, &Cura::ObjectList::default_instance());
    private_data->socket->registerMessageType(&cura::proto::Slice::default_instance());
    private_data->socket->registerMessageType(&cura::proto::Layer::default_instance());
    private_data->socket->registerMessageType(&cura::proto::Progress::default_instance());
    private_data->socket->registerMessageType(&cura::proto::GCodeLayer::default_instance());
    private_data->socket->registerMessageType(&cura::proto::ObjectPrintTime::default_instance());
    private_data->socket->registerMessageType(&cura::proto::SettingList::default_instance());
    private_data->socket->registerMessageType(&cura::proto::GCodePrefix::default_instance());
    private_data->socket->registerMessageType(&cura::proto::SlicingFinished::default_instance());

    private_data->socket->connect(ip, port);

    log("Connecting to %s:%i\n", ip.c_str(), port);

    while(private_data->socket->getState() != Arcus::SocketState::Connected && private_data->socket->getState() != Arcus::SocketState::Error)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log("Connected to %s:%i\n", ip.c_str(), port);
    
    bool slice_another_time = true;
    
    // Start & continue listening as long as socket is not closed and there is no error.
    while(private_data->socket->getState() != Arcus::SocketState::Closed && private_data->socket->getState() != Arcus::SocketState::Error && slice_another_time)
    {
        // Actually start handling messages.
        Arcus::MessagePtr message = private_data->socket->takeNextMessage();
        cura::proto::SettingList* setting_list = dynamic_cast<cura::proto::SettingList*>(message.get());
        if (setting_list)
        {
            handleSettingList(setting_list);
        }

        /*cura::proto::ObjectList* object_list = dynamic_cast<cura::proto::ObjectList*>(message.get());
        if (object_list)
        {
            handleObjectList(object_list);
        }*/
        
        cura::proto::Slice* slice = dynamic_cast<cura::proto::Slice*>(message.get());
        if (slice)
        {
            // Reset object counts
            private_data->object_count = 0;
            for (auto object : slice->object_lists())
            {
                handleObjectList(&object);
            }
        }

        //If there is an object to slice, do so.
        if (private_data->objects_to_slice.size())
        {
            FffProcessor::getInstance()->resetMeshGroupNumber();
            for (auto object : private_data->objects_to_slice)
            {
                if (!FffProcessor::getInstance()->processMeshGroup(object.get()))
                {
                    logError("Slicing mesh group failed!");
                }
            }
            private_data->objects_to_slice.clear();
            FffProcessor::getInstance()->finalize();
            flushGcode();
            sendPrintTime();
            sendFinishedSlicing();
            slice_another_time = false; // TODO: remove this when multiple slicing with CuraEngine is safe
            //TODO: Support all-at-once/one-at-a-time printing
            //private_data->processor->processModel(private_data->object_to_slice.get());
            //private_data->object_to_slice.reset();
            //private_data->processor->resetFileNumber();

            //sendPrintTime();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    log("Closing connection\n");
    private_data->socket->close();
#endif
}

#ifdef ARCUS
void CommandSocket::handleObjectList(cura::proto::ObjectList* list)
{
    if (list->objects_size() <= 0)
    {
        return;
    }

    FMatrix3x3 matrix;
    //private_data->object_count = 0;
    //private_data->object_ids.clear();
    private_data->objects_to_slice.push_back(std::make_shared<MeshGroup>(FffProcessor::getInstance()));
    MeshGroup* meshgroup = private_data->objects_to_slice.back().get();
    
    for (auto setting : list->settings())
    {
        meshgroup->setSetting(setting.name(), setting.value());
    }
    
    for (int extruder_nr = 0; extruder_nr < FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count"); extruder_nr++)
    { // initialize remaining extruder trains and load the defaults
        ExtruderTrain* train = meshgroup->createExtruderTrain(extruder_nr); // create new extruder train objects or use already existing ones
        SettingRegistry::getInstance()->loadExtruderJSONsettings(extruder_nr, train);
    }
    
    for (auto object : list->objects())
    {
        int bytes_per_face = BYTES_PER_FLOAT * FLOATS_PER_VECTOR * VECTORS_PER_FACE;
        int face_count = object.vertices().size() / bytes_per_face;

        if (face_count <= 0)
        {
            logWarning("Got an empty mesh, ignoring it!");
            continue;
        }
        DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("solid Cura_out\n");
        int extruder_train_nr = 0; // TODO: make primary extruder configurable!
        for (auto setting : object.settings())
        {
            if (setting.name() == "extruder_nr")
            {
                extruder_train_nr = std::stoi(setting.value());
                break;
            }
        }
        SettingsBase* extruder_train = meshgroup->getExtruderTrain(extruder_train_nr);

        meshgroup->meshes.push_back(extruder_train); //Construct a new mesh (with the corresponding extruder train as settings parent object) and put it into MeshGroup's mesh list.
        Mesh& mesh = meshgroup->meshes.back();

        for (int i = 0; i < face_count; ++i)
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
        for (auto setting : object.settings())
        {
            mesh.setSetting(setting.name(), setting.value());
        }

        mesh.finish();
    }

    private_data->object_count++;
    meshgroup->finalize();
}

void CommandSocket::handleSettingList(cura::proto::SettingList* list)
{
    for (auto setting : list->settings())
    {
        FffProcessor::getInstance()->setSetting(setting.name(), setting.value());
    }
}
#endif

void CommandSocket::sendLayerInfo(int layer_nr, int32_t z, int32_t height)
{
#ifdef ARCUS
    std::shared_ptr<cura::proto::Layer> layer = private_data->getLayerById(layer_nr);
    layer->set_height(z);
    layer->set_thickness(height);
#endif
}

void CommandSocket::sendPolygons(PrintFeatureType type, int layer_nr, Polygons& polygons, int line_width)
{
#ifdef ARCUS
    if (polygons.size() == 0)
        return;

    std::shared_ptr<cura::proto::Layer> proto_layer = private_data->getLayerById(layer_nr);

    for (unsigned int i = 0; i < polygons.size(); ++i)
    {
        cura::proto::Polygon* p = proto_layer->add_polygons();
        p->set_type(static_cast<cura::proto::Polygon_Type>(type));
        std::string polydata;
        polydata.append(reinterpret_cast<const char*>(polygons[i].data()), polygons[i].size() * sizeof(Point));
        p->set_points(polydata);
        p->set_line_width(line_width);
    }
#endif
}

void CommandSocket::sendProgress(float amount)
{
#ifdef ARCUS
    auto message = std::make_shared<cura::proto::Progress>();
    amount /= private_data->object_count;
    amount += private_data->sliced_objects * (1. / private_data->object_count);
    message->set_amount(amount);
    private_data->socket->sendMessage(message);
#endif
}

void CommandSocket::sendProgressStage(Progress::Stage stage)
{
    // TODO
}

void CommandSocket::sendPrintTime()
{
#ifdef ARCUS
    auto message = std::make_shared<cura::proto::ObjectPrintTime>();
    message->set_time(FffProcessor::getInstance()->getTotalPrintTime());
    message->set_material_amount(FffProcessor::getInstance()->getTotalFilamentUsed(0));
    private_data->socket->sendMessage(message);
#endif
}

void CommandSocket::sendPrintMaterialForObject(int index, int extruder_nr, float print_time)
{
//     socket.sendInt32(CMD_OBJECT_PRINT_MATERIAL);
//     socket.sendInt32(12);
//     socket.sendInt32(index);
//     socket.sendInt32(extruder_nr);
//     socket.sendFloat32(print_time);
}

void CommandSocket::sendLayerData()
{
#ifdef ARCUS
#endif
#ifdef ARCUS
    private_data->sliced_objects++;
    private_data->current_layer_offset = private_data->current_layer_count;
    log("End sliced object called. Sending ", private_data->current_layer_count, " layers.");

    if (private_data->sliced_objects >= private_data->object_count)
    {
        for (std::pair<const int, std::shared_ptr<cura::proto::Layer>> entry : private_data->sliced_layers) //Note: This is in no particular order!
        {
            private_data->socket->sendMessage(entry.second); //Send the actual layers.
        }
        private_data->sliced_objects = 0;
        private_data->current_layer_count = 0;
        private_data->current_layer_offset = 0;
        private_data->sliced_layers.clear();
        auto done_message = std::make_shared<cura::proto::SlicingFinished>();
        private_data->socket->sendMessage(done_message);
    }
#endif
}

void CommandSocket::sendFinishedSlicing()
{
#ifdef ARCUS
    std::shared_ptr<cura::proto::SlicingFinished> done_message = std::make_shared<cura::proto::SlicingFinished>();
    private_data->socket->sendMessage(done_message);
#endif
}

void CommandSocket::beginGCode()
{
#ifdef ARCUS
    FffProcessor::getInstance()->setTargetStream(&private_data->gcode_output_stream);
#endif
}

void CommandSocket::flushGcode()
{
#ifdef ARCUS
    auto message = std::make_shared<cura::proto::GCodeLayer>();
    message->set_data(private_data->gcode_output_stream.str());
    private_data->socket->sendMessage(message);
    
    private_data->gcode_output_stream.str("");
#endif
}

void CommandSocket::sendGCodePrefix(std::string prefix)
{
#ifdef ARCUS
    auto message = std::make_shared<cura::proto::GCodePrefix>();
    message->set_data(prefix);
    private_data->socket->sendMessage(message);
#endif
}

#ifdef ARCUS
std::shared_ptr<cura::proto::Layer> CommandSocket::Private::getLayerById(int id)
{
    id += current_layer_offset;

    auto itr = sliced_layers.find(id);

    std::shared_ptr<cura::proto::Layer> layer;
    if (itr != sliced_layers.end())
    {
        layer = itr->second;
    }
    else
    {
        layer = std::make_shared<cura::proto::Layer>();
        layer->set_id(id);
        current_layer_count++;
        sliced_layers[id] = layer;
    }

    return layer;
}
#endif

}//namespace cura
