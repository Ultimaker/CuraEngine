#include "utils/logoutput.h"
#include "commandSocketArcus.h"

#include <thread>
#include <cinttypes>

#include <string> // stoi

#ifdef _WIN32
#include <windows.h>
#endif

#define DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR(x)
// std::cerr << x;

namespace cura {

#define BYTES_PER_FLOAT 4
#define FLOATS_PER_VECTOR 3
#define VECTORS_PER_FACE 3

class ArcusErrorListener : public Arcus::SocketListener
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
        if(error.getErrorCode() == Arcus::ErrorCode::Debug)
        {
            log("%s\n", error.toString().c_str());
        }
        else
        {
            logError("%s\n", error.toString().c_str());
        }
    }
};

void CommandSocketArcus::connect(const std::string& ip, int port)
{
    socket_ = new Arcus::Socket();
    socket_->addListener(new ArcusErrorListener());

    //socket_->registerMessageType(1, &Cura::ObjectList::default_instance());
    socket_->registerMessageType(&cura::proto::Slice::default_instance());
    socket_->registerMessageType(&cura::proto::SlicedObjectList::default_instance());
    socket_->registerMessageType(&cura::proto::Progress::default_instance());
    socket_->registerMessageType(&cura::proto::GCodeLayer::default_instance());
    socket_->registerMessageType(&cura::proto::ObjectPrintTime::default_instance());
    socket_->registerMessageType(&cura::proto::SettingList::default_instance());
    socket_->registerMessageType(&cura::proto::GCodePrefix::default_instance());
    socket_->registerMessageType(&cura::proto::SlicingFinished::default_instance());

    socket_->connect(ip, port);

    log("Connecting to %s:%i\n", ip.c_str(), port);

    while(socket_->getState() != Arcus::SocketState::Connected && socket_->getState() != Arcus::SocketState::Error)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log("Connected to %s:%i\n", ip.c_str(), port);

    bool slice_another_time = true;

    // Start & continue listening as long as socket is not closed and there is no error.
    while(socket_->getState() != Arcus::SocketState::Closed && socket_->getState() != Arcus::SocketState::Error && slice_another_time)
    {
        // Actually start handling messages.
        Arcus::MessagePtr message = socket_->takeNextMessage();
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
            object_count_ = 0;
            object_ids_.clear();
            for(auto object : slice->object_lists())
            {
                handleObjectList(&object);
            }
        }

        //If there is an object to slice, do so.
        if(objects_to_slice_.size())
        {
            FffProcessor::getInstance()->resetFileNumber();
            for(auto object : objects_to_slice_)
            {
                if(!FffProcessor::getInstance()->processMeshGroup(object.get()))
                {
                    logError("Slicing mesh group failed!");
                }
            }
            objects_to_slice_.clear();
            FffProcessor::getInstance()->finalize();
            flushGcode();
            sendPrintTime();
            sendFinishedSlicing();
            slice_another_time = false; // TODO: remove this when multiple slicing with CuraEngine is safe
            //TODO: Support all-at-once/one-at-a-time printing
            //processor_->processModel(private_data->object_to_slice.get());
            //object_to_slice_.reset();
            //processor_->resetFileNumber();

            //sendPrintTime();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    socket_->close();
}

CommandSocketArcus::CommandSocketArcus()
    : socket_()
    , object_count_()
    , current_sliced_object_()
    , sliced_objects_()
    , current_layer_count_()
    , current_layer_offset_()
    { }

void CommandSocketArcus::handleObjectList(cura::proto::ObjectList* list)
{
    if(list->objects_size() <= 0)
    {
        return;
    }

    FMatrix3x3 matrix;
    //private_data->object_count = 0;
    //private_data->object_ids.clear();
    objects_to_slice_.push_back(std::make_shared<MeshGroup>(FffProcessor::getInstance()));
    MeshGroup* meshgroup = objects_to_slice_.back().get();

    for(auto setting : list->settings())
    {
        meshgroup->setSetting(setting.name(), setting.value());
    }

    for (int extruder_nr = 0; extruder_nr < FffProcessor::getInstance()->getSettingAsCount("machine_extruder_count"); extruder_nr++)
    { // initialize remaining extruder trains and load the defaults
        meshgroup->createExtruderTrain(extruder_nr)->setExtruderTrainDefaults(extruder_nr); // create new extruder train objects or use already existing ones
    }

    for(auto object : list->objects())
    {
        int bytes_per_face = BYTES_PER_FLOAT * FLOATS_PER_VECTOR * VECTORS_PER_FACE;
        int face_count = object.vertices().size() / bytes_per_face;

        if(face_count <= 0)
        {
            logWarning("Got an empty mesh, ignoring it!");
            continue;
        }
        DEBUG_OUTPUT_OBJECT_STL_THROUGH_CERR("solid Cura_out\n");
        int extruder_train_nr = 0; // TODO: make primary extruder configurable!
        for(auto setting : object.settings())
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

        object_ids_.push_back(object.id());
        mesh.finish();
    }

    object_count_++;
    meshgroup->finalize();
}

void CommandSocketArcus::handleSettingList(cura::proto::SettingList* list)
{
    for(auto setting : list->settings())
    {
        FffProcessor::getInstance()->setSetting(setting.name(), setting.value());
    }
}

void CommandSocketArcus::sendLayerInfo(int layer_nr, int32_t z, int32_t height)
{
    if(!current_sliced_object_)
    {
        return;
    }

    cura::proto::Layer* layer = getLayerById(layer_nr);
    layer->set_height(z);
    layer->set_thickness(height);
}

void CommandSocketArcus::sendPolygons(PrintFeatureType type, int layer_nr, Polygons& polygons, int line_width)
{
    if(!current_sliced_object_)
        return;

    if (polygons.size() == 0)
        return;

    cura::proto::Layer* proto_layer = getLayerById(layer_nr);

    for(unsigned int i = 0; i < polygons.size(); ++i)
    {
        cura::proto::Polygon* p = proto_layer->add_polygons();
        p->set_type(static_cast<cura::proto::Polygon_Type>(type));
        std::string polydata;
        polydata.append(reinterpret_cast<const char*>(polygons[i].data()), polygons[i].size() * sizeof(Point));
        p->set_points(polydata);
        p->set_line_width(line_width);
    }
}

void CommandSocketArcus::sendProgress(float amount)
{
    auto message = std::make_shared<cura::proto::Progress>();
    amount /= object_count_;
    amount += sliced_objects_ * (1. / object_count_);
    message->set_amount(amount);
    socket_->sendMessage(message);
}

void CommandSocketArcus::sendProgressStage(Progress::Stage stage)
{
    // TODO
}

void CommandSocketArcus::sendPrintTime()
{
    auto message = std::make_shared<cura::proto::ObjectPrintTime>();
    message->set_time(FffProcessor::getInstance()->getTotalPrintTime());
    message->set_material_amount(FffProcessor::getInstance()->getTotalFilamentUsed(0));
    socket_->sendMessage(message);
}

void CommandSocketArcus::sendPrintMaterialForObject(int index, int extruder_nr, float print_time)
{
    //     socket_.sendInt32(CMD_OBJECT_PRINT_MATERIAL);
    //     socket_.sendInt32(12);
    //     socket_.sendInt32(index);
    //     socket_.sendInt32(extruder_nr);
    //     socket_.sendFloat32(print_time);
}

void CommandSocketArcus::beginSendSlicedObject()
{
    if(!sliced_object_list_)
    {
        sliced_object_list_ = std::make_shared<cura::proto::SlicedObjectList>();
    }

    current_sliced_object_ = sliced_object_list_->add_objects();
    current_sliced_object_->set_id(object_ids_[sliced_objects_]);
}

void CommandSocketArcus::endSendSlicedObject()
{
    sliced_objects_++;
    current_layer_offset_ = current_layer_count_;
    std::cout << "End sliced object called. Sliced objects " << sliced_objects_ << " object count: " << object_count_ << std::endl;

    if(sliced_objects_ >= object_count_)
    {
        socket_->sendMessage(sliced_object_list_);
        sliced_objects_ = 0;
        current_layer_count_ = 0;
        current_layer_offset_ = 0;
        sliced_object_list_.reset();
        current_sliced_object_ = nullptr;
        auto done_message = std::make_shared<cura::proto::SlicingFinished>();
        socket_->sendMessage(done_message);
    }
}

void CommandSocketArcus::sendFinishedSlicing()
{
    std::shared_ptr<cura::proto::SlicingFinished> done_message = std::make_shared<cura::proto::SlicingFinished>();
    socket_->sendMessage(done_message);
}

void CommandSocketArcus::beginGCode()
{
    FffProcessor::getInstance()->setTargetStream(&gcode_output_stream_);
}

void CommandSocketArcus::flushGcode()
{
    auto message = std::make_shared<cura::proto::GCodeLayer>();
    message->set_id(object_ids_[0]);
    message->set_data(gcode_output_stream_.str());
    socket_->sendMessage(message);

    gcode_output_stream_.str("");
}

void CommandSocketArcus::sendGCodePrefix(std::string prefix)
{
    auto message = std::make_shared<cura::proto::GCodePrefix>();
    message->set_data(prefix);
    socket_->sendMessage(message);
}


cura::proto::Layer* CommandSocketArcus::getLayerById(int id)
{
    id += current_layer_offset_;

    auto mutable_layers = current_sliced_object_->mutable_layers();
    auto itr = std::find_if(mutable_layers->begin(), mutable_layers->end(), [id](cura::proto::Layer& l) { return l.id() == id; });

    cura::proto::Layer* layer = nullptr;
    if(itr != current_sliced_object_->mutable_layers()->end())
    {
        layer = &(*itr);
    }
    else
    {
        layer = current_sliced_object_->add_layers();
        layer->set_id(id);
        current_layer_count_++;
    }

    return layer;
}


}//namespace cura
