//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifdef ARCUS

#include <Arcus/Socket.h> //The socket to communicate to.
#include <sstream> //For ostringstream.
#include <thread> //To sleep while waiting for the connection.

#include "ArcusCommunication.h"
#include "Cura.pb.h" //To create Protobuf messages for Cura's front-end.
#include "Listener.h" //To listen to the Arcus socket.
#include "SliceDataStruct.h" //To store sliced layer data.
#include "../FffProcessor.h" //To start a slice.
#include "../PrintFeature.h"
#include "../Slice.h" //To process slices.
#include "../utils/logoutput.h"

namespace cura
{

//Forward declarations for compilation speed.
class MeshGroup;

class ArcusCommunication::Private
{
public:
    Private()
        : socket(nullptr)
        , object_count(0)
        , last_sent_progress(-1)
    { }

    /*
     * Get the unoptimised layer data for a specific layer.
     * \param layer_nr The layer number to get the layer data for.
     * \return The layer data for that layer.
     */
    std::shared_ptr<cura::proto::Layer> getLayerById(LayerIndex layer_nr);

    /*
     * Get the optimised layer data for a specific layer.
     * \param layer_nr The layer number to get the optimised layer data for.
     * \return The optimised layer data for that layer.
     */
    std::shared_ptr<cura::proto::LayerOptimized> getOptimizedLayerById(LayerIndex layer_nr);

    Arcus::Socket* socket; //!< Socket to send data to.
    size_t object_count; //!< Number of objects that need to be sliced.
    std::string temp_gcode_file; //!< Temporary buffer for the g-code.
    std::ostringstream gcode_output_stream; //!< The stream to write g-code to.
    std::vector<std::shared_ptr<MeshGroup>> objects_to_slice; //!< Print object that holds one or more meshes that need to be sliced.

    SliceDataStruct<cura::proto::Layer> sliced_layers;
    SliceDataStruct<cura::proto::LayerOptimized> optimized_layers;

    int last_sent_progress; //!< Last sent progress promille (1/1000th). Used to not send duplicate messages with the same promille.

    /*
     * \brief How often we've sliced so far during this run of CuraEngine.
     *
     * This is currently used to limit the number of slices per run to 1,
     * because CuraEngine produced different output for each slice. The fix was
     * to restart CuraEngine every time you make a slice.
     *
     * Once this bug is resolved, we can allow multiple slices for each run. Our
     * intuition says that there might be some differences if we let stuff
     * depend on the order of iteration in unordered_map or unordered_set,
     * because those data structures will give a different order if more memory
     * has already been reserved for them.
     */
    size_t slice_count; //!< How often we've sliced so far during this run of CuraEngine.
};

class ArcusCommunication::PathCompiler
{
    typedef cura::proto::PathSegment::PointType PointType;
    static_assert(sizeof(PrintFeatureType) == 1, "To be compatible with the Cura frontend code PrintFeatureType needs to be of size 1");
    //! Reference to the private data of the CommandSocket used to send the data to the front end.
    ArcusCommunication::Private& _cs_private_data;
    //! Keeps track of the current layer number being processed. If layer number is set to a different value, the current data is flushed to CommandSocket.
    int _layer_nr;
    int extruder;
    PointType data_point_type;

    std::vector<PrintFeatureType> line_types; //!< Line types for the line segments stored, the size of this vector is N.
    std::vector<float> line_widths; //!< Line widths for the line segments stored, the size of this vector is N.
    std::vector<float> line_thicknesses; //!< Line thicknesses for the line segments stored, the size of this vector is N.
    std::vector<float> line_feedrates; //!< Line feedrates for the line segments stored, the size of this vector is N.
    std::vector<float> points; //!< The points used to define the line segments, the size of this vector is D*(N+1) as each line segment is defined from one point to the next. D is the dimensionality of the point.

    Point last_point;

    PathCompiler(const PathCompiler&) = delete;
    PathCompiler& operator=(const PathCompiler&) = delete;
public:
    PathCompiler(ArcusCommunication::Private& cs_private_data):
        _cs_private_data(cs_private_data),
        _layer_nr(0),
        extruder(0),
        data_point_type(cura::proto::PathSegment::Point2D),
        line_types(),
        line_widths(),
        line_thicknesses(),
        line_feedrates(),
        points(),
        last_point{0,0}
    {}
    ~PathCompiler()
    {
        if (line_types.size())
        {
            flushPathSegments();
        }
    }

    /*!
     * Used to select which layer the following layer data is intended for.
     */
    void setLayer(int new_layer_nr)
    {
        if (_layer_nr != new_layer_nr)
        {
            flushPathSegments();
            _layer_nr = new_layer_nr;
        }
    }
    /*!
     * Returns the current layer which data is written to.
     */
    int getLayer() const
    {
        return _layer_nr;
    }
    /*!
     * Used to set which extruder will be used for printing the following layer data is intended for.
     */
    void setExtruder(int new_extruder)
    {
        if (extruder != new_extruder)
        {
            flushPathSegments();
            extruder = new_extruder;
        }
    }

    /*!
     * Special handling of the first point in an added line sequence.
     * If the new sequence of lines does not start at the current end point
     * of the path this jump is marked as PrintFeatureType::NoneType
     */
    void handleInitialPoint(Point from)
    {
        if (points.size() == 0)
        {
            addPoint2D(from);
        }
        else if (from != last_point)
        {
            addLineSegment(PrintFeatureType::NoneType, from, 1.0, 0.0, 0.0);
        }
    }

    /*!
     * Transfers the currently buffered line segments to the
     * CommandSocket layer message storage.
     */
    void flushPathSegments();

    /*!
     * Move the current point of this path to \position.
     */
    void setCurrentPosition(Point position)
    {
        handleInitialPoint(position);
    }
    /*!
     * Adds a single line segment to the current path. The line segment added is from the current last point to point \p to
     */
    void sendLineTo(PrintFeatureType print_feature_type, Point to, int width, int thickness, int feedrate);
    /*!
     * Adds closed polygon to the current path
     */
    void sendPolygon(PrintFeatureType print_feature_type, ConstPolygonRef poly, int width, int thickness, int feedrate);

private:
    /*!
     * Convert and add a point to the points buffer, each point being represented as two consecutive floats. All members adding a 2D point to the data should use this function.
     */
    void addPoint2D(Point point)
    {
        points.push_back(INT2MM(point.X));
        points.push_back(INT2MM(point.Y));
        last_point = point;
    }

    /*!
     * Implements the functionality of adding a single 2D line segment to the path data. All member functions adding a 2D line segment should use this functions.
     */
    void addLineSegment(PrintFeatureType print_feature_type, Point point, int line_width, int line_thickness, int line_feedrate)
    {
        addPoint2D(point);
        line_types.push_back(print_feature_type);
        line_widths.push_back(INT2MM(line_width));
        line_thicknesses.push_back(INT2MM(line_thickness));
        line_feedrates.push_back(line_feedrate);
    }
};

ArcusCommunication::ArcusCommunication(const std::string& ip, const uint16_t port)
    : private_data(new Private)
    , path_compiler(new PathCompiler(*private_data))
{
    private_data->socket = new Arcus::Socket();
    private_data->socket->addListener(new Listener);

    private_data->socket->registerMessageType(&cura::proto::Slice::default_instance());
    private_data->socket->registerMessageType(&cura::proto::Layer::default_instance());
    private_data->socket->registerMessageType(&cura::proto::LayerOptimized::default_instance());
    private_data->socket->registerMessageType(&cura::proto::Progress::default_instance());
    private_data->socket->registerMessageType(&cura::proto::GCodeLayer::default_instance());
    private_data->socket->registerMessageType(&cura::proto::PrintTimeMaterialEstimates::default_instance());
    private_data->socket->registerMessageType(&cura::proto::SettingList::default_instance());
    private_data->socket->registerMessageType(&cura::proto::GCodePrefix::default_instance());
    private_data->socket->registerMessageType(&cura::proto::SlicingFinished::default_instance());
    private_data->socket->registerMessageType(&cura::proto::SettingExtruder::default_instance());

    log("Connecting to %s:%i\n", ip.c_str(), port);
    private_data->socket->connect(ip, port);
    while(private_data->socket->getState() != Arcus::SocketState::Connected && private_data->socket->getState() != Arcus::SocketState::Error)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); //Wait until we're connected. Check every 100ms.
    }
    log("Connected to %s:%i\n", ip.c_str(), port);
}

ArcusCommunication::~ArcusCommunication()
{
    log("Closing connection.\n");
    private_data->socket->close();
}

const bool ArcusCommunication::hasSlice() const
{
    return private_data->socket->getState() != Arcus::SocketState::Closed
        && private_data->socket->getState() != Arcus::SocketState::Error
        && private_data->slice_count < 1; //Only slice once per run of CuraEngine. See documentation of slice_count.
}

void ArcusCommunication::sliceNext()
{
    const Arcus::MessagePtr message = private_data->socket->takeNextMessage();

    //Handle the main Slice message.
    const cura::proto::Slice* slice = dynamic_cast<cura::proto::Slice*>(message.get()); //See if the message is of the message type Slice. Returns nullptr otherwise.
    if(slice)
    {
        logDebug("Received a Slice message.\n");
        const cura::proto::SettingList& global_settings = slice->global_settings();
        for (const cura::proto::Setting& setting : global_settings.settings())
        {
            FffProcessor::getInstance()->setSetting(setting.name(), setting.value());
        }
        private_data->object_count = 0; //Reset object counts.
        for (cura::proto::ObjectList object_list : slice->object_lists())
        {
            //handleObjectList(&object_list, slice->extruders());
            //TODO: Set up a scene here.
        }

        //For every object, set the extruder fall-backs from the limit_to_extruder.
        for (const cura::proto::SettingExtruder setting_extruder : slice->limit_to_extruder())
        {
            const int32_t extruder_nr = setting_extruder.extruder(); //Implicit cast from Protobuf's int32 to normal uint32.
            for (std::shared_ptr<MeshGroup> meshgroup : private_data->objects_to_slice)
            {
                if (extruder_nr < 0 || extruder_nr >= static_cast<int32_t>(meshgroup->getExtruderCount()))
                {
                    //If extruder_nr == -1 then the setting should be handled as if it has no limit_to_extruder, so we can skip it.
                    //If extruder_nr is less than -1 or more than the extruder count, we received an invalid extruder number from the front-end. Ignore that too.
                    continue;
                }
                const ExtruderTrain* settings_base = meshgroup->getExtruderTrain(extruder_nr); //The extruder train that the setting should fall back to.
                for (Mesh& mesh : meshgroup->meshes)
                {
                    mesh.setSettingInheritBase(setting_extruder.name(), *settings_base);
                }
            }
        }
        logDebug("Done reading Slice message.\n");
    }

    if (!private_data->objects_to_slice.empty())
    {
        const size_t object_count = private_data->objects_to_slice.size();
        logDebug("Slicing %i objects.\n", object_count);
        FffProcessor::getInstance()->resetMeshGroupNumber();
        for (size_t i = 0; i < object_count; i++)
        {
            logDebug("Slicing object %i of %i.\n", i + 1, object_count);
            if (!FffProcessor::getInstance()->processMeshGroup(private_data->objects_to_slice[i].get()))
            {
                logError("Slicing mesh group failed!\n");
            }
        }
        logDebug("Done slicing objects.\n");
        private_data->objects_to_slice.clear();
        FffProcessor::getInstance()->finalize();
        flushGCode();
        sendPrintTimeMaterialEstimates();
        sendFinishedSlicing();
        private_data->slice_count++;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250)); //Pause before checking again for a slice message.
}

} //namespace cura

#endif //ARCUS