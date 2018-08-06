//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATION_H
#define ARCUSCOMMUNICATION_H
#ifdef ARCUS

#include <memory> //For unique_ptr and shared_ptr.

#include "Communication.h" //The class we're implementing.
#include "Cura.pb.h" //To create Protobuf messages for Cura's front-end.

namespace cura
{
//Forward declarations to speed up compilation.
class Slice;

/*
 * \brief Communication class that connects via libArcus to Cura's front-end.
 */
class ArcusCommunication : public Communication
{
public:
    /*
     * \brief Construct a new communicator that listens to libArcus messages via
     * a network socket.
     * \param ip The IP address to connect the socket to.
     * \param port The port number to connect the socket to.
     */
    ArcusCommunication(const std::string& ip, const uint16_t port);

    /*
     * \brief Closes the connection.
     */
    ~ArcusCommunication();

    /*
     * \brief Test if there are any more slices in the queue.
     */
    const bool hasSlice() const override;

    /*
     * \brief Send the current position to the front-end.
     *
     * This may indicate the starting position (or any other jump in the path).
     * \param position The current position to start the next line at.
     */
    void sendCurrentPosition(const Point& position) override;

    /*
     * \brief Indicate to the front-end that a layer is complete and send a
     * visualisation of the layer.
     *
     * This will be called after all the polygons and lines of this layer are
     * sent via sendPolygons, sendPolygon and sendLineTo. This will flush all
     * visualised data for one layer in one go.
     * \param layer_nr The layer that was completed.
     * \param z The z-coordinate of the top side of the layer.
     * \param thickness The thickness of the layer.
     */
    void sendLayerComplete(const LayerIndex& layer_nr, const coord_t& z, const coord_t& thickness) override;

    /*
     * \brief Send a line to the front-end to display in layer view.
     *
     * The line is not actually flushed until ``sendLayerComplete`` is called.
     * \param type The type of print feature the line represents (infill, wall,
     * support, etc).
     * \param to The destination coordinate of the line.
     * \param line_width The width of the line.
     * \param line_thickness The thickness (in the Z direction) of the line.
     * \param velocity The velocity of printing this polygon.
     */
    void sendLineTo(const PrintFeatureType& type, const Point& to, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

    /*
     * \brief Send the sliced layer data to the front-end after the optimisation
     * is done and the actual order in which to print has been set.
     *
     * This layer data will be shown in the layer view of the front end.
     */
    void sendOptimizedLayerData() override;

    /*
     * \brief Send a polygon to the front-end to display in layer view.
     *
     * The polygons are not actually flushed until ``sendLayerComplete`` is
     * called.
     * \param type The type of print feature the polygon represents (infill,
     * wall, support, etc).
     * \param polygon The shape to visualise.
     * \param line_width The width of the lines in this polygon.
     * \param line_thickness The thickness (in the Z direction) of the polygon.
     * \param velocity The velocity of printing this polygon.
     */
    void sendPolygon(const PrintFeatureType& type, const ConstPolygonRef& polygon, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

    /*
     * \brief Send polygons to the front-end to display in layer view.
     *
     * The polygons may not actually be flushed until ``sendLayerComplete`` is
     * called.
     * \param type The type of print feature the polygons represent (infill,
     * wall, support, etc).
     * \param polygons The shapes to visualise.
     * \param line_width The width of the lines in these polygons.
     * \param line_thickness The thickness (in the Z direction) of the polygons.
     * \param velocity The velocity of printing these polygons.
     */
    void sendPolygons(const PrintFeatureType& type, const Polygons& polygons, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

    /*
     * \brief Communicate to Arcus what our progress is.
     */
    void sendProgress(const float& progress) const override;

    /*
     * \brief Set which layer is being used for the following calls to
     * ``sendPolygon``, ``sendPolygons`` and ``sendLineTo``.
     * \param layer_nr The index of the layer to send data for. This is zero-
     * indexed but may be negative for raft layers.
     */
    virtual void setLayerForSend(const LayerIndex& layer_nr) override;

    /*
     * \brief Slice the next scene that the front-end wants us to slice.
     */
    void sliceNext() override;

private:
    /*
     * \brief PIMPL pattern subclass that contains the private implementation.
     */
    class Private;

    /*
     * \brief Pointer that contains the private implementation.
     */
    const std::unique_ptr<Private> private_data;

    /*
     * \brief Another PIMPL pattern subclass for private implementation regarding the
     * compilation of paths to Protobuf messages.
     */
    class PathCompiler;

    /*
     * \brief Pointer that contains the private implementation of the path compiler.
     */
    const std::unique_ptr<PathCompiler> path_compiler;
};

} //namespace cura

#endif //ARCUS
#endif //ARCUSCOMMUNICATION_H