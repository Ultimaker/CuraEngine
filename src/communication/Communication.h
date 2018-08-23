//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "../utils/IntPoint.h" //For coord_t and Point.

namespace cura
{
//Some forward declarations to increase compilation speed.
struct LayerIndex;
struct Velocity;
enum class PrintFeatureType : unsigned char;
class Polygons;
class ConstPolygonRef;
class ExtruderTrain;

/*
 * An abstract class to provide a common interface for all methods of
 * communicating instructions from and to CuraEngine.
 */
class Communication
{
public:
    /*
     * \brief Close the communication channel.
     */
    virtual ~Communication() {}

    /*
     * \brief Test if there are more slices to be queued.
     */
    virtual bool hasSlice() const = 0;

    /*
     * \brief Whether the output needs to be sent from start to finish or not.
     *
     * This determines if the g-code output needs to be output from start to
     * finish in order.
     * This matters because the start g-code contains information on the
     * statistics of the print. These statistics can only be generated at the
     * end of the slice. Preferably we'd send the start g-code last, so that the
     * statistics in the start g-code can be more accurate.
     */
    virtual bool isSequential() const = 0;

    /*
     * \brief Indicate to the communication channel what the current progress of
     * slicing the current slice is.
     */
    virtual void sendProgress(const float& progress) const = 0;

    /*
     * \brief Indicate to the communication channel that a layer is complete and
     * send a visualisation of the layer.
     *
     * This will be called after all the polygons and lines of this layer are
     * sent via sendPolygons, sendPolygon and sendLineTo. Depending on the
     * protocol, this may flush all visualised data for one layer in one go.
     * \param layer_nr The layer that was completed.
     * \param z The z-coordinate of the top side of the layer.
     * \param thickness The thickness of the layer.
     */
    virtual void sendLayerComplete(const LayerIndex& layer_nr, const coord_t& z, const coord_t& thickness) = 0;

    /*
     * \brief Send polygons to the user to visualise.
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
    virtual void sendPolygons(const PrintFeatureType& type, const Polygons& polygons, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) = 0;

    /*
     * \brief Send a polygon to the user to visualise.
     *
     * The polygons may not actually be flushed until ``sendLayerComplete`` is
     * called.
     * \param type The type of print feature the polygon represents (infill,
     * wall, support, etc).
     * \param polygon The shape to visualise.
     * \param line_width The width of the lines in this polygon.
     * \param line_thickness The thickness (in the Z direction) of the polygon.
     * \param velocity The velocity of printing this polygon.
     */
    virtual void sendPolygon(const PrintFeatureType& type, const ConstPolygonRef& polygon, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) = 0;

    /*
     * \brief Send a line to the user to visualise.
     *
     * The line may not actually be flushed until ``sendLayerComplete`` is
     * called.
     * \param type The type of print feature the line represents (infill, wall,
     * support, etc).
     * \param to The destination coordinate of the line.
     * \param line_width The width of the line.
     * \param line_thickness The thickness (in the Z direction) of the line.
     * \param velocity The velocity of printing this polygon.
     */
    virtual void sendLineTo(const PrintFeatureType& type, const Point& to, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) = 0;

    /*
     * \brief Send the current position to visualise.
     *
     * This may indicate the starting position (or any other jump in the path).
     * \param position The current position to start the next line at.
     */
    virtual void sendCurrentPosition(const Point& position) = 0;

    /*
     * \brief Set which extruder is being used for the following calls to
     * ``sendPolygon``, ``sendPolygons`` and ``sendLineTo``.
     */
    virtual void setExtruderForSend(const ExtruderTrain& extruder) = 0;

    /*
     * \brief Set which layer is being used for the following calls to
     * ``sendPolygon``, ``sendPolygons`` and ``sendLineTo``.
     * \param layer_nr The index of the layer to send data for. This is zero-
     * indexed but may be negative for raft layers.
     */
    virtual void setLayerForSend(const LayerIndex& layer_nr) = 0;

    /*
     * \brief Send the sliced layer data through this communication after the
     * optimisation is done and the actual order in which to print has been set.
     *
     * The other side of the communication may use this to visualise the g-code,
     * so that the user can inspect the result of slicing.
     */
    virtual void sendOptimizedLayerData() = 0;

    /*
     * \brief Send an estimate of how long the print would take and how much
     * material it would use.
     */
    virtual void sendPrintTimeMaterialEstimates() const = 0;

    /*
     * \brief Indicate that we're beginning to send g-code.
     */
    virtual void beginGCode() = 0;

    /*
     * \brief Flush all remaining g-code to the user.
     */
    virtual void flushGCode() = 0;

    /*
     * \brief Send the starting g-code separately so that it may be processed by
     * the front-end for its replacement variables.
     */
    virtual void sendGCodePrefix(const std::string& prefix) const = 0;

    /*
     * \brief Sends a message to indicate that all the slicing is done.
     *
     * This should indicate that no more data (g-code, prefix/postfix, metadata
     * or otherwise) should be sent any more regarding the last slice job.
     */
    virtual void sendFinishedSlicing() const = 0;

    /*
     * \brief Get the next slice command from the communication and cause it to
     * slice.
     */
    virtual void sliceNext() = 0;
};

} //namespace cura

#endif //COMMUNICATION_H

