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
     * Closes the connection.
     */
    ~ArcusCommunication();

    /*
     * \brief Test if there are any more slices in the queue.
     */
    const bool hasSlice() const override;

    /*
     * Slice the next scene that the front-end wants us to slice.
     */
    void sliceNext() override;

private:
    /*
     * PIMPL pattern subclass that contains the private implementation.
     */
    class Private;

    /*
     * Pointer that contains the private implementation.
     */
    const std::unique_ptr<Private> private_data;

    /*
     * Another PIMPL pattern subclass for private implementation regarding the
     * compilation of paths to Protobuf messages.
     */
    class PathCompiler;

    /*
     * Pointer that contains the private implementation of the path compiler.
     */
    const std::unique_ptr<PathCompiler> path_compiler;
};

} //namespace cura

#endif //ARCUS
#endif //ARCUSCOMMUNICATION_H