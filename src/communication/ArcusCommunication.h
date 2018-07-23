//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATION_H
#define ARCUSCOMMUNICATION_H
#ifdef ARCUS

#include <memory> //For unique_ptr and shared_ptr.
#include <queue> //To queue up slice commands.

#include "Communication.h" //The class we're implementing.

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

    /*
     * \brief A queue of slices that are still left to do.
     */
    std::queue<Slice> to_slice;
};

} //namespace cura

#endif //ARCUS
#endif //ARCUSCOMMUNICATION_H