//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MOCKSOCKET_H
#define MOCKSOCKET_H

#include <Arcus/Socket.h> //Inheriting from this to be able to swap this socket in the tested class.
#include <deque> //History of sent and received messages.

namespace cura
{

/*
 * \brief Mocks a socket connection from libArcus such that we can test with it
 * without creating an actual connection.
 */
class MockSocket : public Arcus::Socket
{
public:
    MockSocket();
    //virtual ~MockSocket();

    virtual void connect(const std::string& address, int port);
    virtual void listen(const std::string& address, int port);
    virtual void close();
    virtual void reset();

    virtual void sendMessage(Arcus::MessagePtr message);
    virtual Arcus::MessagePtr takeNextMessage();
    //virtual Arcus::MessagePtr createMessage(const std::string& type_name);

    void pushMessageToReceivedQueue(Arcus::MessagePtr message);
    Arcus::MessagePtr popMessageFromSendQueue();
    // void setName(const std::string& new_name);
    // void printMessages();

    std::deque<Arcus::MessagePtr> sent_messages;
    std::deque<Arcus::MessagePtr> received_messages;
};

} //namespace cura

#endif //MOCKSOCKET_H