// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MOCKSOCKET_H
#define MOCKSOCKET_H

#include <deque> //History of sent and received messages.

#include <Arcus/Socket.h> //Inheriting from this to be able to swap this socket in the tested class.

namespace cura
{

/*
 * \brief Mocks a socket connection from libArcus such that we can test with it
 * without creating an actual connection.
 */
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class MockSocket : public Arcus::Socket
{
public:
    MockSocket();

    // These functions are overridden to be no-op.
    void connect(const std::string& address, int port) override;
    void listen(const std::string& address, int port) override;
    void close() override;
    void reset() override;

    // Catch these functions so that we can see whether they are called.
    bool sendMessage(Arcus::MessagePtr message) override;
    Arcus::MessagePtr takeNextMessage() override;

    // Helpers to store send and received messages.
    void pushMessageToReceivedQueue(Arcus::MessagePtr message);
    Arcus::MessagePtr popMessageFromSendQueue();
    std::deque<Arcus::MessagePtr> sent_messages;
    std::deque<Arcus::MessagePtr> received_messages;
};
// NOLINTEND(misc-non-private-member-variables-in-classes)
} // namespace cura

#endif // MOCKSOCKET_H
