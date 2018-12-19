//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "MockSocket.h"

namespace cura
{

MockSocket::MockSocket()
{
}

void MockSocket::connect(const std::string&, int) { /* Do nothing. */ }
void MockSocket::listen(const std::string&, int) { /* Do nothing. */ }
void MockSocket::close() { /* Do nothing. */ }
void MockSocket::reset() { /* Do nothing. */ }

void MockSocket::sendMessage(Arcus::MessagePtr message)
{
    sent_messages.push_back(message);
}

Arcus::MessagePtr MockSocket::takeNextMessage()
{
    Arcus::MessagePtr result = received_messages.front();
    received_messages.pop_front();
    return result;
}

void MockSocket::pushMessageToReceivedQueue(Arcus::MessagePtr message)
{
    received_messages.push_back(message);
}

Arcus::MessagePtr MockSocket::popMessageFromSendQueue()
{
    Arcus::MessagePtr result = sent_messages.front();
    sent_messages.pop_front();
    return result;
}

} //namespace cura