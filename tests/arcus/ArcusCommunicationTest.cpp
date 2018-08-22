//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationTest.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/FffProcessor.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationTest);

    ArcusCommunicationTest::MockSocket::MockSocket()
    : sent_messages(std::vector<Arcus::MessagePtr>())
    , name("")
    {
    }

//    ArcusCommunicationTest::MockSocket::~MockSocket()
//    {
//
//    }

    Arcus::SocketState::SocketState ArcusCommunicationTest::MockSocket::getState() const
    {
        return Arcus::SocketState::Connected;
    }

//    Arcus::Error ArcusCommunicationTest::MockSocket::getLastError()
//    {
//        std::cout << "getLastError\n";
//        const Arcus::Error error = Arcus::Error();
//        return error;
//    }


    void ArcusCommunicationTest::MockSocket::clearError()
    {
        std::cout << "clearError\n";
    }

    bool ArcusCommunicationTest::MockSocket::registerMessageType(const google::protobuf::Message* message_type)
    {
        std::cout << "registerMessageType" << message_type << "\n";
        return true;
    }

    bool ArcusCommunicationTest::MockSocket::registerAllMessageTypes(const std::string& file_name)
    {
        std::cout << "registerAllMessageTypes" << file_name << "\n";
        return true;
    }

    void ArcusCommunicationTest::MockSocket::addListener(Arcus::SocketListener* listener)
    {
        std::cout << "addListener " << listener << "\n";
    }

    void ArcusCommunicationTest::MockSocket::removeListener(Arcus::SocketListener* listener)
    {
        std::cout << "removeListener" << listener << "\n";
    }

    void ArcusCommunicationTest::MockSocket::connect(const std::string& address, int port)
    {
        std::cout << "connect " << address << " - " << port << "\n";
    }

    void ArcusCommunicationTest::MockSocket::listen(const std::string& address, int port)
    {
        std::cout << "listen " << address << " - " << port << "\n";
    }

    void ArcusCommunicationTest::MockSocket::close()
    {
        std::cout << "close\n";
    }

    void ArcusCommunicationTest::MockSocket::reset()
    {
        std::cout << "reset\n";
    }

    void ArcusCommunicationTest::MockSocket::sendMessage(Arcus::MessagePtr message)
    {
        sent_messages.emplace_back(message);
        std::cout << name << ": sendMessage: " << message->ByteSize() << " -> '" << message->DebugString() << "'\n";
    }

    void ArcusCommunicationTest::MockSocket::setName(std::string new_name)
    {
        name = new_name;
    }

    void ArcusCommunicationTest::MockSocket::printMessages()
    {
        std::cout << name << ": printing messages:\n";
        for (auto message : sent_messages)
        {
            std::cout << name << ": message: " << message->ByteSize() << " -> '" << message->DebugString() << "'\n";
        }
    }

//    Arcus::MessagePtr ArcusCommunicationTest::MockSocket::takeNextMessage()
//    {}
//
//    Arcus::MessagePtr ArcusCommunicationTest::MockSocket::createMessage(const std::string& type_name)
//    {
//        std::cout << "createMessage: " << type_name << "\n";
//        return MessagePtr
//    }

    ///////////////////////

    void ArcusCommunicationTest::setUp()
    {
        ip = "0.0.0.0";
        port = 12345;
        socket = new MockSocket();
        ac = new ArcusCommunication();
        ac->private_data->socket = socket;
    }

    void ArcusCommunicationTest::tearDown()
    {
        //Do nothing.
    }

    void ArcusCommunicationTest::beginGCodeTest()
    {
        ac->beginGCode();
    }

    void ArcusCommunicationTest::flushGCodeTest()
    {
        std::cout << "flushGCodeTest...\n";
        socket->setName("flushGCodeTest");
        ac->flushGCode();
        // If I don't do anything, no sendMessage calls should be made
        std::cout << "Checking for sent messages when nothing has been done yet...\n";
        CPPUNIT_ASSERT(socket->sent_messages.size() == 0);
    }

    void ArcusCommunicationTest::isSequentialTest()
    {
        ac->isSequential();
    }

    void ArcusCommunicationTest::hasSliceTest()
    {
        ac->hasSlice();
    }

    void ArcusCommunicationTest::sendCurrentPositionTest()
    {
        socket->setName("sendCurrentPositionTest");
        ac->sendCurrentPosition(Point(1, 2));
//        ac->flushGCode();
//        std::cout << "num messages" << socket->sent_messages.size() << "\n";
//        CPPUNIT_ASSERT(false);
    }

    void ArcusCommunicationTest::sendGCodePrefixTest()
    {
        const std::string& prefix = "bladibla";

        socket->setName("sendGCodePrefixTest");
        ac->sendGCodePrefix(prefix);
        ac->flushGCode();
        std::cout << "making sure that there are any messages sent...\n";
        CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
        socket->printMessages();
        std::cout << "making sure that the original prefix occurs somewhere...\n";
        bool found_prefix = false;
        for (auto message : socket->sent_messages)
        {
            if (message->DebugString().find(prefix) != std::string::npos)
            {
                found_prefix = true;
                break;
            }
        }
        CPPUNIT_ASSERT(found_prefix);
    }

    void ArcusCommunicationTest::sendFinishedSlicingTest()
    {
        socket->setName("sendFinishedSlicingTest");
        std::cout << "sendFinishedSlicingTest...\n";
        ac->sendFinishedSlicing();
        CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
        socket->printMessages();
    }

    void ArcusCommunicationTest::sendLayerCompleteTest()
    {
        socket->setName("sendLayerCompleteTest");
        std::cout << "sendLayerCompleteTest...\n";
        ac->sendLayerComplete(10, 20, 30);
        socket->printMessages();
        //CPPUNIT_ASSERT(false);
    }

    void ArcusCommunicationTest::sendLineToTest()
    {

    }

    void ArcusCommunicationTest::sendOptimizedLayerDataTest()
    {

    }

    void ArcusCommunicationTest::sendPolygonTest()
    {

    }

    void ArcusCommunicationTest::sendPolygonsTest()
    {

    }

    void ArcusCommunicationTest::sendPrintTimeMaterialEstimatesTest()
    {

    }

    void ArcusCommunicationTest::sendProgressTest()
    {

    }

    void ArcusCommunicationTest::setLayerForSendTest()
    {

    }

    void ArcusCommunicationTest::setExtruderForSendTest()
    {

    }

    void ArcusCommunicationTest::sliceNextTest()
    {

    }

}

