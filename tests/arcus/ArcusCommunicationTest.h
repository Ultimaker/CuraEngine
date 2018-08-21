//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATION_TEST_H
#define ARCUSCOMMUNICATION_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <Arcus/Socket.h>
#include <Arcus/Types.h>

#include "../src/communication/ArcusCommunication.h"

namespace cura
{

class ArcusCommunicationTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(ArcusCommunicationTest);
    CPPUNIT_TEST(smokeTest);
    CPPUNIT_TEST(beginGCodeTest);
    CPPUNIT_TEST(flushGCodeTest);
    CPPUNIT_TEST(isSequentialTest);
    CPPUNIT_TEST(hasSliceTest);
    CPPUNIT_TEST(sendCurrentPositionTest);
    CPPUNIT_TEST(sendGCodePrefixTest);
    CPPUNIT_TEST(sendFinishedSlicingTest);
    CPPUNIT_TEST(sendLayerCompleteTest);
    CPPUNIT_TEST(sendLineToTest);
    CPPUNIT_TEST(sendOptimizedLayerDataTest);
    CPPUNIT_TEST(sendPolygonTest);
    CPPUNIT_TEST(sendPolygonsTest);
    CPPUNIT_TEST(sendPrintTimeMaterialEstimatesTest);
    CPPUNIT_TEST(sendProgressTest);
    CPPUNIT_TEST(setLayerForSendTest);
    CPPUNIT_TEST(setExtruderForSendTest);
    CPPUNIT_TEST(sliceNextTest);

    CPPUNIT_TEST_SUITE_END();

public:
    class MockSocket : public Arcus::Socket
    {
    public:
        MockSocket();
        //virtual ~MockSocket();
        virtual Arcus::SocketState::SocketState getState() const;
//        virtual Arcus::Error getLastError() const;
        virtual void clearError();
        virtual bool registerMessageType(const google::protobuf::Message* message_type);
        virtual bool registerAllMessageTypes(const std::string& file_name);
        virtual void addListener(Arcus::SocketListener* listener);
        virtual void removeListener(Arcus::SocketListener* listener);
        virtual void connect(const std::string& address, int port);
        virtual void listen(const std::string& address, int port);
        virtual void close();
        virtual void reset();
        virtual void sendMessage(Arcus::MessagePtr message);
//        virtual Arcus::MessagePtr takeNextMessage();
//        virtual Arcus::MessagePtr createMessage(const std::string& type_name);
    };


    /*!
     * \brief Sets up the test suite to prepare for testing.
     *
     * Since <em>SettingsTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     *
     * Since <em>SettingsTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    /*!
     * \brief Smoke test
     */
    void smokeTest();
    void beginGCodeTest();
    void flushGCodeTest();
    void isSequentialTest();
    void hasSliceTest();
    void sendCurrentPositionTest();
    void sendGCodePrefixTest();
    void sendFinishedSlicingTest();
    void sendLayerCompleteTest();
    void sendLineToTest();
    void sendOptimizedLayerDataTest();
    void sendPolygonTest();
    void sendPolygonsTest();
    void sendPrintTimeMaterialEstimatesTest();
    void sendProgressTest();
    void setLayerForSendTest();
    void setExtruderForSendTest();
    void sliceNextTest();

private:
    std::string ip;
    uint16_t port;
    MockSocket* socket;

};

}

#endif //CURAENGINE_ARCUSCOMMUNICATIONTEST_H
