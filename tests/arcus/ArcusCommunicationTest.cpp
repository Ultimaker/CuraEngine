//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationTest.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/FffProcessor.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationTest);

    ArcusCommunicationTest::MockSocketListener::MockSocketListener()
    {

    }

    void ArcusCommunicationTest::MockSocketListener::sendMessage(Arcus::MessagePtr message)
    {
        std::cout << "message: " << message << "\n";
    }

    void ArcusCommunicationTest::setUp()
    {
        ip = "0.0.0.0";
        port = 12345;
        socket = new MockSocketListener();
    }

    void ArcusCommunicationTest::tearDown()
    {
        //Do nothing.
    }

    void ArcusCommunicationTest::smokeTest()
    {
        Communication* ac = new ArcusCommunication(ip, port);
    }

    void ArcusCommunicationTest::beginGCodeTest()
    {
        ArcusCommunication* ac = new ArcusCommunication(ip, port);
        //ac->private_data->socket = &socket;
        ac->beginGCode();

        // requires a lot of friendly touching privates
//        FffProcessor::getInstance()->gcode_writer.gcode.output_stream;

        //ac->private_data->socket->getState()
        //ac.flushGCode();

    }

    void ArcusCommunicationTest::flushGCodeTest()
    {

    }

    void ArcusCommunicationTest::isSequentialTest()
    {

    }

    void ArcusCommunicationTest::hasSliceTest()
    {

    }

    void ArcusCommunicationTest::sendCurrentPositionTest()
    {

    }

    void ArcusCommunicationTest::sendGCodePrefixTest()
    {

    }

    void ArcusCommunicationTest::sendFinishedSlicingTest()
    {

    }

    void ArcusCommunicationTest::sendLayerCompleteTest()
    {

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

