//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <google/protobuf/message.h>

#include "MockSocket.h" //To mock out the communication with the front-end.
#include "ArcusCommunicationTest.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/FffProcessor.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationTest);

    void ArcusCommunicationTest::setUp()
    {
        ip = "0.0.0.0";
        port = 12345;
        socket = new MockSocket();
        ac = new ArcusCommunication();
        ac->setSocketMock(socket);

        // from: PolygonConnectorTest
        test_square.emplace_back(0, 0);
        test_square.emplace_back(1000, 0);
        test_square.emplace_back(1000, 1000);
        test_square.emplace_back(0, 1000);
        test_shapes.add(test_square);

        test_square2.emplace_back(1100, 1500);
        test_square2.emplace_back(2000, 1500);
        test_square2.emplace_back(2000, -500);
        test_square2.emplace_back(1100, -500);
        test_shapes.add(test_square2);

        test_triangle.emplace_back(0, 2100);
        test_triangle.emplace_back(500, 1100);
        test_triangle.emplace_back(1500, 2100);
        test_shapes.add(test_triangle);

        for (double a = 0; a < 1.0; a += .05)
        {
            test_circle.add(Point(2050, 2050) + Point(std::cos(a * 2 * M_PI)*500, std::sin(a * 2 * M_PI)*500));
        }
        test_shapes.add(test_circle);

        test_convex_shape.emplace_back(-300, 0);
        test_convex_shape.emplace_back(-100, 500);
        test_convex_shape.emplace_back(-100, 600);
        test_convex_shape.emplace_back(-200, 1000);
        test_convex_shape.emplace_back(-500, 1500);
        test_convex_shape.emplace_back(-1500, 1500);
        test_convex_shape.emplace_back(-1500, 1500);
        test_convex_shape.emplace_back(-1600, 1100);
        test_convex_shape.emplace_back(-700, 200);
        test_shapes.add(test_convex_shape);
    }

    void ArcusCommunicationTest::tearDown()
    {
        delete ac;
        ac = nullptr;
    }

    void ArcusCommunicationTest::beginGCodeTest()
    {
        ac->beginGCode();
    }

    void ArcusCommunicationTest::flushGCodeTest()
    {
        // std::cout << "flushGCodeTest...\n";
        // socket->setName("flushGCodeTest");
        // ac->flushGCode();
        // // If I don't do anything, no sendMessage calls should be made
        // std::cout << "Checking for sent messages when nothing has been done yet...\n";
        // CPPUNIT_ASSERT(socket->sent_messages.size() == 0);
    }

    void ArcusCommunicationTest::isSequentialTest()
    {
        CPPUNIT_ASSERT(ac->isSequential());
    }

    void ArcusCommunicationTest::hasSliceTest()
    {
        CPPUNIT_ASSERT(ac->hasSlice()); // TODO: Only possible when there's a slice send.
    }

    void ArcusCommunicationTest::sendCurrentPositionTest()
    {
//         socket->setName("sendCurrentPositionTest");
//         ac->sendCurrentPosition(Point(1, 2));
// //        ac->flushGCode();
// //        std::cout << "num messages" << socket->sent_messages.size() << "\n";
// //        CPPUNIT_ASSERT(false);
    }

    void ArcusCommunicationTest::sendGCodePrefixTest()
    {
        const std::string& prefix = "bladibla";

        //socket->setName("sendGCodePrefixTest");
        ac->sendGCodePrefix(prefix);
        ac->flushGCode();
        std::cout << "making sure that there are any messages sent...\n";
        CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
        //socket->printMessages();
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
        //socket->setName("sendFinishedSlicingTest");
        std::cout << "sendFinishedSlicingTest...\n";
        ac->sendFinishedSlicing();
        CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
        //socket->printMessages();
    }

    void ArcusCommunicationTest::sendLayerCompleteTest()
    {
        //socket->setName("sendLayerCompleteTest");
        std::cout << "sendLayerCompleteTest...\n";
        ac->sendLayerComplete(10, 20, 30);
        //socket->printMessages();
        //CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
    }

    void ArcusCommunicationTest::sendLineToTest()
    {
        const PrintFeatureType& type = PrintFeatureType::OuterWall;
        const Point& to = Point(100, 100);
        const coord_t& line_width = 400;
        const coord_t& line_thickness = 200;
        const Velocity& velocity = Velocity(10.0);

        //socket->setName("sendLineToTest");
        std::cout << "sendLineToTest...\n";
        ac->sendLineTo(type, to, line_width, line_thickness, velocity);
        ac->sendLayerComplete(10, 20, 30);
        //socket->printMessages();
        //CPPUNIT_ASSERT(false);
    }

    void ArcusCommunicationTest::sendOptimizedLayerDataTest()
    {
        //socket->setName("sendOptimizedLayerDataTest");
        std::cout << "sendOptimizedLayerDataTest...\n";
        ac->sendOptimizedLayerData();
        //socket->printMessages();
    }

    void ArcusCommunicationTest::sendPolygonTest()
    {
        const PrintFeatureType& type = PrintFeatureType::OuterWall;
        const ConstPolygonRef& polygon_ref = test_circle;
        const coord_t& line_width = 400;
        const coord_t& line_thickness = 200;
        const Velocity& velocity = Velocity(10.0);

        //socket->setName("sendPolygonTest");
        std::cout << "sendPolygonTest...\n";
        ac->sendPolygon(type, polygon_ref, line_width, line_thickness, velocity);
        //socket->printMessages();
    }

    void ArcusCommunicationTest::sendPolygonsTest()
    {
        const PrintFeatureType& type = PrintFeatureType::OuterWall;
        const Polygons& polygons = test_shapes;
        const coord_t& line_width = 400;
        const coord_t& line_thickness = 200;
        const Velocity& velocity = Velocity(10.0);

        //socket->setName("sendPolygonsTest");
        std::cout << "sendPolygonsTest...\n";
        ac->sendPolygons(type, polygons, line_width, line_thickness, velocity);
        //socket->printMessages();
    }

    void ArcusCommunicationTest::sendPrintTimeMaterialEstimatesTest()
    {
        // Segfault
//        socket->setName("sendPrintTimeMaterialEstimatesTest");
//        std::cout << "sendPrintTimeMaterialEstimatesTest...\n";
//        ac->sendPrintTimeMaterialEstimates();
//        socket->printMessages();
    }

    void ArcusCommunicationTest::sendProgressTest()
    {
        //socket->setName("sendProgressTest");
        std::cout << "sendProgressTest...\n";
        ac->sendProgress(10);
        //socket->printMessages();
        ac->sendProgress(50);
        //socket->printMessages();
    }

    void ArcusCommunicationTest::setLayerForSendTest()
    {
        //socket->setName("setLayerForSendTest");
        std::cout << "setLayerForSendTest...\n";
        ac->setLayerForSend(42);
        //socket->printMessages();
    }

    void ArcusCommunicationTest::setExtruderForSendTest()
    {

    }

    void ArcusCommunicationTest::sliceNextTest()
    {
        //socket->setName("sliceNextTest");
        std::cout << "sliceNextTest...\n";
        ac->sliceNext();
        //socket->printMessages();
    }
}

