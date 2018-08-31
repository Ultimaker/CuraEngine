//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <google/protobuf/message.h>

#include "MockSocket.h" //To mock out the communication with the front-end.
#include "ArcusCommunicationTest.h"
#include "../src/FffProcessor.h"
#include "../src/communication/ArcusCommunicationPrivate.h" //To access the private fields of this communication class.
#include "../src/settings/types/LayerIndex.h"

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

    void ArcusCommunicationTest::flushGCodeTest()
    {
        //Before there is g-code, no messages should be sent if we were to flush.
        ac->flushGCode();
        CPPUNIT_ASSERT(socket->sent_messages.empty());

        //Input some 'g-code' to flush.
        const std::string test_gcode = "This Fibonacci joke is as bad as the last two you heard combined.\n"
                                       "It's pretty cool how the Chinese made a language entirely out of tattoos."; //Multi-line to see flushing behaviour.
        ac->private_data->gcode_output_stream.write(test_gcode.c_str(), test_gcode.size());

        //Call the function we're testing. This time it should give us a message.
        ac->flushGCode();

        CPPUNIT_ASSERT_EQUAL(size_t(1), socket->sent_messages.size());
        const proto::GCodeLayer* message = dynamic_cast<proto::GCodeLayer*>(socket->sent_messages.front().get());
        CPPUNIT_ASSERT_EQUAL(test_gcode, message->data());
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
//         ac->sendCurrentPosition(Point(1, 2));
// //        ac->flushGCode();
    }

    void ArcusCommunicationTest::sendGCodePrefixTest()
    {
        const std::string& prefix = "bladibla";

        ac->sendGCodePrefix(prefix);
        ac->flushGCode();
        CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
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
        ac->sendFinishedSlicing();
        CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
    }

    void ArcusCommunicationTest::sendLayerCompleteTest()
    {
        ac->sendLayerComplete(10, 20, 30);
        //CPPUNIT_ASSERT(socket->sent_messages.size() > 0);
    }

    void ArcusCommunicationTest::sendLineToTest()
    {
        const PrintFeatureType& type = PrintFeatureType::OuterWall;
        const Point& to = Point(100, 100);
        const coord_t& line_width = 400;
        const coord_t& line_thickness = 200;
        const Velocity& velocity = Velocity(10.0);

        ac->sendLineTo(type, to, line_width, line_thickness, velocity);
        ac->sendLayerComplete(10, 20, 30);
    }

    void ArcusCommunicationTest::sendOptimizedLayerDataTest()
    {
        ac->sendOptimizedLayerData();
    }

    void ArcusCommunicationTest::sendPolygonTest()
    {
        const PrintFeatureType& type = PrintFeatureType::OuterWall;
        const ConstPolygonRef& polygon_ref = test_circle;
        const coord_t& line_width = 400;
        const coord_t& line_thickness = 200;
        const Velocity& velocity = Velocity(10.0);

        ac->sendPolygon(type, polygon_ref, line_width, line_thickness, velocity);
    }

    void ArcusCommunicationTest::sendPolygonsTest()
    {
        const PrintFeatureType& type = PrintFeatureType::OuterWall;
        const Polygons& polygons = test_shapes;
        const coord_t& line_width = 400;
        const coord_t& line_thickness = 200;
        const Velocity& velocity = Velocity(10.0);

        ac->sendPolygons(type, polygons, line_width, line_thickness, velocity);
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
        ac->sendProgress(10);;
        ac->sendProgress(50);
    }

    void ArcusCommunicationTest::setLayerForSendTest()
    {
        ac->setLayerForSend(42);
    }

    void ArcusCommunicationTest::setExtruderForSendTest()
    {

    }

    void ArcusCommunicationTest::sliceNextTest()
    {
        ac->sliceNext();
    }
}

