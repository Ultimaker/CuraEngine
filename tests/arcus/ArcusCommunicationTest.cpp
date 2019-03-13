//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <google/protobuf/message.h>
#include <gtest/gtest.h>

#include "MockSocket.h" //To mock out the communication with the front-end.
#include "../src/FffProcessor.h"
#include "../src/communication/ArcusCommunicationPrivate.h" //To access the private fields of this communication class.
#include "../src/settings/types/LayerIndex.h"
#include "../src/utils/polygon.h" //Create test shapes to send over the socket.

namespace cura
{

/*
 * Test fixtures for an arcus communication class and some polygons to try
 * sending over the socket.
 */
class ArcusCommunicationTest : public testing::Test
{
public:
    std::string ip;
    uint16_t port;
    MockSocket* socket;
    ArcusCommunication* ac;

    // From PolygonConnectorTest
    Polygon test_square;
    Polygon test_square2; // larger and more to the right
    Polygon test_triangle;
    Polygon test_circle;
    Polygon test_convex_shape;

    Polygons test_shapes; // all above polygons

    void SetUp()
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

    void TearDown()
    {
        delete ac;
        ac = nullptr;
    }
};

TEST_F(ArcusCommunicationTest, FlushGCodeTest)
{
    //Before there is g-code, no messages should be sent if we were to flush.
    ac->flushGCode();
    ASSERT_TRUE(socket->sent_messages.empty());

    //Input some 'g-code' to flush.
    const std::string test_gcode = "This Fibonacci joke is as bad as the last two you heard combined.\n"
                                   "It's pretty cool how the Chinese made a language entirely out of tattoos."; //Multi-line to see flushing behaviour.
    ac->private_data->gcode_output_stream.write(test_gcode.c_str(), test_gcode.size());

    //Call the function we're testing. This time it should give us a message.
    ac->flushGCode();

    ASSERT_EQ(size_t(1), socket->sent_messages.size());
    const proto::GCodeLayer* message = dynamic_cast<proto::GCodeLayer*>(socket->sent_messages.back().get());
    EXPECT_EQ(test_gcode, message->data());
}

TEST_F(ArcusCommunicationTest, IsSequential)
{
    EXPECT_FALSE(ac->isSequential());
}

TEST_F(ArcusCommunicationTest, HasSlice)
{
    EXPECT_TRUE(ac->hasSlice());
    ac->private_data->slice_count = 1;
    EXPECT_FALSE(ac->hasSlice()) << "Can't slice more than once.";
}

TEST_F(ArcusCommunicationTest, SendGCodePrefix)
{
    const std::string& prefix = "bladibla";

    ac->sendGCodePrefix(prefix);
    ac->flushGCode();
    EXPECT_GT(socket->sent_messages.size(), 0);
    bool found_prefix = false;
    for (auto message : socket->sent_messages)
    {
        if (message->DebugString().find(prefix) != std::string::npos)
        {
            found_prefix = true;
            break;
        }
    }
    EXPECT_TRUE(found_prefix);
}

TEST_F(ArcusCommunicationTest, SendFinishedSlicingTest)
{
    ac->sendFinishedSlicing();
    EXPECT_EQ(size_t(1), socket->sent_messages.size());
}

TEST_F(ArcusCommunicationTest, SendLayerComplete)
{
    const LayerIndex layer_nr = 10;
    constexpr coord_t layer_z = 20;
    constexpr coord_t layer_thickness = 30;
    ac->sendLayerComplete(layer_nr, layer_z, layer_thickness);
    const std::shared_ptr<proto::LayerOptimized> message = ac->private_data->getOptimizedLayerById(layer_nr);
    EXPECT_EQ(static_cast<google::protobuf::int32>(layer_nr), message->id()) << "getOptimizedLayerById() must return a layer with the correct ID.";
    EXPECT_EQ(static_cast<float>(layer_z), message->height());
    EXPECT_EQ(static_cast<float>(layer_thickness), message->thickness());
}

TEST_F(ArcusCommunicationTest, SendProgress)
{
    ac->private_data->object_count = 2; //If there are two objects, all progress should get halved.

    ac->sendProgress(10);
    ASSERT_EQ(size_t(1), socket->sent_messages.size());
    proto::Progress* message = dynamic_cast<proto::Progress*>(socket->sent_messages.back().get());
    EXPECT_EQ(float(5), message->amount());

    ac->sendProgress(50);
    ASSERT_EQ(size_t(2), socket->sent_messages.size());
    message = dynamic_cast<proto::Progress*>(socket->sent_messages.back().get());
    EXPECT_EQ(float(25), message->amount());
}

}