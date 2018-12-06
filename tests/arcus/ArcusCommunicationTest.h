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
#include "../src/utils/polygon.h"

namespace cura
{

class ArcusCommunicationTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(ArcusCommunicationTest);
    CPPUNIT_TEST(flushGCodeTest);
    CPPUNIT_TEST(isSequentialTest);
    CPPUNIT_TEST(hasSliceTest);
    CPPUNIT_TEST(sendGCodePrefixTest);
    CPPUNIT_TEST(sendFinishedSlicingTest);
    CPPUNIT_TEST(sendLayerCompleteTest);
    CPPUNIT_TEST(sendProgressTest);
    CPPUNIT_TEST_SUITE_END();

public:
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
     * \brief Tests for every function
     */
    void flushGCodeTest();
    void isSequentialTest();
    void hasSliceTest();
    void readExtruderSettingsMessageTest();
    void sendGCodePrefixTest();
    void sendFinishedSlicingTest();
    void sendLayerCompleteTest();
    void sendProgressTest();

private:
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
};

}

#endif //CURAENGINE_ARCUSCOMMUNICATIONTEST_H
