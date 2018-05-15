//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POLYGON_UTILS_TEST_H
#define POLYGON_UTILS_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/utils/PolygonConnector.h"

namespace cura
{

class PolygonConnectorTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(PolygonConnectorTest);
    CPPUNIT_TEST(getBridgeTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>LinearAlg2DTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>LinearAlg2DTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();
    
    // These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.

    void getBridgeTest();
private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;
    
    Polygon test_square;
    Polygon test_triangle;
    Polygons test_shapes; // all above polygons
    /*!
     * cppunit assert for PolygonUtils::getNextParallelIntersection
     */
    void getBridgeAssert(std::optional<PolygonConnector::PolygonBridge> predicted, ConstPolygonRef from_poly, std::vector<ConstPolygonPointer>& to_polygons);
};

}

#endif // POLYGON_UTILS_TEST_H

