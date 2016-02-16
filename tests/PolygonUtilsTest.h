//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POLYGON_UTILS_TEST_H
#define POLYGON_UTILS_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/intpoint.h>
#include <../src/utils/polygon.h>

namespace cura
{

class PolygonUtilsTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(PolygonUtilsTest);
    CPPUNIT_TEST(cornerInsideTest);
    CPPUNIT_TEST(edgeInsideTest);
    CPPUNIT_TEST(cornerOutsideTest);
    CPPUNIT_TEST(edgeOutsideTest);
    CPPUNIT_TEST(cornerInsideTest2);
    CPPUNIT_TEST(edgeInsideTest2);
    CPPUNIT_TEST(cornerOutsideTest2);
    CPPUNIT_TEST(edgeOutsideTest2);
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
    
    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void cornerInsideTest();
    void edgeInsideTest();
    void cornerOutsideTest();
    void edgeOutsideTest();
    void cornerInsideTest2();
    void edgeInsideTest2();
    void cornerOutsideTest2();
    void edgeOutsideTest2();

private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;
    
    Polygon test_square;
    
    /*!
     * cppunit assert for PolygonUtils::moveInside(ClosestPolygonPoint, int)
     */
    void moveInsideAssert(const PolygonRef poly, Point close_to, const int distance, Point supposed);
    
    /*!
     * cppunit assert for findSmallestConnection(ClosestPolygonPoint&, ClosestPolygonPoint&, int)
     */
    void moveInside2Assert(const PolygonRef poly, Point close_to, const int distance, Point supposed);
};

}

#endif //LINEARALG2DTEST_H

