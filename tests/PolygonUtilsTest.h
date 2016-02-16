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

private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;
    
    /*!
     * \brief Performs the actual assertion for the getDist2FromLineSegmentTest.
     * 
     * This is essentially a parameterised version of all unit tests pertaining
     * to the getDist2FromLineSegment tests.
     * 
     * \param line_start The start of the line to check the distance to.
     * \param line_end The end of the line to check the distance to.
     * \param point The point to check the distance to the line with.
     * \param actual_distance2 The correct distance from the point to the line,
     * squared.
     * \param actual_is_beyond Whether the point is actually beyond the line.
     */
    void moveInsideAssert(const PolygonRef poly, Point close_to, const int distance, Point supposed);
};

}

#endif //LINEARALG2DTEST_H

