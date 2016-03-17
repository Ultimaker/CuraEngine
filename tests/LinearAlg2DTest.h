//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LINEARALG2DTEST_H
#define LINEARALG2DTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/intpoint.h>

namespace cura
{

class LinearAlg2DTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(LinearAlg2DTest);
    CPPUNIT_TEST(getDist2FromLineSegmentHorizontalNearTest);
    CPPUNIT_TEST(getDist2FromLineSegmentHorizontalOnTest);
    CPPUNIT_TEST(getDist2FromLineSegmentHorizontalBeyondTest);
    CPPUNIT_TEST(getDist2FromLineSegmentHorizontalBeforeTest);
    CPPUNIT_TEST(getDist2FromLineSegmentHorizontalCornerTest);
    CPPUNIT_TEST(getDist2FromLineSegmentHorizontalPerpendicularTest);
    CPPUNIT_TEST(getDist2FromLineSegmentVerticalNearTest);
    CPPUNIT_TEST(getDist2FromLineSegmentVerticalOnTest);
    CPPUNIT_TEST(getDist2FromLineSegmentVerticalBeyondTest);
    CPPUNIT_TEST(getDist2FromLineSegmentVerticalBeforeTest);
    CPPUNIT_TEST(getDist2FromLineSegmentVerticalCornerTest);
    CPPUNIT_TEST(getDist2FromLineSegmentVerticalPerpendicularTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonalNearTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonalOnTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonalBeyondTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonalBeforeTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonalCornerTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonalPerpendicularTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2NearTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2OnTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2PointTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2BeyondTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2BeforeTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2CornerTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2PerpendicularTest);
    CPPUNIT_TEST(getDist2FromLineSegmentDiagonal2LargeTest);
    CPPUNIT_TEST(getDist2FromLineSegmentZeroNearTest);
    CPPUNIT_TEST(getDist2FromLineSegmentZeroOnTest);

    CPPUNIT_TEST(getAngleStraightTest);
    CPPUNIT_TEST(getAngle90CcwTest);
    CPPUNIT_TEST(getAngle90CwTest);
    CPPUNIT_TEST(getAngle45CcwTest);
    CPPUNIT_TEST(getAngleStraightBackTest);
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
    void getDist2FromLineSegmentHorizontalNearTest();
    void getDist2FromLineSegmentHorizontalOnTest();
    void getDist2FromLineSegmentHorizontalBeyondTest();
    void getDist2FromLineSegmentHorizontalBeforeTest();
    void getDist2FromLineSegmentHorizontalCornerTest();
    void getDist2FromLineSegmentHorizontalPerpendicularTest();
    void getDist2FromLineSegmentVerticalNearTest();
    void getDist2FromLineSegmentVerticalOnTest();
    void getDist2FromLineSegmentVerticalBeyondTest();
    void getDist2FromLineSegmentVerticalBeforeTest();
    void getDist2FromLineSegmentVerticalCornerTest();
    void getDist2FromLineSegmentVerticalPerpendicularTest();
    void getDist2FromLineSegmentDiagonalNearTest();
    void getDist2FromLineSegmentDiagonalOnTest();
    void getDist2FromLineSegmentDiagonalBeyondTest();
    void getDist2FromLineSegmentDiagonalBeforeTest();
    void getDist2FromLineSegmentDiagonalCornerTest();
    void getDist2FromLineSegmentDiagonalPerpendicularTest();
    void getDist2FromLineSegmentDiagonal2NearTest();
    void getDist2FromLineSegmentDiagonal2OnTest();
    void getDist2FromLineSegmentDiagonal2PointTest();
    void getDist2FromLineSegmentDiagonal2BeyondTest();
    void getDist2FromLineSegmentDiagonal2BeforeTest();
    void getDist2FromLineSegmentDiagonal2CornerTest();
    void getDist2FromLineSegmentDiagonal2PerpendicularTest();
    void getDist2FromLineSegmentDiagonal2LargeTest();
    void getDist2FromLineSegmentZeroNearTest();
    void getDist2FromLineSegmentZeroOnTest();
    
    void getAngleStraightTest();
    void getAngle90CcwTest();
    void getAngle90CwTest();
    void getAngle45CcwTest();
    void getAngleStraightBackTest();

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
    void getDist2FromLineSegmentAssert(Point line_start,Point line_end,Point point,int64_t actual_distance2,char actual_is_beyond);
    
    /*!
     * \brief The maximum allowed error in angle measurements.
     */
    static constexpr float maximum_error_angle = 1.0;
    
    /*!
     * Performs the assertion of the getAngle tests
     * 
     * \param a the a parameter of getAngle
     * \param b the b parameter of getAngle
     * \param c the c parameter of getAngle
     * \param actual_angle_in_half_rounds the actual angle where 0.5 equals ???
     */
    void getAngleAssert(Point a, Point b, Point c, float actual_angle_in_half_rounds);
};

}

#endif //LINEARALG2DTEST_H

