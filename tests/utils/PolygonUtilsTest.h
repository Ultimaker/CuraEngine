//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POLYGON_UTILS_TEST_H
#define POLYGON_UTILS_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/utils/IntPoint.h"
#include "../src/utils/polygon.h"
#include "../src/utils/polygonUtils.h"

namespace cura
{

class PolygonUtilsTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(PolygonUtilsTest);
    CPPUNIT_TEST(cornerInsideTest);
    CPPUNIT_TEST(edgeInsideTest);
    CPPUNIT_TEST(cornerOutsideTest);
    CPPUNIT_TEST(edgeOutsideTest);
    CPPUNIT_TEST(cornerCrookedTest);
    CPPUNIT_TEST(cornerEdgeTest);
    CPPUNIT_TEST(onBorderTest);
    CPPUNIT_TEST(insideTest);
    CPPUNIT_TEST(middleTest);
    CPPUNIT_TEST(middleTestPenalty);
    CPPUNIT_TEST(noMoveTest);
    CPPUNIT_TEST(farMoveTest);
    CPPUNIT_TEST(cornerInsideTest2);
    CPPUNIT_TEST(edgeInsideTest2);
    CPPUNIT_TEST(cornerOutsideTest2);
    CPPUNIT_TEST(edgeOutsideTest2);
    CPPUNIT_TEST(cornerFindCloseTest);
    CPPUNIT_TEST(edgeFindCloseTest);
    CPPUNIT_TEST(middleEdgeFindCloseTest);
    CPPUNIT_TEST(moveInsidePointyCornerTest);
    CPPUNIT_TEST(moveOutsidePointyCornerTest);
    CPPUNIT_TEST(moveInsidePointyCornerTestFail);
    CPPUNIT_TEST(moveOutsidePointyCornerTestFail);
    CPPUNIT_TEST(cornerCrookedTest2);
    CPPUNIT_TEST(cornerEdgeTest2);
    CPPUNIT_TEST(onBorderTest2);
    CPPUNIT_TEST(insideTest2);
    CPPUNIT_TEST(middleTest2);
    CPPUNIT_TEST(noMoveTest2);
    CPPUNIT_TEST(farMoveTest2);
    CPPUNIT_TEST(spreadDotsTestSegment);
    CPPUNIT_TEST(spreadDotsTestFull);
    CPPUNIT_TEST(getNextParallelIntersectionTest1);
    CPPUNIT_TEST(getNextParallelIntersectionTest2);
    CPPUNIT_TEST(getNextParallelIntersectionTest3);
    CPPUNIT_TEST(getNextParallelIntersectionTest4);
    CPPUNIT_TEST(getNextParallelIntersectionTest5);
    CPPUNIT_TEST(getNextParallelIntersectionTest6);
    CPPUNIT_TEST(getNextParallelIntersectionTest7);
    CPPUNIT_TEST(getNextParallelIntersectionTest8);
    CPPUNIT_TEST(getNextParallelIntersectionTest9);
    CPPUNIT_TEST(getNextParallelIntersectionTest10);
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
    void cornerCrookedTest();
    void cornerEdgeTest();
    void onBorderTest();
    void insideTest();
    void middleTest();
    void middleTestPenalty();
    void noMoveTest();
    void farMoveTest();
    void cornerInsideTest2();
    void edgeInsideTest2();
    void cornerOutsideTest2();
    void edgeOutsideTest2();
    void cornerCrookedTest2();
    void cornerEdgeTest2();
    void onBorderTest2();
    void insideTest2();
    void middleTest2();
    void noMoveTest2();
    void farMoveTest2();

    void polygonOffsetTest();

    void cornerFindCloseTest();
    void edgeFindCloseTest();
    void middleEdgeFindCloseTest();

    void moveInsidePointyCornerTest();
    void moveOutsidePointyCornerTest();
    void moveInsidePointyCornerTestFail();
    void moveOutsidePointyCornerTestFail();

    void spreadDotsTestSegment();
    void spreadDotsTestFull();

    void getNextParallelIntersectionTest1();
    void getNextParallelIntersectionTest2();
    void getNextParallelIntersectionTest3();
    void getNextParallelIntersectionTest4();
    void getNextParallelIntersectionTest5();
    void getNextParallelIntersectionTest6();
    void getNextParallelIntersectionTest7();
    void getNextParallelIntersectionTest8();
    void getNextParallelIntersectionTest9();
    void getNextParallelIntersectionTest10();
private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;
    
    Polygon test_square;
    Polygons test_squares; // same in a vector with one element
    Polygon pointy_square;
    
    /*!
     * cppunit assert for PolygonUtils::moveInside(ClosestPolygonPoint, int)
     */
    void moveInsideAssert(const PolygonRef poly, Point close_to, const int distance, Point supposed);
    
    /*!
     * cppunit assert for findSmallestConnection(ClosestPolygonPoint&, ClosestPolygonPoint&, int)
     */
    void moveInside2Assert(const PolygonRef poly, Point close_to, const int distance, Point supposed);
    
    /*!
     * cppunit assert for PolygonUtils::findClose
     */
    void findCloseAssert(const PolygonRef poly, Point close_to, Point supposed, int cell_size, const std::function<int(Point)>* penalty_function = nullptr);

    /*!
     * cppunit assert for PolygonUtils::spreadDots
     */
    void spreadDotsAssert(PolygonsPointIndex start, PolygonsPointIndex end, unsigned int n_dots, const std::vector<ClosestPolygonPoint>& supposed);

    /*!
     * cppunit assert for PolygonUtils::getNextParallelIntersection
     */
    void getNextParallelIntersectionAssert(std::optional<Point> predicted, Point start_point, Point line_to, bool forward, coord_t dist);
};

}

#endif // POLYGON_UTILS_TEST_H

