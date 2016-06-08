//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonUtilsTest.h"

#include <../src/utils/polygonUtils.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonUtilsTest);

void PolygonUtilsTest::setUp()
{
    test_square.emplace_back(0, 0);
    test_square.emplace_back(100, 0);
    test_square.emplace_back(100, 100);
    test_square.emplace_back(0, 100);
}

void PolygonUtilsTest::tearDown()
{
    //Do nothing.
}

void PolygonUtilsTest::cornerInsideTest()
{
    moveInsideAssert(test_square, Point(110, 110), 28, Point(80, 80));
}

void PolygonUtilsTest::edgeInsideTest()
{
    moveInsideAssert(test_square, Point(50, 110), 20, Point(50, 80));
}

void PolygonUtilsTest::cornerOutsideTest()
{
    moveInsideAssert(test_square, Point(110, 110), -28, Point(120, 120));
}

void PolygonUtilsTest::edgeOutsideTest()
{
    moveInsideAssert(test_square, Point(50, 110), -20, Point(50, 120));
}

void PolygonUtilsTest::cornerCrookedTest()
{
    moveInsideAssert(test_square, Point(110, 105), 28, Point(80, 80));
}

void PolygonUtilsTest::cornerEdgeTest()
{
    const Point close_to(110, 100);
    const Point supposed1(80, 80); //Allow two possible values here, since the behaviour for this edge case is not specified.
    const Point supposed2(72, 100);
    const int distance = 28;
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, test_square);
    const Point result = PolygonUtils::moveInside(cpp, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << " or " << supposed2 << ".\n";
        ss << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; idem supposed = " << vSize(cpp.location - result) << ".\n";
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.pos << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error);
    }
}

void PolygonUtilsTest::onBorderTest()
{
    moveInsideAssert(test_square, Point(100, 50), 20, Point(80, 50));
}

void PolygonUtilsTest::insideTest()
{
    moveInsideAssert(test_square, Point(80, 50), 20, Point(80, 50));
}

void PolygonUtilsTest::middleTest()
{
    const Point close_to(50, 50);
    const Point supposed1(80, 50); //Allow four possible values here, since the behaviour for this edge case is not specified.
    const Point supposed2(50, 80);
    const Point supposed3(20, 50);
    const Point supposed4(50, 20);
    const int distance = 20;
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, test_square);
    const Point result = PolygonUtils::moveInside(cpp, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << ", " << supposed2 << ", " << supposed3 << " or " << supposed4 << ".\n";
        ss << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; idem supposed = " << vSize(cpp.location - result) << ".\n";
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.pos << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error || vSize(result - supposed3) <= maximum_error || vSize(result - supposed4) <= maximum_error);
    }
}

void PolygonUtilsTest::noMoveTest()
{
    moveInsideAssert(test_square, Point(110, 50), 0, Point(100, 50));
}

void PolygonUtilsTest::farMoveTest()
{
    moveInsideAssert(test_square, Point(110, 50), 100000, Point(-99900, 50));
}

void PolygonUtilsTest::cornerInsideTest2()
{
    moveInside2Assert(test_square, Point(110, 110), 28, Point(80, 80));
}

void PolygonUtilsTest::edgeInsideTest2()
{
    moveInside2Assert(test_square, Point(50, 110), 20, Point(50, 80));
}

void PolygonUtilsTest::cornerOutsideTest2()
{
    moveInside2Assert(test_square, Point(110, 110), -28, Point(120, 120));
}

void PolygonUtilsTest::edgeOutsideTest2()
{
    moveInside2Assert(test_square, Point(50, 110), -20, Point(50, 120));
}

void PolygonUtilsTest::cornerCrookedTest2()
{
    moveInside2Assert(test_square, Point(110, 105), 28, Point(80, 80));
}

void PolygonUtilsTest::cornerEdgeTest2()
{
    const Point close_to(110, 100);
    const Point supposed1(80, 80); //Allow two possible values here, since the behaviour for this edge case is not specified.
    const Point supposed2(72, 100);
    const int distance = 28;
    Polygons polys;
    polys.add(test_square);
    Point result = close_to;
    PolygonUtils::moveInside(polys, result, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed1 << " or " << supposed2 << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error);
    }
}

void PolygonUtilsTest::onBorderTest2()
{
    moveInside2Assert(test_square, Point(100, 50), 20, Point(80, 50));
}

void PolygonUtilsTest::insideTest2()
{
    moveInside2Assert(test_square, Point(80, 50), 20, Point(80, 50));
}

void PolygonUtilsTest::middleTest2()
{
    moveInside2Assert(test_square, Point(50, 50), 20, Point(50, 50)); // this moveInside function leaves points which are already inside as they are
}

void PolygonUtilsTest::noMoveTest2()
{
    moveInside2Assert(test_square, Point(110, 50), 0, Point(100, 50));
}

void PolygonUtilsTest::farMoveTest2()
{
    moveInside2Assert(test_square, Point(110, 50), 100000, Point(-99900, 50));
}

void PolygonUtilsTest::moveInsideAssert(const PolygonRef poly, Point close_to, const int distance, Point supposed)
{
    ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, poly);
    Point result = PolygonUtils::moveInside(cpp, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n";
        ss << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; idem supposed = " << vSize(cpp.location - result) << ".\n";
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.pos << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
}

void PolygonUtilsTest::moveInside2Assert(const PolygonRef poly, Point close_to, const int distance, Point supposed)
{
    Polygons polys;
    polys.add(poly);
    Point result = close_to;
    PolygonUtils::moveInside(polys, result, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
}

void PolygonUtilsTest::cornerFindCloseTest()
{
    findCloseAssert(test_square, Point(110,110), Point(100,100), 15);
}

void PolygonUtilsTest::edgeFindCloseTest()
{
    findCloseAssert(test_square, Point(50,110), Point(50,100), 15);
}



void PolygonUtilsTest::findCloseAssert(const PolygonRef poly, Point close_to, Point supposed, int cell_size)
{
    Polygons polys;
    polys.add(poly);
    BucketGrid2D<PolygonsPointIndex>* loc_to_line = PolygonUtils::createLocToLineGrid(polys, cell_size);
    
    ClosestPolygonPoint* cpp = PolygonUtils::findClose(close_to, polys, *loc_to_line);
    if (cpp)
    {
        std::stringstream ss;
        Point result = cpp->location;
        ss << "Close to " << close_to << " we found " << result << " rather than " << supposed << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
    else 
    {
        std::stringstream ss;
        ss << "Couldn't find anything close to " << close_to << " ( should have been " << supposed << ").\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    
    delete loc_to_line;
}

}