//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PolygonUtilsTest.h"


namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(PolygonUtilsTest);

void PolygonUtilsTest::setUp()
{
    test_square.emplace_back(0, 0);
    test_square.emplace_back(100, 0);
    test_square.emplace_back(100, 100);
    test_square.emplace_back(0, 100);

    test_squares.add(test_square);


    pointy_square.emplace_back(0, 0);
    pointy_square.emplace_back(47, 0);
    pointy_square.emplace_back(50, 80);
    pointy_square.emplace_back(53, 0);
    pointy_square.emplace_back(100, 0);
    pointy_square.emplace_back(100, 100);
    pointy_square.emplace_back(55, 100);
    pointy_square.emplace_back(50, 180);
    pointy_square.emplace_back(45, 100);
    pointy_square.emplace_back(0, 100);
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
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".\n";
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
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed1) <= maximum_error || vSize(result - supposed2) <= maximum_error || vSize(result - supposed3) <= maximum_error || vSize(result - supposed4) <= maximum_error);
    }
}

void PolygonUtilsTest::middleTestPenalty()
{
    const Point close_to(50, 50);
    const Point supposed(80, 50); 
    const Point preferred_dir(120, 60);
    const int distance = 20;
    const ClosestPolygonPoint cpp = PolygonUtils::findClosest(close_to, test_square, [preferred_dir](Point candidate) { return vSize2(candidate - preferred_dir); } );
    const Point result = PolygonUtils::moveInside(cpp, distance);
    {
        std::stringstream ss;
        ss << close_to << " moved with " << distance << " micron inside to " << result << " rather than " << supposed << ".\n";
        ss << "\tPS: dist to boundary computed = " << vSize(cpp.location - result) << "; idem supposed = " << vSize(cpp.location - result) << ".\n";
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
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
    PolygonUtils::moveInside2(polys, result, distance);
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
        ss << "\tclosest point = " << cpp.location << " at index " << cpp.point_idx << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) <= maximum_error);
    }
}

void PolygonUtilsTest::moveInside2Assert(const PolygonRef poly, Point close_to, const int distance, Point supposed)
{
    Polygons polys;
    polys.add(poly);
    Point result = close_to;
    PolygonUtils::moveInside2(polys, result, distance);
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

void PolygonUtilsTest::middleEdgeFindCloseTest()
{
    std::function<int(Point)> penalty_function([](Point candidate){ return -vSize2(candidate - Point(50,100)); }); // further from 50,100 is less penalty
    findCloseAssert(test_square, Point(50,50), Point(50,0), 60, &penalty_function);
}


void PolygonUtilsTest::findCloseAssert(const PolygonRef poly, Point close_to, Point supposed, int cell_size, const std::function<int(Point)>* penalty_function)
{
    Polygons polys;
    polys.add(poly);
    SparseLineGrid<PolygonsPointIndex, PolygonsPointIndexSegmentLocator>* loc_to_line = PolygonUtils::createLocToLineGrid(polys, cell_size);
    
    std::optional<ClosestPolygonPoint> cpp;
    if (penalty_function)
    {
        cpp = PolygonUtils::findClose(close_to, polys, *loc_to_line, *penalty_function);
    }
    else
    {
        cpp = PolygonUtils::findClose(close_to, polys, *loc_to_line);
    }
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

void PolygonUtilsTest::moveInsidePointyCornerTest()
{
    Point from(55, 170); // above pointy bit
    Point result(from);
    Polygons inside;
    inside.add(pointy_square);
    ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(inside, result, 10);
    if (cpp.point_idx == NO_INDEX || cpp.poly_idx == NO_INDEX)
    {
        std::stringstream ss;
        ss << "Couldn't ensure point inside close to " << from << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    else
    {
        std::stringstream ss;
        ss << from << " couldn't be moved inside.\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), inside.inside(result));
    }
}

void PolygonUtilsTest::moveInsidePointyCornerTestFail()
{ // should fail with normal moveInside2 (and the like)
    Point from(55, 170); // above pointy bit
    Point result(from);
    Polygons inside;
    inside.add(pointy_square);
    ClosestPolygonPoint cpp = PolygonUtils::moveInside2(inside, result, 10);
    if (cpp.point_idx == NO_INDEX || cpp.poly_idx == NO_INDEX)
    {
        std::stringstream ss;
        ss << "Couldn't ensure point inside close to " << from << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    else
    {
        std::stringstream ss;
        ss << from << " could be moved inside, while it was designed to fail.\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), !inside.inside(result));
    }
}

void PolygonUtilsTest::moveOutsidePointyCornerTest()
{
    Point from(60, 70); // above pointy bit
    Point result(from);
    Point supposed(50, 70); // 10 below pointy bit
    Polygons inside;
    inside.add(pointy_square);
//     ClosestPolygonPoint cpp = PolygonUtils::moveInside2(inside, result, -10);
    ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(inside, result, -10);
    if (cpp.point_idx == NO_INDEX || cpp.poly_idx == NO_INDEX)
    {
        std::stringstream ss;
        ss << "Couldn't ensure point inside close to " << from << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    else
    {
        std::stringstream ss;
        ss << from << " couldn't be moved inside.\n";
//         CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) < 5 + maximum_error && !inside.inside(result)); // +5 because ensureInside might do half the preferred distance moved inside
        CPPUNIT_ASSERT_MESSAGE(ss.str(), !inside.inside(result)); // +5 because ensureInside might do half the preferred distance moved inside
    }
}

void PolygonUtilsTest::moveOutsidePointyCornerTestFail()
{ // should fail with normal moveInside2 (and the like)
    Point from(60, 70); // above pointy bit
    Point result(from);
    Point supposed(50, 70); // 10 below pointy bit
    Polygons inside;
    inside.add(pointy_square);
    ClosestPolygonPoint cpp = PolygonUtils::moveInside2(inside, result, -10);
//     ClosestPolygonPoint cpp = PolygonUtils::ensureInsideOrOutside(inside, result, -10);
    if (cpp.point_idx == NO_INDEX || cpp.poly_idx == NO_INDEX)
    {
        std::stringstream ss;
        ss << "Couldn't ensure point inside close to " << from << ".\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    else
    {
        std::stringstream ss;
        ss << from << " could be moved inside to " << result << ", while it was designed to fail.\n";
//         CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(result - supposed) < 5 + maximum_error && !inside.inside(result)); // +5 because ensureInside might do half the preferred distance moved inside
        CPPUNIT_ASSERT_MESSAGE(ss.str(), inside.inside(result)); // +5 because ensureInside might do half the preferred distance moved inside
    }
}










void PolygonUtilsTest::spreadDotsTestSegment()
{
    std::vector<ClosestPolygonPoint> supposed;
    supposed.emplace_back(Point(50, 0), 0, test_squares[0], 0);
    supposed.emplace_back(Point(100, 0), 1, test_squares[0], 0);
    supposed.emplace_back(Point(100, 50), 1, test_squares[0], 0);

    spreadDotsAssert(PolygonsPointIndex(&test_squares, 0, 0), PolygonsPointIndex(&test_squares, 0, 2), 3, supposed);
}


void PolygonUtilsTest::spreadDotsTestFull()
{
    std::vector<ClosestPolygonPoint> supposed;
    supposed.emplace_back(Point(0, 0), 0, test_squares[0], 0);
    supposed.emplace_back(Point(50, 0), 0, test_squares[0], 0);
    supposed.emplace_back(Point(100, 0), 1, test_squares[0], 0);
    supposed.emplace_back(Point(100, 50), 1, test_squares[0], 0);
    supposed.emplace_back(Point(100, 100), 2, test_squares[0], 0);
    supposed.emplace_back(Point(50, 100), 2, test_squares[0], 0);
    supposed.emplace_back(Point(0, 100), 3, test_squares[0], 0);
    supposed.emplace_back(Point(0, 50), 3, test_squares[0], 0);

    spreadDotsAssert(PolygonsPointIndex(&test_squares, 0, 0), PolygonsPointIndex(&test_squares, 0, 0), 8, supposed);

}



void PolygonUtilsTest::spreadDotsAssert(PolygonsPointIndex start, PolygonsPointIndex end, unsigned int n_dots, const std::vector<ClosestPolygonPoint>& supposed)
{
    std::vector<ClosestPolygonPoint> result;
    PolygonUtils::spreadDots(start, end, n_dots, result);

    std::stringstream ss;
    ss << "PolygonUtils::spreadDots(" << start.point_idx << ", " << end.point_idx << ", " << n_dots << ") generated " << result.size() << " points, rather than " << supposed.size() << "!\n";
    CPPUNIT_ASSERT_MESSAGE(ss.str(), result.size() == supposed.size());

    for (unsigned int point_idx = 0 ; point_idx < result.size(); point_idx++)
    {
        std::stringstream ss;
        ss << point_idx << "nd point of PolygonUtils::spreadDots(" << start.point_idx << ", " << end.point_idx << ", " << n_dots << ") was " << result[point_idx].p() << ", rather than " << supposed[point_idx].p() << "!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), result[point_idx].p() == supposed[point_idx].p());
    }
}

void PolygonUtilsTest::getNextParallelIntersectionTest1()
{
    Point start_point(20, 100);
    Point line_to = Point(150, 200);
    bool forward = true;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(0, 40), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest2()
{
    Point start_point(80, 100);
    Point line_to = Point(150, 200);
    bool forward = true;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(37, 100), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest3()
{
    Point start_point(20, 100);
    Point line_to = Point(120, 200);
    bool forward = false;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(70, 100), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest4()
{
    Point start_point(50, 100);
    Point line_to = Point(150, 200);
    bool forward = true;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(0, 0), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest5()
{
    Point start_point(10, 0);
    Point line_to = Point(-90, -100);
    bool forward = true;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(60, 0), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest6()
{
    Point start_point(10, 0);
    Point line_to = Point(-90, -100);
    bool forward = false;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(0, 40), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest7()
{
    Point start_point(50, 100);
    Point line_to = Point(150, 100);
    bool forward = true;
    coord_t dist = 25;
    getNextParallelIntersectionAssert(Point(0, 75), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest8()
{
    Point start_point(50, 100);
    Point line_to = Point(50, 200);
    bool forward = true;
    coord_t dist = 25;
    getNextParallelIntersectionAssert(Point(25, 100), start_point, line_to, forward, dist);
}
void PolygonUtilsTest::getNextParallelIntersectionTest9()
{
    Point start_point(100, 100);
    Point line_to = Point(200, 200);
    bool forward = true;
    coord_t dist = 80;
    getNextParallelIntersectionAssert(std::optional<Point>(), start_point, line_to, forward, dist);
}

void PolygonUtilsTest::getNextParallelIntersectionTest10()
{
    Point start_point(5, 100);
    Point line_to = Point(105, 200);
    bool forward = true;
    coord_t dist = 35;
    getNextParallelIntersectionAssert(Point(0, 45), start_point, line_to, forward, dist);
}

void PolygonUtilsTest::getNextParallelIntersectionAssert(std::optional<Point> predicted, Point start_point, Point line_to, bool forward, coord_t dist)
{
    ClosestPolygonPoint start = PolygonUtils::findClosest(start_point, test_squares);
    std::optional<ClosestPolygonPoint> computed = PolygonUtils::getNextParallelIntersection(start, line_to, dist, forward);

    std::stringstream ss;
    ss << "PolygonUtils::getNextParallelIntersection(" << start_point << ", " << line_to << ", " << dist << ", " << forward << ") ";

    constexpr bool draw_problem_scenario = false; // make this true if you are debugging the function getNextParallelIntersection(.)

    auto draw = [this, predicted, start, line_to, dist, computed]()
        {
            if (!draw_problem_scenario)
            {
                return;
            }
            SVG svg("output/bs.svg", AABB(test_squares), Point(500,500));
            svg.writePolygons(test_squares);
            svg.writeLine(start.p(), line_to, SVG::Color::BLUE);
            svg.writePoint(start.p(), true);
            Point vec = line_to - start.p();
            Point shift = normal(turn90CCW(vec), dist);
            svg.writeLine(start.p() - vec + shift, line_to + vec + shift, SVG::Color::GREEN);
            svg.writeLine(start.p() - vec - shift, line_to + vec - shift, SVG::Color::GREEN);
            if (computed)
            {
                svg.writePoint(computed->p(), true, 5, SVG::Color::RED);
            }
            if (predicted)
            {
                svg.writePoint(*predicted, true, 5, SVG::Color::GREEN);
            }
        };

    if (!predicted && computed)
    {
        draw();
        ss << "gave a result (" << computed->p() << ") rather than the predicted no result!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    if (predicted && !computed)
    {
        draw();
        ss << "gave no result rather than the predicted " << *predicted << "!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), false);
    }
    if (predicted && computed)
    {
        draw();
        ss << "gave " << computed->p() << " while it was predicted to be " << *predicted << "!\n";
        CPPUNIT_ASSERT_MESSAGE(ss.str(), vSize(*predicted - computed->p()) < maximum_error);
    }
}

}
