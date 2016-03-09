//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BUCKETGRID2DTEST_H
#define BUCKETGRID2DTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/intpoint.h> //For Point.
#include <../src/utils/BucketGrid2D.h>

#include <unordered_set>

namespace cura
{

class BucketGrid2DTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(BucketGrid2DTest);
    CPPUNIT_TEST(findNearbyObjectsFarTest);
    CPPUNIT_TEST(findNearbyObjectsLine2Test);
    CPPUNIT_TEST(findNearbyObjectsLineTest);
    CPPUNIT_TEST(findNearbyObjectsNearTest);
    CPPUNIT_TEST(findNearbyObjectsSameTest);
    CPPUNIT_TEST(findNearestObjectChoiceTest);
    CPPUNIT_TEST(findNearestObjectEqualTest);
    CPPUNIT_TEST(findNearestObjectNoneTest);
    CPPUNIT_TEST(findNearestObjectSameTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     */
    void tearDown();

    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void findNearbyObjectsFarTest();
    void findNearbyObjectsLine2Test();
    void findNearbyObjectsLineTest();
    void findNearbyObjectsNearTest();
    void findNearbyObjectsSameTest();
    void findNearestObjectChoiceTest();
    void findNearestObjectEqualTest();
    void findNearestObjectNoneTest();
    void findNearestObjectSameTest();

private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;

    /*!
     * \brief Performs the actual assertion for the findNearbyObjects tests.
     *
     * This is essentially a parameterised version of all unit tests pertaining
     * to the findNearbyObjects tests.
     * It tests for some points whether they are near and for some whether they
     * are far. This also allows for points to be indeterminate, where either
     * answer is allowed.
     *
     * \param registered_points The points already in the grid, from which a
     * subset must be found that is near the target point.
     * \param target The target point, near which we must find points.
     * \param grid_size The grid size of the BucketGrid2D to use.
     * \param expected_near The expected set of points which is near.
     * \param expected_far The expected set of points which is far.
     */
    void findNearbyObjectsAssert(const std::vector<Point>& registered_points, Point target, unsigned long long grid_size, const std::unordered_set<Point>& expected_near, const std::unordered_set<Point>& expected_far);

    /*!
     * \brief Performs the actual assertion for the findNearestObject tests.
     *
     * This is essentially a parameterised version of all unit tests pertaining
     * to the findNearestObject tests.
     *
     * \param registered_points The points already in the grid, from which the
     * nearest point must be selected.
     * \param target The target point, to which the nearest point must be
     * selected.
     * \param grid_size The grid size of the BucketGrid2D to use.
     * \param expected The expected closest point, or <em>nullptr</em> if the
     * call is expected to return <em>false</em>.
     * \param precondition A boolean function on Points to filter by. Leave this
     * parameter out if you don't wish to filter.
     */
    void findNearestObjectAssert(const std::vector<Point>& registered_points, Point target, const unsigned long long grid_size, Point* expected, std::function<bool(Point location, Point& object)> precondition = BucketGrid2D<Point>::no_precondition);
};

}

#endif //LINEARALG2DTEST_H

