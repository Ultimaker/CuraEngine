//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TESTS_UTILS_SPARSE_GRID_TEST_H
#define TESTS_UTILS_SPARSE_GRID_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/utils/SparsePointGridInclusive.h"

#include <unordered_set>

namespace cura
{

class SparseGridTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(SparseGridTest);
    CPPUNIT_TEST(getNearbyFarTest);
    CPPUNIT_TEST(getNearbyLine2Test);
    CPPUNIT_TEST(getNearbyLineTest);
    CPPUNIT_TEST(getNearbyNearTest);
    CPPUNIT_TEST(getNearbySameTest);
    CPPUNIT_TEST(getNearestChoiceTest);
    CPPUNIT_TEST(getNearestEqualTest);
    CPPUNIT_TEST(getNearestFilterTest);
    CPPUNIT_TEST(getNearestNoneTest);
    CPPUNIT_TEST(getNearestSameTest);
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

    //These are the actual test cases. The name of the function sort
    //of describes what it tests but I refuse to document all of
    //these, sorry.
    void getNearbyFarTest();
    void getNearbyLine2Test();
    void getNearbyLineTest();
    void getNearbyNearTest();
    void getNearbySameTest();
    void getNearestChoiceTest();
    void getNearestEqualTest();
    void getNearestFilterTest();
    void getNearestNoneTest();
    void getNearestSameTest();

private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;

    /*!
     * \brief Performs the actual assertion for the getNearby tests.
     *
     * This is essentially a parameterised version of all unit tests pertaining
     * to the getNearby tests.
     * It tests for some points whether they are near and for some whether they
     * are far. This also allows for points to be indeterminate, where either
     * answer is allowed.
     *
     * \param registered_points The points already in the grid, from which a
     * subset must be found that is near the target point.
     * \param target The target point, near which we must find points.
     * \param grid_size The grid size of the SparsePointGridInclusive to use.
     * \param expected_near The expected set of points which is near.
     * \param expected_far The expected set of points which is far.
     */
    void getNearbyAssert(
        const std::vector<Point>& registered_points,
        Point target, coord_t grid_size,
        const std::unordered_set<Point>& expected_near,
        const std::unordered_set<Point>& expected_far);

    /*!
     * \brief Performs the actual assertion for the getNearest tests.
     *
     * This is essentially a parameterised version of all unit tests pertaining
     * to the getNearest tests.
     *
     * \param registered_points The points already in the grid, from which the
     * nearest point must be selected.
     * \param target The target point, to which the nearest point must be
     * selected.
     * \param grid_size The grid size of the SparsePointGridInclusive to use.
     * \param expected The expected closest point, or <em>nullptr</em> if the
     * call is expected to return <em>false</em>.
     * \param precondition A boolean function on Points to filter by. Leave this
     * parameter out if you don't wish to filter.
     */
    void getNearestAssert(
        const std::vector<Point>& registered_points,
        Point target, const coord_t grid_size,
        Point* expected,
        const std::function<bool(const typename SparsePointGridInclusive<Point>::Elem& elem)> &precondition =
            SparsePointGridInclusive<Point>::no_precondition);
};

}

#endif // TESTS_UTILS_SPARSE_GRID_TEST_H
