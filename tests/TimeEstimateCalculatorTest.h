//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TIMEESTIMATECALCULATORTEST_H
#define TIMEESTIMATECALCULATORTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/timeEstimate.h" //The class we're testing.

namespace cura
{

/*
 * \brief Tests the routines of the TimeEstimateCalculator class.
 */
class TimeEstimateCalculatorTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(TimeEstimateCalculatorTest);
    CPPUNIT_TEST(addTime);
    CPPUNIT_TEST(startWithZero);
    CPPUNIT_TEST(singleMoveX);
    CPPUNIT_TEST_SUITE_END();

public:
    /*
     * \brief Resets the fixtures for a new test.
     */
    void setUp();

    /*
     * \brief Tests adding extra time to the calculation.
     */
    void addTime();

    /*
     * \brief Tests whether the calculator starts with an estimate of 0 seconds.
     */
    void startWithZero();

    /*
     * \brief Tests planning one single move along the X axis.
     *
     * At the start, the position and velocity are 0. At the end, they must also
     * be 0. We test how long it would take to execute this move.
     */
    void singleMoveX();

private:
    /*
     * Fixture calculator that starts without any time or moves planned.
     */
    TimeEstimateCalculator calculator;
};

} //namespace cura

#endif //TIMEESTIMATECALCULATORTEST_H