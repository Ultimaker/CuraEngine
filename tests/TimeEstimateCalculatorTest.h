//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TIMEESTIMATECALCULATORTEST_H
#define TIMEESTIMATECALCULATORTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/timeEstimate.h" //The class we're testing.
#include "../src/settings/Settings.h" //To set the firmware settings for acceleration/feedrate/etc.

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
     * \brief Tests whether moving to the current location doesn't incur any
     * time cost.
     */
    void moveToCurrentLocation();

    /*
     * \brief Time estimate of a single line without acceleration.
     *
     * Jerk is causing 'instant' acceleration here, so only the maximum speed
     * has an influence (apart from additional effects in the planner itself).
     */
    void singleLineOnlyJerk();

private:
    /*
     * Fixture calculator that starts without any time or moves planned.
     */
    TimeEstimateCalculator calculator;

    /*
     * Firmware settings that the Ultimaker 3 has in its Marlin version.
     *
     * This functions as a real-world case.
     */
    Settings um3;

    /*
     * Firmware settings that have infinite jerk and max speed on 50, so it
     * always runs at 50mm/s per axis.
     *
     * This is a simple case if you're only moving in the X direction, because
     * then the time should be easy to calculate by dividing the distance by 50.
     *
     * In places where acceleration happens without using jerk it will still use
     * the acceleration. Acceleration is also set to 50, so to accelerate from
     * 0 to full speed should take 1 second (which also makes it easy to
     * calculate).
     */
    Settings always_50;
};

} //namespace cura

#endif //TIMEESTIMATECALCULATORTEST_H