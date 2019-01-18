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
    CPPUNIT_TEST(singleLineOnlyJerk);
    CPPUNIT_TEST(doubleLineOnlyJerk);
    CPPUNIT_TEST(singleLineNoJerk);
    CPPUNIT_TEST(doubleLineNoJerk);
    CPPUNIT_TEST(diagonalLineNoJerk);
    CPPUNIT_TEST(straightAngleOnlyJerk);
    CPPUNIT_TEST(straightAngleNoJerk);
    CPPUNIT_TEST(straightAnglePartialJerk);
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
     * Jerk is causing "instant" acceleration here, so only the maximum speed
     * has an influence.
     */
    void singleLineOnlyJerk();

    /*
     * \brief Time estimate of two lengthwise line segments without
     * acceleration.
     *
     * Jerk is causing an "instant" acceleration here, so only the maximum speed
     * has an influence.
     */
    void doubleLineOnlyJerk();

    /*
     * \brief Tests printing a single line with only acceleration.
     *
     * There is no jerk, so acceleration is the only factor in this line.
     */
    void singleLineNoJerk();

    /*
     * \brief Tests printing two lengthwise line segments without jerk.
     *
     * There is now a joint between these lines, but since the lines are exactly
     * lengthwise, it will not need to decelerate in the middle.
     */
    void doubleLineNoJerk();

    /*
     * \brief Tests printing a diagonal line without jerk.
     *
     * This line can accelerate in two axes at once, so the acceleration will
     * actually be greater than when the line is straight in one dimension.
     */
    void diagonalLineNoJerk();

    /*
     * \brief Tests printing two lines with a 90 degree angle where jerk governs
     * all acceleration.
     *
     * It should not slow down in the corner here.
     */
    void straightAngleOnlyJerk();

    /*
     * \brief Tests printing two lines with a 90 degree angle without jerk.
     *
     * This means that the nozzle will have to decelerate completely to 0 and
     * then accelerate in a different axis to maximum speed again.
     */
    void straightAngleNoJerk();

    /*
     * \brief Tests printing two lines with a 90 degree angle where part of the
     * speed is instantaneous by jerk, but it also has to decelerate to make the
     * corner at a limited jerk rate.
     */
    void straightAnglePartialJerk();

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

    /*
     * Firmware settings to print without any jerk. Only acceleration has any
     * effect.
     *
     * Acceleration is set to accelerate at 50 mm/s², so it will take one second
     * to reach the maximum velocity of 50 mm/s.
     */
    Settings jerkless;
};

} //namespace cura

#endif //TIMEESTIMATECALCULATORTEST_H