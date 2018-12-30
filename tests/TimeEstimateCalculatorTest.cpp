//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <numeric>

#include "TimeEstimateCalculatorTest.h"
#include "../src/PrintFeature.h" //We get time estimates per print feature.
#include "../src/settings/types/Duration.h"

namespace cura
{

#define MINIMUM_PLANNER_SPEED 0.05 // mm/sec
#define EPSILON 0.00001 //Allowed error in comparing floating point values.

CPPUNIT_TEST_SUITE_REGISTRATION(TimeEstimateCalculatorTest);

void TimeEstimateCalculatorTest::setUp()
{
    //Reset the calculator, but not by using its reset() function. That would be broken if the reset() function is broken.
    calculator = TimeEstimateCalculator();

    um3.add("machine_max_feedrate_x", "300");
    um3.add("machine_max_feedrate_y", "300");
    um3.add("machine_max_feedrate_z", "40");
    um3.add("machine_max_feedrate_e", "45");
    um3.add("machine_max_acceleration_x", "9000");
    um3.add("machine_max_acceleration_y", "9000");
    um3.add("machine_max_acceleration_z", "100");
    um3.add("machine_max_acceleration_e", "10000");
    um3.add("machine_max_jerk_xy", "20");
    um3.add("machine_max_jerk_z", "0.4");
    um3.add("machine_max_jerk_e", "5");
    um3.add("machine_minimum_feedrate", "0");
    um3.add("machine_acceleration", "3000");

    always_50.add("machine_max_feedrate_x", "50");
    always_50.add("machine_max_feedrate_y", "50");
    always_50.add("machine_max_feedrate_z", "50");
    always_50.add("machine_max_feedrate_e", "50");
    always_50.add("machine_max_acceleration_x", "50");
    always_50.add("machine_max_acceleration_y", "50");
    always_50.add("machine_max_acceleration_z", "50");
    always_50.add("machine_max_acceleration_e", "50");
    always_50.add("machine_max_jerk_xy", "1000");
    always_50.add("machine_max_jerk_z", "1000");
    always_50.add("machine_max_jerk_e", "1000");
    always_50.add("machine_minimum_feedrate", "0");
    always_50.add("machine_acceleration", "50");

    jerkless.add("machine_max_feedrate_x", "50");
    jerkless.add("machine_max_feedrate_y", "50");
    jerkless.add("machine_max_feedrate_z", "50");
    jerkless.add("machine_max_feedrate_e", "50");
    jerkless.add("machine_max_acceleration_x", "50");
    jerkless.add("machine_max_acceleration_y", "50");
    jerkless.add("machine_max_acceleration_z", "50");
    jerkless.add("machine_max_acceleration_e", "50");
    jerkless.add("machine_max_jerk_xy", "0");
    jerkless.add("machine_max_jerk_z", "0");
    jerkless.add("machine_max_jerk_e", "0");
    jerkless.add("machine_minimum_feedrate", "0");
    jerkless.add("machine_acceleration", "50");

    calculator.setFirmwareDefaults(um3);
    calculator.setPosition(TimeEstimateCalculator::Position(0, 0, 0, 0));
}

void TimeEstimateCalculatorTest::addTime()
{
    calculator.addTime(2);
    std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(2.0), result[static_cast<size_t>(PrintFeatureType::NoneType)], EPSILON);

    calculator.addTime(3); //Has to add up, not replace.
    result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(5.0), result[static_cast<size_t>(PrintFeatureType::NoneType)], EPSILON);

    calculator.addTime(-7);
    result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(5.0), result[static_cast<size_t>(PrintFeatureType::NoneType)], EPSILON); //Due to how Duration works, it can never go lower.
}

void TimeEstimateCalculatorTest::startWithZero()
{
    const std::vector<Duration> result = calculator.calculate();

    CPPUNIT_ASSERT_DOUBLES_EQUAL(static_cast<size_t>(PrintFeatureType::NumPrintFeatureTypes), result.size(), EPSILON);

    for (const Duration estimate : result)
    {
        CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("Time estimates must be zero before anything has been planned.", Duration(0.0), estimate, EPSILON);
    }
}

void TimeEstimateCalculatorTest::moveToCurrentLocation()
{
    const TimeEstimateCalculator::Position position(1000, 2000, 3000, 4000);
    calculator.setPosition(position);

    std::vector<Duration> result = calculator.calculate();
    Duration estimate = std::accumulate(result.begin(), result.end(), Duration(0.0));
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("setPosition should not add any time to the estimate.", Duration(0.0), estimate, EPSILON);

    calculator.plan(position, Velocity(10), PrintFeatureType::Infill);

    result = calculator.calculate();
    estimate = std::accumulate(result.begin(), result.end(), Duration(0.0));
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("Moving to the same location as where you already were should not cost any time.", Duration(0.0), estimate, EPSILON);
}

void TimeEstimateCalculatorTest::singleLineOnlyJerk()
{
    calculator.setFirmwareDefaults(always_50);

    const TimeEstimateCalculator::Position destination(1000, 0, 0, 0);

    /*
     * This line:
     * Accelerate instantly from 0 to 50 mm/s because of jerk.
     * Travel at 50 mm/s throughout the line.
     * Decelerate to minimum planner speed because at the end of the planner, jerk is not used.
     */
    calculator.plan(destination, 50.0, PrintFeatureType::Infill);

    //Deceleration distance is 1/2 at² + vt. We decelerate at 50mm/s for almost a second, ending at a speed of MINIMUM_PLANNER_SPEED.
    const double t = (50.0 - MINIMUM_PLANNER_SPEED) / 50.0;
    const double decelerate_distance = 0.5 * 50.0 * t * t + MINIMUM_PLANNER_SPEED * t;
    const double cruise_distance = 1000.0 - decelerate_distance;

    const std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(
        Duration(cruise_distance / 50.0 + t), //1000mm at 50mm/s, then decelerate for almost one second.
        result[static_cast<size_t>(PrintFeatureType::Infill)],
        EPSILON
    );
}

void TimeEstimateCalculatorTest::doubleLineOnlyJerk()
{
    calculator.setFirmwareDefaults(always_50);

    /*
     * These lines:
     * Accelerate instantly from 0 to 50mm/s because of jerk.
     * Travel at 50 mm/s throughout the first line.
     * At the end of the first line, continue at 50 mm/s with the second line. No acceleration is needed.
     * Travel at 50 mm/s throughout the second line.
     * Decelerate to minimum planner speed because at the end of the planner, jerk is not used.
     */
    const TimeEstimateCalculator::Position destination_1(1000, 0, 0, 0);
    calculator.plan(destination_1, 50.0, PrintFeatureType::Infill);
    const TimeEstimateCalculator::Position destination_2(2000, 0, 0, 0);
    calculator.plan(destination_2, 50.0, PrintFeatureType::Infill);

    //Deceleration distance is 1/2 at² + vt. We decelerate at 50mm/s for almost a second, ending at a speed of MINIMUM_PLANNER_SPEED.
    const double t = (50.0 - MINIMUM_PLANNER_SPEED) / 50.0;
    const double decelerate_distance = 0.5 * 50.0 * t * t + MINIMUM_PLANNER_SPEED * t;
    const double cruise_distance = 2000.0 - decelerate_distance;

    const std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(
        Duration(cruise_distance / 50.0 + t), //2000mm at 50mm/s, then decelerate for almost one second.
        result[static_cast<size_t>(PrintFeatureType::Infill)],
        EPSILON
    );
}

void TimeEstimateCalculatorTest::singleLineNoJerk()
{
    calculator.setFirmwareDefaults(jerkless);

    /*
     * This line:
     * Accelerate from 0 to 50mm/s in one second.
     * At the end, decelerate from 50 to 0mm/s in one second again.
     * In the middle, cruise at 50mm/s.
     */
    const TimeEstimateCalculator::Position destination(1000, 0, 0, 0);
    calculator.plan(destination, 50.0, PrintFeatureType::Infill);

    //Distance needed to accelerate: 1/2 at² + vt. We accelerate at 50mm/s². No initial velocity, but we decelerate to MINIMUM_PLANNER_SPEED.
    const double accelerate_distance = 0.5 * 50 * 1 * 1 + 0 * 1;
    const double decelerate_t = (50.0 - MINIMUM_PLANNER_SPEED) / 50.0;
    const double decelerate_distance = 0.5 * 50.0 * decelerate_t * decelerate_t + MINIMUM_PLANNER_SPEED * decelerate_t;
    const double cruise_distance = 1000.0 - accelerate_distance - decelerate_distance;

    const std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(
        Duration(1.0 + cruise_distance / 50.0 + decelerate_t), //Accelerate, cruise, decelerate.
        result[static_cast<size_t>(PrintFeatureType::Infill)],
        EPSILON
    );
}

void TimeEstimateCalculatorTest::doubleLineNoJerk()
{
    calculator.setFirmwareDefaults(jerkless);

    /*
     * These lines:
     * Accelerate from 0 to 50mm/s in one second.
     * Cruise at 50mm/s to the halfway point. It won't need to decelerate since the next line is in the same direction.
     * Cruise at 50mm/s to just before the end.
     * Decelerate from 50 to 0mm/s in one second.
     */
    const TimeEstimateCalculator::Position destination_1(1000, 0, 0, 0);
    calculator.plan(destination_1, 50.0, PrintFeatureType::Infill);
    const TimeEstimateCalculator::Position destination_2(2000, 0, 0, 0);
    calculator.plan(destination_2, 50.0, PrintFeatureType::Infill);

    //Distance needed to accelerate: 1/2 at² + vt. We accelerate at 50mm/s². No initial velocity, but we decelerate to MINIMUM_PLANNER_SPEED.
    const double accelerate_distance = 0.5 * 50 * 1 * 1 + 0 * 1;
    const double decelerate_t = (50.0 - MINIMUM_PLANNER_SPEED) / 50.0;
    const double decelerate_distance = 0.5 * 50.0 * decelerate_t * decelerate_t + MINIMUM_PLANNER_SPEED * decelerate_t;
    const double cruise_distance = 2000.0 - accelerate_distance - decelerate_distance;

    const std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(
        Duration(1.0 + cruise_distance / 50.0 + decelerate_t), //Accelerate, cruise, decelerate.
        result[static_cast<size_t>(PrintFeatureType::Infill)],
        EPSILON
    );
}

void TimeEstimateCalculatorTest::diagonalLineNoJerk()
{
    calculator.setFirmwareDefaults(jerkless);

    /*
     * This line:
     * Total acceleration will be 50mm/s, since the minimum of X and Y acceleration is 50 (both are 50 in fact).
     * Accelerate the X and Y axes from 0 to 50mm/s in 1 second.
     * Cruise at 50mm/s towards the destination.
     * Decelerate the X and Y axes from 50mm/s to the minimum planner speed in less than one second.
     */
    const TimeEstimateCalculator::Position destination(1000, 1000, 0, 0);
    calculator.plan(destination, 50.0, PrintFeatureType::Infill);

    //Distance needed to accelerate: 1/2 at² + vt. We accelerate at 50mm/s². No initial velocity, but we decelerate to MINIMUM_PLANNER_SPEED.
    const double accelerate_distance = 0.5 * 50.0 * 1.0 * 1.0 + 0.0 * 1.0;
    const double decelerate_t = (50.0 - MINIMUM_PLANNER_SPEED) / 50.0;
    const double decelerate_distance = 0.5 * 50.0 * decelerate_t * decelerate_t + MINIMUM_PLANNER_SPEED * decelerate_t;
    const double cruise_distance = std::sqrt(2.0) * 1000.0 - accelerate_distance - decelerate_distance; //Thank you Mr. Pythagoras.

    const std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_DOUBLES_EQUAL(
        Duration(1.0 + cruise_distance / 50.0 + decelerate_t), //Accelerate, cruise, decelerate.
        result[static_cast<size_t>(PrintFeatureType::Infill)],
        EPSILON
    );
}

} //namespace cura