//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <numeric>

#include "TimeEstimateCalculatorTest.h"
#include "../src/PrintFeature.h" //We get time estimates per print feature.
#include "../src/settings/types/Duration.h"

namespace cura
{

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

    calculator.setFirmwareDefaults(um3);
}

void TimeEstimateCalculatorTest::addTime()
{
    calculator.addTime(2);
    std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_EQUAL(Duration(2.0), result[static_cast<size_t>(PrintFeatureType::NoneType)]);

    calculator.addTime(3); //Has to add up, not replace.
    result = calculator.calculate();
    CPPUNIT_ASSERT_EQUAL(Duration(5.0), result[static_cast<size_t>(PrintFeatureType::NoneType)]);

    calculator.addTime(-7);
    result = calculator.calculate();
    CPPUNIT_ASSERT_EQUAL(Duration(5.0), result[static_cast<size_t>(PrintFeatureType::NoneType)]); //Due to how Duration works, it can never go lower.
}

void TimeEstimateCalculatorTest::startWithZero()
{
    const std::vector<Duration> result = calculator.calculate();

    CPPUNIT_ASSERT_EQUAL(static_cast<size_t>(PrintFeatureType::NumPrintFeatureTypes), result.size());

    for (const Duration estimate : result)
    {
        CPPUNIT_ASSERT_EQUAL_MESSAGE("Time estimates must be zero before anything has been planned.", Duration(0.0), estimate);
    }
}

void TimeEstimateCalculatorTest::moveToCurrentLocation()
{
    const TimeEstimateCalculator::Position position(1000, 2000, 3000, 4000);
    calculator.setPosition(position);

    std::vector<Duration> result = calculator.calculate();
    Duration estimate = std::accumulate(result.begin(), result.end(), Duration(0.0));
    CPPUNIT_ASSERT_EQUAL_MESSAGE("setPosition should not add any time to the estimate.", Duration(0.0), estimate);

    calculator.plan(position, Velocity(10), PrintFeatureType::Infill);

    result = calculator.calculate();
    estimate = std::accumulate(result.begin(), result.end(), Duration(0.0));
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Moving to the same location as where you already were should not cost any time.", Duration(0.0), estimate);
}

} //namespace cura