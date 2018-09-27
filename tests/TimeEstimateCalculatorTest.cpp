//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

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
}

void TimeEstimateCalculatorTest::addTime()
{
    calculator.addTime(2);
    std::vector<Duration> result = calculator.calculate();
    CPPUNIT_ASSERT_EQUAL(Duration(2.0), result[static_cast<size_t>(PrintFeatureType::NoneType)]);

    calculator.addTime(3); //Has to add up, not replace.
    result = calculator.calculate();
    CPPUNIT_ASSERT_EQUAL(Duration(3.0), result[static_cast<size_t>(PrintFeatureType::NoneType)]);

    calculator.addTime(-7);
    result = calculator.calculate();
    CPPUNIT_ASSERT_EQUAL(Duration(0.0), result[static_cast<size_t>(PrintFeatureType::NoneType)]); //Due to how Duration works, it can never go below 0.
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

void TimeEstimateCalculatorTest::singleMoveX()
{
    constexpr double distance = 100;
    constexpr Velocity feed_rate = 20;
    constexpr PrintFeatureType feature = PrintFeatureType::Skin;
    calculator.plan(TimeEstimateCalculator::Position(distance, 0, 0, 0), feed_rate, feature); //Move to 100,0 with 20mm/s.

    //The jerk is by default set to 20.0, which is equal to our feedrate, so we expect to be at maximum velocity immediately.
    constexpr Duration expected_result = distance / feed_rate;

    CPPUNIT_ASSERT_EQUAL(Duration(5.0), calculator.calculate()[static_cast<size_t>(feature)]);
}

} //namespace cura