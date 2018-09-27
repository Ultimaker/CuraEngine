//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TimeEstimateCalculatorTest.h"
#include "../src/PrintFeature.h" //We get time estimates per print feature.
#include "../src/settings/types/Duration.h"

namespace cura
{
void TimeEstimateCalculatorTest::setUp()
{
    //Reset the calculator, but not by using its reset() function. That would be broken if the reset() function is broken.
    calculator = TimeEstimateCalculator();
}

void TimeEstimateCalculatorTest::startWithZero()
{
    std::vector<Duration> result = calculator.calculate();

    CPPUNIT_ASSERT_EQUAL(static_cast<size_t>(PrintFeatureType::NumPrintFeatureTypes), result.size());

    for (const Duration estimate : result)
    {
        CPPUNIT_ASSERT_EQUAL_MESSAGE("Time estimates must be zero before anything has been planned.", Duration(0.0), estimate);
    }
}

} //namespace cura