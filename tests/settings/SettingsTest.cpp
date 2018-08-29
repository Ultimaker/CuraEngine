//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath> //For M_PI.

#include "SettingsTest.h"
#include "../src/settings/FlowTempGraph.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/settings/types/AngleRadians.h"
#include "../src/settings/types/AngleDegrees.h"
#include "../src/settings/types/Temperature.h"
#include "../src/settings/types/Velocity.h"
#include "../src/settings/types/Ratio.h"
#include "../src/settings/types/Duration.h"
#include "../src/utils/floatpoint.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SettingsTest);

void SettingsTest::addSettingStringTest()
{
    const std::string setting_value("The human body contains enough bones to make an entire skeleton.");
    settings.add("test_setting", setting_value);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'string' setting is not the same as the expected value!",
                                 setting_value, settings.get<std::string>("test_setting"));
}

void SettingsTest::addSettingIntTest()
{
    settings.add("test_setting", "42");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting is not the same as the expected value!",
                                 int(42), settings.get<int>("test_setting"));

    settings.add("test_setting", "-1");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting #2 is not the same as the expected value!",
                                 int(-1), settings.get<int>("test_setting"));
}

void SettingsTest::addSettingDoubleTest()
{
    settings.add("test_setting", "1234567.890");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'double' setting is not the same as the expected value!",
                                 double(1234567.89), settings.get<double>("test_setting"));
}

void SettingsTest::addSettingSizeTTest()
{
    settings.add("test_setting", "666");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'size_t' setting is not the same as the expected value!",
                                 size_t(666), settings.get<size_t>("test_setting"));
}

void SettingsTest::addSettingUnsignedIntTest()
{
    settings.add("test_setting", "69");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'unsigned int' setting is not the same as the expected value!",
                                 unsigned(69), settings.get<unsigned int>("test_setting"));
}

void SettingsTest::addSettingBoolTest()
{
    settings.add("test_setting", "true");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting is not the same as the expected value!",
                                 true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "on");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #2 is not the same as the expected value!",
                                 true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "yes");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #3 is not the same as the expected value!",
                                 true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "True");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #4 is not the same as the expected value!",
                                 true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "50");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "0");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #5 is not the same as the expected value!",
                                 false, settings.get<bool>("test_setting"));

    settings.add("test_setting", "false");
    CPPUNIT_ASSERT_EQUAL(false, settings.get<bool>("test_setting"));

    settings.add("test_setting", "False");
    CPPUNIT_ASSERT_EQUAL(false, settings.get<bool>("test_setting"));
}

void SettingsTest::addSettingExtruderTrainTest()
{
    // TODO: Do it when the implementation is done.
    CPPUNIT_ASSERT_MESSAGE("TODO: The value of the 'ExtruderTrain' setting is not the same as the expected value!",
                           false);
}

void SettingsTest::addSettingLayerIndexTest()
{
    settings.add("test_setting", "-4");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'LayerIndex' setting is not the same as the expected value!",
                                 LayerIndex(-4), settings.get<LayerIndex>("test_setting"));
}

void SettingsTest::addSettingCoordTTest()
{
    settings.add("test_setting", "8589934592"); //2^33, so this MUST be a 64-bit integer! (Or at least 33-bit, but those don't exist.)
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'coord_t' setting is not the same as the expected value!",
                                 coord_t(8589934592), settings.get<coord_t>("test_setting"));
}

void SettingsTest::addSettingAngleRadiansTest()
{
    settings.add("test_setting", "180");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'AngleRadians' setting is not the same as the expected value!",
                                         AngleRadians(M_PI), settings.get<AngleRadians>("test_setting"), DELTA); //180 degrees is 1 pi radians.

    settings.add("test_setting", "810");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(AngleRadians(M_PI / 2.0), settings.get<AngleRadians>("test_setting"), DELTA); //810 degrees in clock arithmetic is 90 degrees, which is 0.5 pi radians.
}

void SettingsTest::addSettingAngleDegreesTest()
{
    settings.add("test_setting", "4442.4");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'AngleDegrees' setting is not the same as the expected value!",
                                         AngleDegrees(122.4), settings.get<AngleDegrees>("test_setting"), DELTA);
}

void SettingsTest::addSettingTemperatureTest()
{
    settings.add("test_setting", "245.5");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Temperature' setting is not the same as the expected value!",
                                         Temperature(245.5), settings.get<Temperature>("test_setting"), DELTA);
}

void SettingsTest::addSettingVelocityTest()
{
    settings.add("test_setting", "12.345");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Velocity' setting is not the same as the expected value!",
                                         Velocity(12.345), settings.get<Velocity>("test_setting"), DELTA);

    settings.add("test_setting", "-78");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Velocity' setting #2 is not the same as the expected value!",
                                         Velocity(-78), settings.get<Velocity>("test_setting"), DELTA);
}

void SettingsTest::addSettingRatioTest()
{
    settings.add("test_setting", "1.618");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Ratio' setting is not the same as the expected value!",
                                         Ratio(1.618), settings.get<Ratio>("test_setting"), DELTA);
}

void SettingsTest::addSettingDurationTest()
{
    settings.add("test_setting", "1234.5678");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Duration' setting is not the same as the expected value!",
                                         Duration(1234.5678), settings.get<Duration>("test_setting"), DELTA);

    settings.add("test_setting", "-1234.5678");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Duration' setting #2 is not the same as the expected value!",
                                         Duration(0), settings.get<Duration>("test_setting"), DELTA);
}

void SettingsTest::addSettingFlowTempGraphTest()
{
    settings.add("test_setting", "[[1.50, 10.1],[ 25.1,40.4 ], [26.5,75], [50 , 100.10]]"); //Try various spacing and radixes.
    const FlowTempGraph flow_temp_graph = settings.get<FlowTempGraph>("test_setting");

    double stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting for flow #1 is not the same as the expected value!",
                                         79.272340425531914893617021276596, stored_temperature, DELTA); //75 + (100.10 - 75) * (30.5 - 26.5) / (50 - 26.5)

    stored_temperature = flow_temp_graph.getTemp(1, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting for flow #2 is not the same as the expected value!",
                                         10.1, stored_temperature, DELTA); //Flow too low - Return lower temperature in the graph.

    stored_temperature = flow_temp_graph.getTemp(80, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting for flow #3 is not the same as the expected value!",
                                         100.1, stored_temperature, DELTA); //Flow too high - Return higher temperature in the graph.

    stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, false);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting when not flow dependant temperature"
                                                 " is not the same as the expected value!",
                                         200.0, stored_temperature, DELTA);
}

void SettingsTest::addSettingFMatrix3x3Test()
{
    settings.add("test_setting", "[[1.0, 2.0, 3.3],[ 2 , 3.0 , 1.0],[3.0 ,1.0,2.0 ]]"); //Try various spacing and radixes.
    FMatrix3x3 flow_matrix = settings.get<FMatrix3x3>("test_setting");

    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, flow_matrix.m[0][0], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, flow_matrix.m[0][1], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.3, flow_matrix.m[0][2], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, flow_matrix.m[1][0], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, flow_matrix.m[1][1], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, flow_matrix.m[1][2], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, flow_matrix.m[2][0], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, flow_matrix.m[2][1], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, flow_matrix.m[2][2], DELTA);
}

void SettingsTest::addSettingVectorTest()
{
    settings.add("test_setting", "[0, 1, 1,2, 3 , 5,  8,13]");
    std::vector<int> vector_int = settings.get<std::vector<int>>("test_setting");
    std::vector<int> ground_truth = {0, 1, 1, 2, 3, 5, 8, 13};
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The resulting vector doesn't have the correct size.",
                                 ground_truth.size(), vector_int.size());
    for (size_t i = 0; i < ground_truth.size(); i++)
    {
        CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'vector<int>' setting is not the same as the expected value!",
                                             ground_truth[i], vector_int[i]);
    }

}

void SettingsTest::overwriteSettingTest()
{
    settings.add("test_setting", "P");
    settings.add("test_setting", "NP");
    CPPUNIT_ASSERT_MESSAGE("The two values to be inserted must be different!",
                           std::string("P") != std::string("NP")); //Test for whether the test case is correct.
    CPPUNIT_ASSERT_MESSAGE("The value of the 'int' setting must not be equals to the second value inserted!",
                           settings.get<std::string>("test_setting") != std::string("P"));
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting must be equals to the first value inserted!",
                                 std::string("NP"), settings.get<std::string>("test_setting"));
}

}
