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
    CPPUNIT_ASSERT_EQUAL(setting_value, settings.get<std::string>("test_setting"));
}

void SettingsTest::addSettingIntTest()
{
    settings.add("test_setting", "42");
    CPPUNIT_ASSERT_EQUAL(int(42), settings.get<int>("test_setting"));

    settings.add("test_setting", "-1");
    CPPUNIT_ASSERT_EQUAL(int(-1), settings.get<int>("test_setting"));
}

void SettingsTest::addSettingDoubleTest()
{
    settings.add("test_setting", "1234567.890");
    CPPUNIT_ASSERT_EQUAL(double(1234567.89), settings.get<double>("test_setting"));
}

void SettingsTest::addSettingSizeTTest()
{
    settings.add("test_setting", "666");
    CPPUNIT_ASSERT_EQUAL(size_t(666), settings.get<size_t>("test_setting"));
}

void SettingsTest::addSettingUnsignedIntTest()
{
    settings.add("test_setting", "69");
    CPPUNIT_ASSERT_EQUAL(unsigned(69), settings.get<unsigned int>("test_setting"));
}

void SettingsTest::addSettingBoolTest()
{
    settings.add("test_setting", "true");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "on");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "yes");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "True");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "50");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "0");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("0 should cast to false.",
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
    CPPUNIT_ASSERT_EQUAL_MESSAGE("LayerIndex settings start counting from 0, so subtract one.",
                                 LayerIndex(-5), settings.get<LayerIndex>("test_setting"));
}

void SettingsTest::addSettingCoordTTest()
{
    settings.add("test_setting", "8589934.592"); //2^33 microns, so this MUST be a 64-bit integer! (Or at least 33-bit, but those don't exist.)
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Coordinates must be entered in the setting as millimetres, but are converted to micrometres.",
                                 coord_t(8589934592), settings.get<coord_t>("test_setting"));
}

void SettingsTest::addSettingAngleRadiansTest()
{
    settings.add("test_setting", "180");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("180 degrees is 1 pi radians.",
                                         AngleRadians(M_PI), settings.get<AngleRadians>("test_setting"), DELTA);

    settings.add("test_setting", "810");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("810 degrees in clock arithmetic is 90 degrees, which is 0.5 pi radians.",
                                 AngleRadians(M_PI / 2.0), settings.get<AngleRadians>("test_setting"), DELTA);
}

void SettingsTest::addSettingAngleDegreesTest()
{
    settings.add("test_setting", "4442.4");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("4442.4 in clock arithmetic is 360 degrees.",
                                         AngleDegrees(122.4), settings.get<AngleDegrees>("test_setting"), DELTA);
}

void SettingsTest::addSettingTemperatureTest()
{
    settings.add("test_setting", "245.5");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Temperature(245.5), settings.get<Temperature>("test_setting"), DELTA);
}

void SettingsTest::addSettingVelocityTest()
{
    settings.add("test_setting", "12.345");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Velocity(12.345), settings.get<Velocity>("test_setting"), DELTA);

    settings.add("test_setting", "-78");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Velocity(-78), settings.get<Velocity>("test_setting"), DELTA);
}

void SettingsTest::addSettingRatioTest()
{
    settings.add("test_setting", "1.618");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("With ratios, the input is interpreted in percentages.",
                                         Ratio(0.01618), settings.get<Ratio>("test_setting"), DELTA);
}

void SettingsTest::addSettingDurationTest()
{
    settings.add("test_setting", "1234.5678");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(1234.5678), settings.get<Duration>("test_setting"), DELTA);

    settings.add("test_setting", "-1234.5678");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(0), settings.get<Duration>("test_setting"), DELTA);
}

void SettingsTest::addSettingFlowTempGraphTest()
{
    settings.add("test_setting", "[[1.50, 10.1],[ 25.1,40.4 ], [26.5,75], [50 , 100.10]]"); //Try various spacing and radixes.
    const FlowTempGraph flow_temp_graph = settings.get<FlowTempGraph>("test_setting");

    double stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(75.0 + (100.10 - 75.0) * (30.5 - 26.5) / (50.0 - 26.5), stored_temperature, DELTA);

    stored_temperature = flow_temp_graph.getTemp(1, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(10.1, stored_temperature, DELTA); //Flow too low - Return lower temperature in the graph.

    stored_temperature = flow_temp_graph.getTemp(80, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(100.1, stored_temperature, DELTA); //Flow too high - Return higher temperature in the graph.

    stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, false);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(200.0, stored_temperature, DELTA);
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
    CPPUNIT_ASSERT_EQUAL(ground_truth.size(), vector_int.size());
    for (size_t i = 0; i < ground_truth.size(); i++)
    {
        CPPUNIT_ASSERT_EQUAL(ground_truth[i], vector_int[i]);
    }

}

void SettingsTest::overwriteSettingTest()
{
    settings.add("test_setting", "P");
    settings.add("test_setting", "NP");
    CPPUNIT_ASSERT_MESSAGE("When overriding a setting, the original value was not changed.",
                           settings.get<std::string>("test_setting") != std::string("P"));
    CPPUNIT_ASSERT_EQUAL(std::string("NP"), settings.get<std::string>("test_setting"));
}

}
